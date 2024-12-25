#include <Wire.h>
#include <Adafruit_BMP280.h>
#include <LiquidCrystal_I2C.h>
#include <DHT.h>
#include "MAX30105.h"
#include "heartRate.h"
#include "spo2_algorithm.h"
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>  // Import ArduinoJson

#include "secrets.h"  // Include secrets.h to access credentials and certificates

// Define I2C addresses
#define LCD_ADDRESS 0x27
#define BMP280_ADDRESS 0x76

// Define I2C pins for BMP280 and MAX30102
#define SDA_PIN 8
#define SCL_PIN 9

LiquidCrystal_I2C lcd(LCD_ADDRESS, 16, 2);
Adafruit_BMP280 bmp280;
MAX30105 particleSensor;

// Define pins for DHT11 and SW420
#define DHTPIN 4      // Pin connected to DHT11 sensor
#define DHTTYPE DHT11 // Type of DHT sensor used
#define SW420_PIN 17  // Pin connected to SW420 sensor

DHT dht(DHTPIN, DHTTYPE);

// WiFi & MQTT Configuration
WiFiClientSecure net;
PubSubClient client(net);

// Buffer for sensor data
#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__)
uint16_t irBuffer[100]; // infrared LED sensor data
uint16_t redBuffer[100]; // red LED sensor data
#else
uint32_t irBuffer[100]; // infrared LED sensor data
uint32_t redBuffer[100]; // red LED sensor data
#endif

const byte RATE_SIZE = 4; // Increase this for more averaging. 4 is good.
byte rates[RATE_SIZE]; // Array of heart rates
byte rateSpot = 0;
long lastBeat = 0; // Time at which the last beat occurred
float beatsPerMinute;
int beatAvg;

int32_t bufferLength; // data length
int32_t spo2; // SPO2 value
int8_t validSPO2; // indicator to show if the SPO2 calculation is valid
int32_t heartRate; // heart rate value
int8_t validHeartRate; // indicator to show if the heart rate calculation is valid

const char* publishTopic = "kelompok1pubs";  // Topic for publishing data
const char* subscribeTopic = "kelompok1subs";  // Topic for subscribing

void connectAWS() {
  // Connect to Wi-Fi using credentials from secrets.h
  WiFi.mode(WIFI_STA);  // Set Wi-Fi mode to Station (client mode)
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);  // Corrected WiFi.begin usage
  Serial.println("Connecting to WiFi...");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("Connected to WiFi!");

  // Set up the secure client for AWS IoT using certificates from secrets.h
  net.setCACert(AWS_CERT_CA);
  net.setCertificate(AWS_CERT_CRT);
  net.setPrivateKey(AWS_CERT_PRIVATE);

  client.setServer(AWS_IOT_ENDPOINT, 8883);
  client.setCallback(messageHandler);

  Serial.println("Connecting to AWS IoT...");
  while (!client.connected()) {
    if (client.connect(THINGNAME)) {
      Serial.println("Connected to AWS IoT!");
      client.subscribe(subscribeTopic);  // Subscribe to kelompok1subs
    } else {
      Serial.print(".");
      delay(100);
    }
  }
}

void messageHandler(char* topic, byte* payload, unsigned int length) {
  // Handle incoming messages
  Serial.print("Incoming message on topic: ");
  Serial.println(topic);
}

void publishMessage(float temperature_dht, float humidity, float temperature_bmp, float pressure, int beatAvg, int spo2, bool getaranDetected) {
  StaticJsonDocument<200> doc;
  doc["temperature_dht"] = temperature_dht;
  doc["humidity"] = humidity;
  doc["temperature_bmp"] = temperature_bmp;
  doc["pressure"] = pressure;
  doc["beatAvg"] = beatsPerMinute; // This is BPM
  doc["spO2"] = spo2;
  doc["getaran"] = getaranDetected ? "Detected" : "Not Detected";

  char jsonBuffer[512];
  serializeJson(doc, jsonBuffer);

  Serial.print("Sending data to AWS: ");
  Serial.println(jsonBuffer); // Debug: Print the JSON data being sent

  // Publish to kelompok1pubs topic
  if (client.publish(publishTopic, jsonBuffer)) {
    Serial.println("Data successfully sent to AWS!"); // Debug: Success message
  } else {
    Serial.println("Failed to send data to AWS."); // Debug: Failure message
  }
}

void readDHTSensor(float &temperature_dht, float &humidity) {
  temperature_dht = dht.readTemperature();
  humidity = dht.readHumidity();
  if (isnan(temperature_dht) || isnan(humidity)) {
    Serial.println("Failed to read from DHT sensor!");
    temperature_dht = 0;
    humidity = 0;
  }
}

void readBMP280Sensor(float &temperature_bmp, float &pressure) {
  temperature_bmp = bmp280.readTemperature();
  pressure = bmp280.readPressure() / 100.0F;
  delay(2000);  // Delay for BMP280 sensor
}

void readMAX30102Sensor() {
  bufferLength = 100; // buffer length of 100 stores 4 seconds of samples running at 25sps

  // Read the first 100 samples
  for (byte i = 0; i < bufferLength; i++) {
    while (particleSensor.available() == false) // do we have new data?
      particleSensor.check(); // Check the sensor for new data

    redBuffer[i] = particleSensor.getRed();
    irBuffer[i] = particleSensor.getIR();
    particleSensor.nextSample(); // We're finished with this sample so move to next sample
  }

  // Calculate heart rate and SpO2 after first 100 samples
  maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);

  // Continuously taking samples from MAX30105
  for (byte i = 0; i < 25; i++) {
    while (particleSensor.available() == false) // do we have new data?
      particleSensor.check(); // Check the sensor for new data

    redBuffer[i] = particleSensor.getRed();
    irBuffer[i] = particleSensor.getIR();
    particleSensor.nextSample(); // We're finished with this sample so move to next sample

    // Check for heart beat
    long irValue = particleSensor.getIR();
    if (checkForBeat(irValue) == true) {
      long delta = millis() - lastBeat;
      lastBeat = millis();
      beatsPerMinute = 60 / (delta / 1000.0);

      if (beatsPerMinute < 255 && beatsPerMinute > 20) {
        rates[rateSpot++] = (byte)beatsPerMinute; // Store this reading in the array
        rateSpot %= RATE_SIZE; // Wrap variable

        // Take average of readings
        beatAvg = 0;
        for (byte x = 0; x < RATE_SIZE; x++)
          beatAvg += rates[x];
        beatAvg /= RATE_SIZE;
      }
    }
  }

  // After gathering new samples recalculate HR and SpO2
  maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);
  delay(2000);  // Delay for MAX30102 sensor
}

// Function to read SW-420 sensor
bool readSW420Sensor() {
  bool getaranDetected = digitalRead(SW420_PIN) == LOW;
  // Display vibration status on Serial Monitor
  if (getaranDetected) {
    Serial.println("Getaran terdeteksi!");
  } else {
    Serial.println("Tidak ada getaran.");
  }
  return getaranDetected;
}

void setup() {
  Serial.begin(115200);
  Wire.begin(SDA_PIN, SCL_PIN);  // Set I2C pins
  lcd.init();
  lcd.backlight();
  dht.begin();

  if (!bmp280.begin(BMP280_ADDRESS)) {
    Serial.println("BMP280 not found!");
    while (1);
  }

  pinMode(SW420_PIN, INPUT);

  if (!particleSensor.begin()) {
    Serial.println("MAX30102 not found!");
    while (1);
  }

  particleSensor.setup(60, 4, 2, 100, 411, 4096); // LED brightness, sample average, LED mode, sample rate, pulse width, ADC range
  particleSensor.setPulseAmplitudeRed(0x1F); // Increase the pulse amplitude
  particleSensor.setPulseAmplitudeIR(0x1F);  // Increase the pulse amplitude ```cpp
  connectAWS();  // Connect to WiFi and AWS IoT
}

void loop() {
  // Read DHT11 sensor
  float temperature_dht, humidity;
  readDHTSensor(temperature_dht, humidity);

  // Read BMP280 sensor
  float temperature_bmp, pressure;
  readBMP280Sensor(temperature_bmp, pressure);

  // Read MAX30102 sensor
  readMAX30102Sensor();

  // Read SW420 sensor for vibration detection
  bool getaranDetected = readSW420Sensor(); // Call function to read SW420

  // Send sensor data to AWS
  publishMessage(temperature_dht, humidity, temperature_bmp, pressure, beatAvg, spo2, getaranDetected);

  // Handle MQTT communication
  client.loop();
  
  // Display on LCD
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("T1:");
  lcd.print(temperature_dht);
  lcd.print("C H:");
  lcd.print(humidity);
  lcd.print("%");

  lcd.setCursor(0, 1);
  lcd.print("T2:");
  lcd.print(temperature_bmp);
  lcd.print("C P:");
  lcd.print(pressure);
  lcd.print("hPa");

  lcd.setCursor(0, 2);
  lcd.print("BPM : ");
  lcd.print(beatAvg);
  lcd.print(" SpO2: ");
  lcd.print(spo2);

  // Display results in Serial Monitor vertically
  Serial.print("Temperature Tubuh: ");
  Serial.print(temperature_dht);
  Serial.println(" °C");
  Serial.print("Kelembapan: ");
  Serial.print(humidity);
  Serial.println(" %");
  Serial.print("Temperature Lingkungan: ");
  Serial.print(temperature_bmp);
  Serial.println(" °C");
  Serial.print("Tekanan Darah: ");
  Serial.print(pressure);
  Serial.println(" mmHg");
  Serial.print("Heart Rate: ");
  Serial.print(beatsPerMinute);
  Serial.println(" bpm");
  Serial.print("SpO2: ");
  Serial.print(spo2);
  Serial.println(" %");

  delay(500);  // Delay before reading sensors again (for DHT and SW420)
}
