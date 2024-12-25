#include <WiFi.h>
#include <PubSubClient.h>
#include <DHT.h>

// WiFi Configuration
const char* ssid = "BUS.ID";          // Ganti dengan SSID WiFi Anda
const char* password = "clean123456";  // Ganti dengan password WiFi Anda

// ThingsBoard Configuration
const char* mqtt_server = "demo.thingsboard.io";  // Ganti jika menggunakan server ThingsBoard lain
const char* access_token = "P38o0Ofg9U40zOGRsPkX";   // Ganti dengan Access Token ThingsBoard device Anda

WiFiClient espClient;
PubSubClient client(espClient);

// Pin Configuration
const int ldrPin = 2;
const int rainPin = 5;
const int hallPin = 16;
const int trigPin = 17;
const int echoPin = 18;
const int dhtPin = 4;

#define DHT_TYPE DHT11
DHT dht(dhtPin, DHT_TYPE);

// Variables
unsigned long pulseCount = 0;
unsigned long rpm = 0;
unsigned long previousMillis = 0;
unsigned long lastPulseTime = 0;
const unsigned long debounceDelay = 5;

void setup_wifi() {
  delay(10);
  Serial.println();
  Serial.print("Connecting to WiFi...");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
  }

  Serial.println("\nWiFi connected");
}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Connecting to ThingsBoard... ");
    if (client.connect("ESP32_Client", access_token, NULL)) {
      Serial.println("connected");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}

void setup() {
  Serial.begin(115200);
  setup_wifi();
  client.setServer(mqtt_server, 1883);

  // Pin modes
  pinMode(ldrPin, INPUT);
  pinMode(hallPin, INPUT);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  dht.begin();
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  unsigned long currentMillis = millis();

  // Membaca data sensor
  int ldrState = digitalRead(ldrPin);
  int rainValue = analogRead(rainPin);

  if (digitalRead(hallPin) == LOW) {
    if (currentMillis - lastPulseTime > debounceDelay) {
      pulseCount++;
      lastPulseTime = currentMillis;
    }
  }
  if (currentMillis - previousMillis >= 1000) {
    previousMillis = currentMillis;
    rpm = pulseCount * 60;
    pulseCount = 0;
  }

  float humidity = dht.readHumidity();
  float temperature = dht.readTemperature();

  long duration, distance;
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  duration = pulseIn(echoPin, HIGH);
  distance = duration * 0.034 / 2;

  // Menyiapkan payload JSON
  String payload = "{";
  payload += "\"ldrState\":" + String(ldrState) + ",";
  payload += "\"rainValue\":" + String(rainValue) + ",";
  payload += "\"rpm\":" + String(rpm) + ",";
  payload += "\"temperature\":" + String(isnan(temperature) ? 0 : temperature) + ",";
  payload += "\"humidity\":" + String(isnan(humidity) ? 0 : humidity) + ",";
  payload += "\"distance\":" + String((distance >= 400 || distance <= 2) ? 0 : distance);
  payload += "}";

  // Mengirimkan data ke ThingsBoard
  client.publish("v1/devices/me/telemetry", payload.c_str());
  Serial.println("Data sent to ThingsBoard: " + payload);

  delay(5000); // Kirim data setiap 5 detik
}
