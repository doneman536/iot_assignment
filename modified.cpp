#define BLYNK_TEMPLATE_ID "TMPL69AbetbbM"
#define BLYNK_TEMPLATE_NAME "IoT Fall Detection System"
#define BLYNK_AUTH_TOKEN "jCW4zdCr-jwtLe28r0fDji35x2G6BlMx"

#define BLYNK_PRINT Serial
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <BlynkSimpleEsp32.h>
#include <PubSubClient.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <TinyGPSPlus.h>
#include <HardwareSerial.h>
#include <math.h>

// ---- Blynk + WiFi ----
char auth[] = BLYNK_AUTH_TOKEN;
char ssid[] = "Wokwi-GUEST";
char pass[] = "";

// ---- MQTT (HiveMQ Cloud) ----
const char* mqtt_server = "6b8727ba6b3640e9ad7f0a72a199ae2f.s1.eu.hivemq.cloud";
const int mqtt_port = 8883;
const char* mqtt_user = "root1234";
const char* mqtt_password = "Root1234";
const char* topic_alert = "fall/alert";
const char* topic_health = "fall/health";
const char* topic_gps = "fall/location";

WiFiClientSecure espClient;
PubSubClient mqttClient(espClient);
BlynkTimer timer;
Adafruit_MPU6050 mpu;

// ---- GPS ----
TinyGPSPlus gps;
HardwareSerial SerialGPS(1); // Use UART1
#define RXD2 16
#define TXD2 17

// ---- Pins ----
#define LED_PIN 5
#define PULSE_PIN 35

// ---- Thresholds ----
const float SOFT_THRESHOLD = 1.0 * 9.8;
const float HARD_THRESHOLD = 2.0 * 9.8;
int minHeartRate = 45;
int maxHeartRate = 95;

// ---- Variables ----
int heartRate = 75;
float latitude = 0.0;
float longitude = 0.0;

// -------------------- MQTT --------------------
void reconnectMQTT() {
  while (!mqttClient.connected()) {
    Serial.print("Connecting to MQTT...");
    if (mqttClient.connect("ESP32Client", mqtt_user, mqtt_password)) {
      Serial.println("Connected to HiveMQ Cloud!");
    } else {
      Serial.print("failed, rc=");
      Serial.print(mqttClient.state());
      delay(3000);
    }
  }
}

// -------------------- LED Blink --------------------
void blinkLED(int times, int delayMs) {
  for (int i = 0; i < times; i++) {
    digitalWrite(LED_PIN, HIGH);
    delay(delayMs);
    digitalWrite(LED_PIN, LOW);
    delay(delayMs);
  }
}

// -------------------- Pulse Reading --------------------
int readPulse() {
  int rawValue = analogRead(PULSE_PIN);
  int bpm = map(rawValue, 0, 4095, 60, 130);
  Serial.print("Heart Rate: ");
  Serial.print(bpm);
  Serial.println(" bpm");

  // Blynk + MQTT
  Blynk.virtualWrite(V4, bpm);
  char msg[50];
  snprintf(msg, sizeof(msg), "Heart Rate: %d bpm", bpm);
  mqttClient.publish(topic_health, msg);

  return bpm;
}

// -------------------- GPS Reading --------------------
void readGPS() {
  while (SerialGPS.available() > 0) {
    gps.encode(SerialGPS.read());
  }

  if (gps.location.isValid()) {
    latitude = gps.location.lat();
    longitude = gps.location.lng();

    Serial.print("GPS Location: ");
    Serial.print(latitude, 6);
    Serial.print(", ");
    Serial.println(longitude, 6);

    // Send to Blynk
    Blynk.virtualWrite(V7, latitude);
    Blynk.virtualWrite(V8, longitude);

    // Send to MQTT
    char msg[80];
    snprintf(msg, sizeof(msg), "GPS: %.6f, %.6f", latitude, longitude);
    mqttClient.publish(topic_gps, msg);
  } else {
    Serial.println("Waiting for GPS signal...");
  }
}

// -------------------- Fall + Pulse Detection --------------------
void sendSensor() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  float ax = a.acceleration.x;
  float ay = a.acceleration.y;
  float az = a.acceleration.z;
  float totalAcc = sqrt(ax * ax + ay * ay + az * az);

  Serial.print("Total Acceleration: ");
  Serial.print(totalAcc);
  Serial.println(" m/s^2");

  Blynk.virtualWrite(V0, ax);
  Blynk.virtualWrite(V1, ay);
  Blynk.virtualWrite(V2, az);
  Blynk.virtualWrite(V6, totalAcc);

  // Read pulse and GPS
  heartRate = readPulse();
  readGPS();

  // ---- Case 1: Critical Fall ----
  if (totalAcc > HARD_THRESHOLD && (heartRate < minHeartRate || heartRate > maxHeartRate)) {
    Serial.println("🚨 CRITICAL FALL DETECTED! (Abnormal Heart Rate)");
    Blynk.logEvent("fall_detected", "🚨 CRITICAL FALL DETECTED! Check pulse!");
    mqttClient.publish(topic_alert, "🚨 CRITICAL FALL DETECTED! (Abnormal Pulse)");
    blinkLED(6, 150);
  }
  // ---- Case 2: Regular Fall ----
  else if (totalAcc > HARD_THRESHOLD) {
    Serial.println("⚠️ FALL DETECTED!");
    Blynk.logEvent("fall_detected", "⚠️ Fall Detected!");
    mqttClient.publish(topic_alert, "⚠️ Fall Detected!");
    blinkLED(4, 200);
  }
  // ---- Case 3: Suspicious Movement ----
  else if (totalAcc > SOFT_THRESHOLD) {
    Serial.println("⚠️ Suspicious Movement Detected!");
    mqttClient.publish(topic_alert, "⚠️ Suspicious Movement Detected!");
    blinkLED(2, 300);
  }
}

// -------------------- SETUP --------------------
void setup() {
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  // MPU6050 Init
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050!");
    while (1) delay(10);
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  Serial.println("MPU6050 initialized.");

  // WiFi + Blynk
  WiFi.begin(ssid, pass);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println(" WiFi connected!");
  Blynk.begin(auth, ssid, pass);

  // MQTT
  espClient.setInsecure();
  mqttClient.setServer(mqtt_server, mqtt_port);
  reconnectMQTT();

  // GPS Init
  SerialGPS.begin(9600, SERIAL_8N1, RXD2, TXD2);
  Serial.println("GPS module initialized.");

  timer.setInterval(2000L, sendSensor); // every 2 seconds
}

// -------------------- LOOP --------------------
void loop() {
  if (!mqttClient.connected()) reconnectMQTT();
  mqttClient.loop();
  Blynk.run();
  timer.run();
}
