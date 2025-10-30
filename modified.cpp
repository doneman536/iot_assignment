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
#include <math.h>

char auth[] = BLYNK_AUTH_TOKEN;
char ssid[] = "Wokwi-GUEST";
char pass[] = "";

// MQTT (HiveMQ Cloud)
const char* mqtt_server = "6b8727ba6b3640e9ad7f0a72a199ae2f.s1.eu.hivemq.cloud";
const int mqtt_port = 8883;
const char* mqtt_user = "root1234";
const char* mqtt_password = "Root1234";
const char* topic_alert = "fall/alert";

WiFiClientSecure espClient;
PubSubClient mqttClient(espClient);
BlynkTimer timer;
Adafruit_MPU6050 mpu;

#define LED_PIN 5  // GPIO 5 -> Red LED

const float SOFT_THRESHOLD = 1.0 * 9.8;
const float HARD_THRESHOLD = 2.0 * 9.8;

float latitude = 17.3850;
float longitude = 78.4867;

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

void blinkLED(int times, int delayMs) {
  for (int i = 0; i < times; i++) {
    digitalWrite(LED_PIN, HIGH);
    delay(delayMs);
    digitalWrite(LED_PIN, LOW);
    delay(delayMs);
  }
}

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

  if (totalAcc > HARD_THRESHOLD) {
    Serial.println("⚠️ FALL DETECTED!");
    Blynk.logEvent("fall_detected", "⚠️ Fall Detected!");
    mqttClient.publish(topic_alert, "⚠️ Fall Detected!");
    blinkLED(5, 150); // Blink LED 5 times fast
  } 
  else if (totalAcc > SOFT_THRESHOLD) {
    Serial.println("⚠️ Suspicious Movement Detected!");
    mqttClient.publish(topic_alert, "⚠️ Suspicious Movement Detected!");
    blinkLED(3, 300); // Blink LED 3 times slower
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  // Initialize MPU6050
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

  // MQTT setup
  espClient.setInsecure();
  mqttClient.setServer(mqtt_server, mqtt_port);
  reconnectMQTT();

  timer.setInterval(200L, sendSensor);
}

void loop() {
  if (!mqttClient.connected()) reconnectMQTT();
  mqttClient.loop();
  Blynk.run();
  timer.run();
}
