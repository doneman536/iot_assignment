/*****************************************************
 *  IoT Based Fall Detection System (Modified Dual-Threshold Version)
 *  Hardware: Arduino UNO + ESP8266 + MPU6050 + Buzzer + Button
 *****************************************************/

#include <Wire.h>
#include <MPU6050.h>
#include <ESP8266WiFi.h>

MPU6050 mpu;

const char* ssid = "YourWiFi";
const char* password = "YourPassword";

WiFiClient client;

const int buzzer = D1;
const int napButton = D2;

// Dual thresholds
float softThreshold = 1.5; // Detect suspicious movement
float hardThreshold = 2.5; // Confirm actual fall
bool possibleFall = false;
bool confirmedFall = false;

unsigned long lastSoftTrigger = 0;
unsigned long softTimeout = 1500; // 1.5s window between soft & hard hit

void setup() {
  Serial.begin(9600);
  Wire.begin();
  mpu.initialize();

  pinMode(buzzer, OUTPUT);
  pinMode(napButton, INPUT_PULLUP);

  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nConnected to WiFi!");
}

void loop() {
  int16_t ax, ay, az;
  mpu.getAcceleration(&ax, &ay, &az);

  float accX = ax / 16384.0;
  float accY = ay / 16384.0;
  float accZ = az / 16384.0;

  float totalAcc = sqrt(accX * accX + accY * accY + accZ * accZ);

  // Detect possible fall (soft threshold)
  if (totalAcc > softThreshold && totalAcc < hardThreshold && !possibleFall) {
    possibleFall = true;
    lastSoftTrigger = millis();
    Serial.println("⚠️ Suspicious Movement Detected (Soft Threshold)");
  }

  // Confirm fall (hard threshold within 1.5s)
  if (possibleFall && totalAcc >= hardThreshold && !confirmedFall) {
    confirmedFall = true;
    Serial.println("🚨 Confirmed Fall Detected!");
    digitalWrite(buzzer, HIGH);

    // Send alert via IoT request
    if (client.connect("maker.ifttt.com", 80)) {
      String url = "/trigger/fall_detected/with/key/Your_IFTTT_Key";
      client.print(String("GET ") + url + " HTTP/1.1\r\n" +
                   "Host: maker.ifttt.com\r\n" + 
                   "Connection: close\r\n\r\n");
    }
  }

  // Reset if hard threshold not hit in time
  if (possibleFall && (millis() - lastSoftTrigger > softTimeout) && !confirmedFall) {
    possibleFall = false;
    Serial.println("⏳ No hard impact — soft alert cancelled.");
  }

  // False alarm reset
  if (digitalRead(napButton) == LOW) {
    possibleFall = false;
    confirmedFall = false;
    digitalWrite(buzzer, LOW);
    Serial.println("🟢 False Alert Cancelled.");
    delay(1000);
  }

  delay(200);
}
