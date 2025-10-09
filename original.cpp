/*****************************************************
 *  IoT Based Fall Detection System (Base Paper Version)
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
float threshold = 1.8; // G-force threshold for fall detection
bool fallDetected = false;

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

  if (totalAcc > threshold && !fallDetected) {
    fallDetected = true;
    digitalWrite(buzzer, HIGH);
    Serial.println("⚠️ Fall Detected!");
    
    // Send alert to server or local MQTT/HTTP (example)
    WiFiClient client;
    if (client.connect("maker.ifttt.com", 80)) {
      String url = "/trigger/fall_detected/with/key/Your_IFTTT_Key";
      client.print(String("GET ") + url + " HTTP/1.1\r\n" +
                   "Host: maker.ifttt.com\r\n" + 
                   "Connection: close\r\n\r\n");
    }
  }

  if (digitalRead(napButton) == LOW) {
    Serial.println("False alert! Resetting...");
    fallDetected = false;
    digitalWrite(buzzer, LOW);
    delay(1000);
  }

  delay(200);
}
