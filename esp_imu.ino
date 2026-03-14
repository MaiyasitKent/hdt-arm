#include <WiFi.h>
#include <WiFiMulti.h>
#include <WiFiUdp.h>
#include <Wire.h>
#include <Adafruit_BNO08x.h>

#define LED_PIN 2 

// =================(SetDevice)=================
#define DEVICE_IS_FOREARM 
// #define DEVICE_IS_WRIST 

#ifdef DEVICE_IS_FOREARM
  const int UDP_PORT = 8888;
  const String DEVICE_NAME = "FOREARM (Port 8888)";
#else
  const int UDP_PORT = 8889;
  const String DEVICE_NAME = "WRIST (Port 8889)";
#endif

// ================= Network Configuration =================
WiFiMulti wifiMulti;
String current_target_ip = "192.168.1.255"; 
String lastSSID = "";

void updateTargetIP() {
    String currentSSID = WiFi.SSID();
    
    if (currentSSID == "Worm_32.exe_2.5G") {
        current_target_ip = "192.168.1.255";
    } 
    else if (currentSSID == "Ju9Sork") {
        current_target_ip = "172.20.10.15"; 
    }
    // else if (currentSSID == "WiFi_3_Name") {
    //     current_target_ip = "192.168.0.255";
    // }
    // else if (currentSSID == "WiFi_4_Name") {
    //     current_target_ip = "192.168.1.255";
    // }
    else {
        current_target_ip = "192.168.1.255"; 
    }
    
    Serial.println("------------------------------------");
    Serial.print("Connected to: "); Serial.println(currentSSID);
    Serial.print("Target IP updated to: "); Serial.println(current_target_ip);
    Serial.println("------------------------------------");
}

WiFiUDP udp;
Adafruit_BNO08x bno08x(-1);
sh2_SensorValue_t sensorValue;

// Sensor
float qw=1, qx=0, qy=0, qz=0;
float ax=0, ay=0, az=0;
float gx=0, gy=0, gz=0;
float mx=0, my=0, mz=0;
float lx=0, ly=0, lz=0;

unsigned long lastSendTime = 0;

void setup() {
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);
  
  Serial.println("\n====================================");
  Serial.println("IMU Sensor Node : " + DEVICE_NAME);
  Serial.println("====================================");

  wifiMulti.addAP("Worm_32.exe_2.5G", "0E040E270E22");
  wifiMulti.addAP("Ju9Sork", "password");
  // wifiMulti.addAP("WiFi_3_Name", "WiFi_3_Password");
  // wifiMulti.addAP("WiFi_4_Name", "WiFi_4_Password");
  // wifiMulti.addAP("WiFi_5_Name", "WiFi_5_Password");

  Serial.println("[1/3] Connecting to WiFi Multi...");
  
  while (wifiMulti.run() != WL_CONNECTED) {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(500);
    Serial.print(".");
  }
  
  digitalWrite(LED_PIN, HIGH);
  updateTargetIP(); 

  Serial.println("[2/3] WiFi Initialized!");
  Serial.print("IP Address: "); Serial.println(WiFi.localIP());

  Serial.println("[3/3] Initializing BNO08x...");
  if (!bno08x.begin_I2C()) {
    Serial.println("Failed to find BNO08x! Check wiring.");
    while (1) { 
      digitalWrite(LED_PIN, !digitalRead(LED_PIN)); 
      delay(100); 
    }
  }

  //(10,000 us = 100Hz)
  bno08x.enableReport(SH2_ROTATION_VECTOR, 10000);
  bno08x.enableReport(SH2_ACCELEROMETER, 10000);
  bno08x.enableReport(SH2_GYROSCOPE_CALIBRATED, 10000);
  bno08x.enableReport(SH2_MAGNETIC_FIELD_CALIBRATED, 10000);
  bno08x.enableReport(SH2_LINEAR_ACCELERATION, 10000);
  
  Serial.println("All Systems GO! Sending data...");
}

void loop() {
  // ตรวจสอบการเชื่อมต่อ WiFi และอัปเดต IP อัตโนมัติเมื่อ SSID เปลี่ยน
  if (wifiMulti.run() != WL_CONNECTED) {
    digitalWrite(LED_PIN, LOW);
    return; 
  }
  
  digitalWrite(LED_PIN, HIGH);

  // ถ้าเปลี่ยน WiFi ให้เปลี่ยน Target IP ทันที
  if (WiFi.SSID() != lastSSID) {
    updateTargetIP();
    lastSSID = WiFi.SSID();
  }

  // get data from BNO08x
  if (bno08x.getSensorEvent(&sensorValue)) {
    switch (sensorValue.sensorId) {
      case SH2_ROTATION_VECTOR:
        qw = sensorValue.un.rotationVector.real;
        qx = sensorValue.un.rotationVector.i;
        qy = sensorValue.un.rotationVector.j;
        qz = sensorValue.un.rotationVector.k;
        break;
      case SH2_ACCELEROMETER:
        ax = sensorValue.un.accelerometer.x; ay = sensorValue.un.accelerometer.y; az = sensorValue.un.accelerometer.z;
        break;
      case SH2_GYROSCOPE_CALIBRATED:
        gx = sensorValue.un.gyroscope.x; gy = sensorValue.un.gyroscope.y; gz = sensorValue.un.gyroscope.z;
        break;
      case SH2_MAGNETIC_FIELD_CALIBRATED:
        mx = sensorValue.un.magneticField.x; my = sensorValue.un.magneticField.y; mz = sensorValue.un.magneticField.z;
        break;
      case SH2_LINEAR_ACCELERATION:
        lx = sensorValue.un.linearAcceleration.x; ly = sensorValue.un.linearAcceleration.y; lz = sensorValue.un.linearAcceleration.z;
        break;
    }
  }

  // ส่งข้อมูลทุก 10ms (100Hz)
  // if (millis() - lastSendTime >= 10) {
  //   lastSendTime = millis();
  //   char buffer[300];
    
  //   snprintf(buffer, sizeof(buffer), "%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f",
  //            qw, qx, qy, qz, ax, ay, az, gx, gy, gz, mx, my, mz, lx, ly, lz);
    
  //   udp.beginPacket(current_target_ip.c_str(), UDP_PORT);
  //   udp.print(buffer);
  //   udp.endPacket();
  // }

  float target_hz = 10.0; // hz setting
  unsigned long interval = 1000 / target_hz; 

  if (millis() - lastSendTime >= interval) {
    lastSendTime = millis();
    char buffer[300];
    
    // sent (Quaternion, Accel, Gyro, Mag, Linear Accel)
    snprintf(buffer, sizeof(buffer), "%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f",
             qw, qx, qy, qz, ax, ay, az, gx, gy, gz, mx, my, mz, lx, ly, lz);
    
    udp.beginPacket(current_target_ip.c_str(), UDP_PORT);
    udp.print(buffer);
    udp.endPacket();
  }
}