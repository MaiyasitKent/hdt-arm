/**
 * imu_node.ino — BNO08x IMU UDP Sender (optimized)
 *
 * Changes from original:
 *   [F1] ลบ double rate-limiting: BNO08x 200Hz → ส่งทันทีทุก rotation event
 *        ไม่ใช้ millis() throttle อีกต่อไป
 *   [F2] ส่งเฉพาะ quaternion (4 floats = 20 bytes) แทน 16 floats (64+ bytes)
 *        ลด packet size → WiFi transmit time ลดลง
 *   [F3] event-driven: ส่งทันทีเมื่อได้ SH2_ROTATION_VECTOR ใหม่
 *        ไม่รอ interval timer
 *   [F4] ปิด sensor reports ที่ไม่จำเป็น (accel, gyro, mag, linear_accel)
 *        ลด I2C bus load → BNO08x มีเวลาประมวลผล rotation มากขึ้น
 *   [F5] UDP packet format เปลี่ยนเป็น binary float (16 bytes)
 *        แทน ASCII string → ลด parse overhead ทั้งสองฝั่ง
 *
 * UDP Packet Format (16 bytes, little-endian):
 *   [0-3]  float qw
 *   [4-7]  float qx
 *   [8-11] float qy
 *   [12-15] float qz
 *
 * ====================================================
 * SetDevice: เปิด define ที่ต้องการ
 * ====================================================
 */

#include <WiFi.h>
#include <WiFiMulti.h>
#include <WiFiUdp.h>
#include <Wire.h>
#include <Adafruit_BNO08x.h>

#define LED_PIN 2

// =================(SetDevice)=================
// #define DEVICE_IS_UPPERARM
#define DEVICE_IS_FOREARM

#ifdef DEVICE_IS_UPPERARM
  const int    UDP_PORT   = 8888;
  const String DEVICE_NAME = "UPPERARM (Port 8888)";
#else
  const int    UDP_PORT   = 8889;
  const String DEVICE_NAME = "FOREARM (Port 8889)";
#endif

// ================= Network =================
WiFiMulti wifiMulti;
WiFiUDP   udp;
String    current_target_ip = "192.168.1.255";
String    lastSSID           = "";

void updateTargetIP() {
  String ssid = WiFi.SSID();
  if      (ssid == "Worm_32.exe_2.5G") current_target_ip = "192.168.1.255";
  else if (ssid == "Ju9Sork")          current_target_ip = "172.20.10.15";
  else                                  current_target_ip = "192.168.1.255";

  Serial.println("------------------------------------");
  Serial.print("Connected to: ");   Serial.println(ssid);
  Serial.print("Target IP: ");      Serial.println(current_target_ip);
  Serial.println("------------------------------------");
}

// ================= IMU =================
Adafruit_BNO08x  bno08x(-1);
sh2_SensorValue_t sensorValue;

// [F5] Binary packet: 4 floats = 16 bytes
struct __attribute__((packed)) ImuPacket {
  float qw, qx, qy, qz;
};

void setup() {
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);

  Serial.println("\n====================================");
  Serial.println("IMU Sensor Node : " + DEVICE_NAME);
  Serial.println("====================================");

  wifiMulti.addAP("Worm_32.exe_2.5G", "0E040E270E22");
  wifiMulti.addAP("Ju9Sork", "password");

  Serial.println("[1/3] Connecting to WiFi...");
  while (wifiMulti.run() != WL_CONNECTED) {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(500);
    Serial.print(".");
  }
  digitalWrite(LED_PIN, HIGH);
  updateTargetIP();
  Serial.println("[2/3] WiFi OK — IP: " + WiFi.localIP().toString());

  Serial.println("[3/3] Initializing BNO08x...");
  if (!bno08x.begin_I2C()) {
    Serial.println("BNO08x not found! Check wiring.");
    while (1) { digitalWrite(LED_PIN, !digitalRead(LED_PIN)); delay(100); }
  }

  // [F4] เปิดเฉพาะ rotation vector — ลด I2C bus load
  // [F1] 5000us = 200Hz (เดิมมี millis() throttle เหลือ 100Hz)
  bno08x.enableReport(SH2_ROTATION_VECTOR, 5000);

  Serial.println("All Systems GO!");
  Serial.println("Sending quaternion-only packets at 200Hz...");
}

void loop() {
  // reconnect หาก WiFi หลุด
  if (wifiMulti.run() != WL_CONNECTED) {
    digitalWrite(LED_PIN, LOW);
    return;
  }
  digitalWrite(LED_PIN, HIGH);

  // update target IP ถ้า SSID เปลี่ยน
  if (WiFi.SSID() != lastSSID) {
    updateTargetIP();
    lastSSID = WiFi.SSID();
  }

  // [F3] event-driven: ส่งทันทีเมื่อได้ rotation vector ใหม่
  // ไม่รอ millis() interval อีกต่อไป
  if (bno08x.getSensorEvent(&sensorValue)) {
    if (sensorValue.sensorId == SH2_ROTATION_VECTOR) {

      // [F5] binary packet 16 bytes แทน ASCII 64+ bytes
      ImuPacket pkt;
      pkt.qw = sensorValue.un.rotationVector.real;
      pkt.qx = sensorValue.un.rotationVector.i;
      pkt.qy = sensorValue.un.rotationVector.j;
      pkt.qz = sensorValue.un.rotationVector.k;

      udp.beginPacket(current_target_ip.c_str(), UDP_PORT);
      udp.write((uint8_t*)&pkt, sizeof(pkt));
      udp.endPacket();
    }
    // [F4] ignore sensor events อื่นๆ ทั้งหมด (ไม่ได้ enable แล้ว)
  }
}
