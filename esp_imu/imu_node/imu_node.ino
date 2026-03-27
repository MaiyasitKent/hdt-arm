/**
 * imu_node_serial_test.ino — Serial transport diagnostic build
 *
 * ส่งข้อมูลพร้อมกันสองช่องทาง:
 *   1. Serial USB  (เพื่อ diagnose)
 *   2. UDP         (ยังคงทำงานปกติ)
 *
 * Serial packet format (20 bytes):
 *   [0-3]   uint32_t seq       — sequence number สำหรับตรวจ drop
 *   [4-7]   float    qw
 *   [8-11]  float    qx
 *   [12-15] float    qy
 *   [16-19] float    qz
 *
 * วิธีใช้:
 *   1. flash firmware นี้
 *   2. รัน imu_serial_diag.py บน PC
 *   3. เทียบ jitter ระหว่าง Serial vs UDP ที่ ROS2 node รับ
 *
 * หลังทดสอบเสร็จ: ลบ SERIAL_DIAG define ออก หรือ comment กลับ
 */

#include <WiFi.h>
#include <WiFiMulti.h>
#include <WiFiUdp.h>
#include <Wire.h>
#include <Adafruit_BNO08x.h>

#define LED_PIN 2
#define SERIAL_DIAG          // comment บรรทัดนี้ออกเพื่อ build production
#define SERIAL_BAUD 115200   // สูงสุดที่ ESP32 USB-CDC รองรับ

// =================(SetDevice)=================
#define DEVICE_IS_UPPERARM
// #define DEVICE_IS_FOREARM

#ifdef DEVICE_IS_UPPERARM
  const int    UDP_PORT    = 8888;
  const String DEVICE_NAME = "UPPERARM (Port 8888)";
#else
  const int    UDP_PORT    = 8889;
  const String DEVICE_NAME = "FOREARM (Port 8889)";
#endif

// ================= Serial packet =================
struct __attribute__((packed)) SerialPacket {
  uint8_t  magic[2];   // { 0xAA, 0x55 }
  uint32_t seq;
  float    qw, qx, qy, qz;
};
// struct __attribute__((packed)) SerialPacket {
//   uint32_t seq;
//   float    qw, qx, qy, qz;
// };

// ================= UDP packet (ไม่เปลี่ยน) =================
struct __attribute__((packed)) ImuPacket {
  float qw, qx, qy, qz;
};

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
}

// ================= IMU =================
Adafruit_BNO08x   bno08x(-1);
sh2_SensorValue_t sensorValue;
uint32_t          seq = 0;

void setup() {
  Serial.begin(SERIAL_BAUD);
  pinMode(LED_PIN, OUTPUT);

  wifiMulti.addAP("Worm_32.exe_2.5G", "0E040E270E22");
  wifiMulti.addAP("Ju9Sork", "password");

  while (wifiMulti.run() != WL_CONNECTED) {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(500);
  }
  digitalWrite(LED_PIN, HIGH);
  updateTargetIP();

  if (!bno08x.begin_I2C()) {
    while (1) { digitalWrite(LED_PIN, !digitalRead(LED_PIN)); delay(100); }
  }
  bno08x.enableReport(SH2_ROTATION_VECTOR, 20000); // 50Hz
}

void loop() {
  // WiFi check (ไม่เปลี่ยน)
  if (wifiMulti.run() != WL_CONNECTED) {
    digitalWrite(LED_PIN, LOW);
    return;
  }
  digitalWrite(LED_PIN, HIGH);

  if (WiFi.SSID() != lastSSID) {
    updateTargetIP();
    lastSSID = WiFi.SSID();
  }

  if (bno08x.getSensorEvent(&sensorValue)) {
    if (sensorValue.sensorId == SH2_ROTATION_VECTOR) {

      float qw = sensorValue.un.rotationVector.real;
      float qx = sensorValue.un.rotationVector.i;
      float qy = sensorValue.un.rotationVector.j;
      float qz = sensorValue.un.rotationVector.k;

      // --- UDP (ไม่เปลี่ยน) ---
      ImuPacket udpPkt = { qw, qx, qy, qz };
      udp.beginPacket(current_target_ip.c_str(), UDP_PORT);
      udp.write((uint8_t*)&udpPkt, sizeof(udpPkt));
      udp.endPacket();

#ifdef SERIAL_DIAG
      // --- Serial binary (เพิ่มเติม) ---
      // seq ช่วยให้ Python script ตรวจ drop packet ได้
      SerialPacket sPkt;
      sPkt.magic[0] = 0xAA;
      sPkt.magic[1] = 0x55;
      sPkt.seq = seq++;
      sPkt.qw = qw; sPkt.qx = qx;
      sPkt.qy = qy; sPkt.qz = qz;
      Serial.write((uint8_t*)&sPkt, sizeof(sPkt));  // 22 bytes
#endif
    }
  }
}