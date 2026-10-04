#include <Wire.h>
#include <WiFi.h>
#include <WiFiUdp.h>

const byte MPU_ADDR = 0x68;
const byte TFLUNA_ADDR = 0x10;

const char* ssid = "00";
const char* password = "00000000";

const int UDP_PORT = 4210;
const int BUZZER_PIN = 5;   // D5 / GPIO5
int buzzerDistanceCM = 50;  // changed from website with: BUZZ <cm>

WiFiUDP udp;

int readTFLuna() {
  Wire.beginTransmission(TFLUNA_ADDR);
  Wire.write(0x00);
  if (Wire.endTransmission(false) != 0) return -1;

  Wire.requestFrom(TFLUNA_ADDR, (byte)2);
  if (Wire.available() < 2) return -1;

  byte lowByte = Wire.read();
  byte highByte = Wire.read();
  return (highByte << 8) | lowByte;
}

void checkUdpCommands() {
  int packetSize = udp.parsePacket();
  if (!packetSize) return;

  char cmd[64];
  int n = udp.read(cmd, sizeof(cmd) - 1);
  if (n <= 0) return;
  cmd[n] = '\0';

  if (strncmp(cmd, "BUZZ ", 5) == 0) {
    int cm = atoi(cmd + 5);
    buzzerDistanceCM = constrain(cm, 0, 800);
    Serial.print("Buzzer threshold set to ");
    Serial.print(buzzerDistanceCM);
    Serial.println(" cm");
  }
}

void setup() {
  Serial.begin(9600);
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);

  Wire.begin(21, 22);

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();
  Serial.println("MPU6500 started");

  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();
  Serial.println("WiFi connected!");
  Serial.print("ESP32 IP: ");
  Serial.println(WiFi.localIP());

  // Listen for commands from demo.py on the same UDP port.
  udp.begin(UDP_PORT);
}

void loop() {
  checkUdpCommands();

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 14, true);

  if (Wire.available() >= 14) {
    int16_t ax = Wire.read() << 8 | Wire.read();
    int16_t ay = Wire.read() << 8 | Wire.read();
    int16_t az = Wire.read() << 8 | Wire.read();
    int16_t temp = Wire.read() << 8 | Wire.read();
    int16_t gx = Wire.read() << 8 | Wire.read();
    int16_t gy = Wire.read() << 8 | Wire.read();
    int16_t gz = Wire.read() << 8 | Wire.read();

    int distance = readTFLuna();

    // 0 disables the distance buzzer.
    bool danger = distance > 0 && buzzerDistanceCM > 0 && distance <= buzzerDistanceCM;
    digitalWrite(BUZZER_PIN, danger ? HIGH : LOW);

    char data[120];
    snprintf(data, sizeof(data), "%d,%d,%d,%d,%d,%d,%d",
             ax, ay, az, gx, gy, gz, distance);

    Serial.println(data);

    udp.beginPacket("255.255.255.255", UDP_PORT);
    udp.print(data);
    udp.endPacket();
  }

  delay(100);
}
