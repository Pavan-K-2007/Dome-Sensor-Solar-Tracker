/*
  =============================================================================
  COMPANION ESP32 TRANSMITTER -> ARDUINO UNO R4 WIFI (UART2 + OPTIONAL UDP)
  =============================================================================
  Reads the 5 BPW34 + MCP6004 dome sensor channels on ESP32 ADC1 pins and sends:
    S1,S2,S3,S4,S5\n
  simultaneously to:
    1. USB Serial  (115200) -> Web Dashboard (index.html) keeps working live!
    2. UART2 TX2   (GPIO 17, 115200) -> UNO R4 WiFi Pin 0 (RX) + Common GND
    3. Optional Wi-Fi UDP -> UNO R4 WiFi SoftAP ("DomeTracker_R4", 192.168.4.1:4210)
       (Set USE_WIFI_UDP 1 on both boards for a 0-wire wireless sensor link!)
  =============================================================================
*/

#define USE_WIFI_UDP 0

#if USE_WIFI_UDP
#include <WiFi.h>
#include <WiFiUdp.h>
const char* AP_SSID  = "DomeTracker_R4";
const char* AP_PASS  = "solar1234";
const char* R4_IP    = "192.168.4.1";
const uint16_t R4_PORT = 4210;
WiFiUDP udp;
#endif

const int N = 5;
const int PINS[N] = {36, 39, 34, 35, 32}; // S1 Top, S2 N, S3 E, S4 S, S5 W
const int ESP_TX2_PIN = 17;               // Connect to UNO R4 Pin 0 (RX) via level shifter
const int ESP_RX2_PIN = 16;               // Unused (one-way stream)
const int OVERSAMPLE = 16;
const unsigned long PERIOD_MS = 50;       // 20 Hz update rate

void setup() {
  Serial.begin(115200);
  Serial2.begin(115200, SERIAL_8N1, ESP_RX2_PIN, ESP_TX2_PIN);

  analogReadResolution(12);
  for (int i = 0; i < N; i++) {
    pinMode(PINS[i], INPUT);
    analogSetPinAttenuation(PINS[i], ADC_11db);
  }

#if USE_WIFI_UDP
  WiFi.mode(WIFI_STA);
  WiFi.begin(AP_SSID, AP_PASS);
  unsigned long t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 6000) {
    delay(200);
  }
  udp.begin(4211);
#endif

  delay(300);
  Serial.println("# ESP32 dome transmitter ready (USB + UART2 GPIO17)");
}

void loop() {
  static unsigned long lastSend = 0;
  if (millis() - lastSend < PERIOD_MS) return;
  lastSend = millis();

  char line[64];
  uint32_t v[N];
  for (int i = 0; i < N; i++) {
    uint32_t sum = 0;
    for (int k = 0; k < OVERSAMPLE; k++) {
      sum += analogRead(PINS[i]);
    }
    v[i] = sum / OVERSAMPLE;
  }

  snprintf(line, sizeof(line), "%lu,%lu,%lu,%lu,%lu",
           (unsigned long)v[0], (unsigned long)v[1], (unsigned long)v[2],
           (unsigned long)v[3], (unsigned long)v[4]);

  // 1. Stream to PC USB Serial (Web Dashboard compatible)
  Serial.println(line);

  // 2. Stream to UNO R4 WiFi over Hardware UART2 (GPIO 17)
  Serial2.println(line);

#if USE_WIFI_UDP
  // 3. Stream wirelessly to UNO R4 WiFi over UDP if connected
  if (WiFi.status() == WL_CONNECTED) {
    udp.beginPacket(R4_IP, R4_PORT);
    udp.print(line);
    udp.print('\n');
    udp.endPacket();
  }
#endif
}
