/*
  =============================================================================
  ESP32 DOME SENSOR -> ARDUINO UNO R4 WIFI (WIRELESS UDP + USB SERIAL)
  =============================================================================
  Connects to the Arduino UNO R4 WiFi's built-in Wi-Fi Access Point
  ("DomeTracker_R4") and streams the 5 photodiode readings over UDP 20x/sec,
  while still printing S1,S2,S3,S4,S5 over USB Serial for the Web Dashboard!
  =============================================================================
*/

#include <WiFi.h>
#include <WiFiUdp.h>

// Must match the Access Point settings in the UNO R4 WiFi sketch
const char* AP_SSID = "DomeTracker_R4";
const char* AP_PASS = "solar1234";
const uint16_t R4_UDP_PORT = 4210;

WiFiUDP udp;

// 5 Photodiode MCP6004 channels on ESP32 ADC1 pins (work while Wi-Fi is active!)
const int N = 5;
const int PINS[N] = {36, 39, 34, 35, 32}; // S1 Top, S2 N, S3 E, S4 S, S5 W
const int OVERSAMPLE = 16;
const unsigned long PERIOD_MS = 50;       // 50 ms = 20 Hz
unsigned long lastWifiRetry = 0;

void connectToArduinoWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false); // Low-latency UDP streaming
  WiFi.setAutoReconnect(true);
  WiFi.begin(AP_SSID, AP_PASS);
}

void setup() {
  Serial.begin(115200);

  analogReadResolution(12);
  for (int i = 0; i < N; i++) {
    pinMode(PINS[i], INPUT);
    analogSetPinAttenuation(PINS[i], ADC_11db);
  }

  connectToArduinoWiFi();
  udp.begin(4211);

  Serial.println("# ESP32 Dome Sensor starting -> connecting to UNO R4 WiFi AP...");
}

void loop() {
  // Automatically retry connecting if UNO R4 WiFi powered up later or rebooted
  if (WiFi.status() != WL_CONNECTED && millis() - lastWifiRetry >= 4000) {
    lastWifiRetry = millis();
    WiFi.disconnect();
    WiFi.begin(AP_SSID, AP_PASS);
  }

  static unsigned long lastSend = 0;
  if (millis() - lastSend < PERIOD_MS) return;
  lastSend = millis();

  uint32_t v[N];
  for (int i = 0; i < N; i++) {
    uint32_t sum = 0;
    for (int k = 0; k < OVERSAMPLE; k++) {
      sum += analogRead(PINS[i]);
    }
    v[i] = sum / OVERSAMPLE;
  }

  char line[64];
  snprintf(line, sizeof(line), "%lu,%lu,%lu,%lu,%lu",
           (unsigned long)v[0], (unsigned long)v[1], (unsigned long)v[2],
           (unsigned long)v[3], (unsigned long)v[4]);

  // 1. Keep USB Serial working for your Web Dashboard (index.html)
  Serial.println(line);

  // 2. Send wirelessly to the Arduino UNO R4 WiFi Access Point (gateway IP)
  if (WiFi.status() == WL_CONNECTED) {
    IPAddress r4Ip = WiFi.gatewayIP();
    udp.beginPacket(r4Ip, R4_UDP_PORT);
    udp.print(line);
    udp.print('\n');
    udp.endPacket();
  }
}
