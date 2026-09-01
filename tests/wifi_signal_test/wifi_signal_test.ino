// ESP32 WiFi hardware test — confirms radio + antenna work correctly.
// Scans for nearby WiFi networks and prints SSID + signal strength (RSSI).
// This is a plain WiFi test, separate from ESP-NOW — just proves the radio
// hardware itself is working before you build anything on top of it.

#include <WiFi.h>

void setup() {
  Serial.begin(115200);
  delay(500);
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  delay(100);
  Serial.println("WiFi hardware test starting...");
}

void loop() {
  Serial.println("Scanning for networks...");
  int n = WiFi.scanNetworks();

  if (n == 0) {
    Serial.println("No networks found. If you expected some nearby, check the board/antenna.");
  } else {
    Serial.print(n);
    Serial.println(" networks found:");
    for (int i = 0; i < n; i++) {
      Serial.print(i + 1);
      Serial.print(": ");
      Serial.print(WiFi.SSID(i));
      Serial.print(" (RSSI: ");
      Serial.print(WiFi.RSSI(i));
      Serial.println(" dBm)");
    }
  }

  Serial.println("---");
  delay(5000);
}
