// ESP32 WiFi (router-based) UDP "hi" test — Phase 1 alternative to ESP-NOW.
// Both boards join the SAME WiFi network and broadcast a "hi" message to
// each other over UDP. Flash this SAME file on both boards, changing only
// ROBOT_ID each time, and fill in your WiFi credentials below.
//
// NOTE: if this doesn't work on college/public WiFi, it's very likely
// "client isolation" blocking device-to-device traffic on that network —
// test on a home WiFi or phone hotspot first to rule that out.

#include <WiFi.h>
#include <WiFiUdp.h>

#define ROBOT_ID 1  // <-- CHANGE THIS to 1 or 2 before each upload

const char* WIFI_SSID = "YOUR_WIFI_NAME";
const char* WIFI_PASSWORD = "YOUR_WIFI_PASSWORD";

const unsigned int UDP_PORT = 4210;
WiFiUDP udp;

char incomingPacket[64];

void setup() {
  Serial.begin(115200);
  delay(500);

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();
  Serial.print("Connected! IP address: ");
  Serial.println(WiFi.localIP());

  udp.begin(UDP_PORT);
  Serial.print("Robot ");
  Serial.print(ROBOT_ID);
  Serial.println(" ready. Sending 'hi' every 2 seconds.");
}

void loop() {
  // Broadcast so neither board needs to know the other's IP address ahead of time
  IPAddress broadcastIP(255, 255, 255, 255);
  char message[32];
  snprintf(message, sizeof(message), "hi from robot %d", ROBOT_ID);

  udp.beginPacket(broadcastIP, UDP_PORT);
  udp.print(message);
  udp.endPacket();

  // Check for an incoming message from the other robot
  int packetSize = udp.parsePacket();
  if (packetSize) {
    int len = udp.read(incomingPacket, sizeof(incomingPacket) - 1);
    if (len > 0) incomingPacket[len] = 0;

    // Ignore our own broadcast bouncing back to us
    char selfMsg[32];
    snprintf(selfMsg, sizeof(selfMsg), "hi from robot %d", ROBOT_ID);
    if (strcmp(incomingPacket, selfMsg) != 0) {
      Serial.print("Received: ");
      Serial.print(incomingPacket);
      Serial.print(" from ");
      Serial.println(udp.remoteIP());
    }
  }

  delay(2000);
}
