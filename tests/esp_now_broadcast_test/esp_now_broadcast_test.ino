// ESP-NOW broadcast test — Phase 1, Step 4
// Flash this SAME file on all 3 boards, changing only ROBOT_ID each time.
// Goal: confirm each board can send AND receive broadcasts from the others
// before any route/conflict logic is added.

#include <WiFi.h>
#include <esp_now.h>

#define ROBOT_ID 1   // <-- CHANGE THIS to 1, 2, or 3 before each upload

typedef struct {
  int robot_id;
  char message[32];
} TestMessage;

TestMessage outgoing;
TestMessage incoming;

uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  // Uncomment to debug send failures:
  // Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Send OK" : "Send FAIL");
}

// NOTE: newer ESP32 Arduino cores (IDF5 / core 3.x) use this signature.
// If you get a compile error here, your installed core is older —
// see the fallback signature commented below.
void onDataRecv(const esp_now_recv_info_t *recv_info, const uint8_t *incomingData, int len) {
  memcpy(&incoming, incomingData, sizeof(incoming));
  if (incoming.robot_id != ROBOT_ID) {
    Serial.print("Received from Robot ");
    Serial.print(incoming.robot_id);
    Serial.print(": ");
    Serial.println(incoming.message);
  }
}

// --- Fallback for older ESP32 cores (uncomment this version and comment out
// the one above if the compiler complains about esp_now_recv_info_t) ---
// void onDataRecv(const uint8_t *mac_addr, const uint8_t *incomingData, int len) {
//   memcpy(&incoming, incomingData, sizeof(incoming));
//   if (incoming.robot_id != ROBOT_ID) {
//     Serial.print("Received from Robot ");
//     Serial.print(incoming.robot_id);
//     Serial.print(": ");
//     Serial.println(incoming.message);
//   }
// }

void setup() {
  Serial.begin(115200);
  delay(500);

  WiFi.mode(WIFI_STA);

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    return;
  }

  esp_now_register_send_cb(onDataSent);
  esp_now_register_recv_cb(onDataRecv);

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add broadcast peer");
    return;
  }

  Serial.print("Robot ");
  Serial.print(ROBOT_ID);
  Serial.println(" ready. Broadcasting every 2 seconds.");
}

void loop() {
  outgoing.robot_id = ROBOT_ID;
  snprintf(outgoing.message, sizeof(outgoing.message), "hello from robot %d", ROBOT_ID);

  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *)&outgoing, sizeof(outgoing));

  if (result != ESP_OK) {
    Serial.println("Send failed");
  }

  delay(2000);
}
