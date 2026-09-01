// Minimal connection test — confirms the ESP32 is powered, connected via USB,
// and actually running your code (not just detected as a port).
// Onboard LED blinks and Serial prints a heartbeat every second.

#define LED_PIN 2  // most ESP32 DevKit boards have the onboard LED on GPIO2

void setup() {
  Serial.begin(115200);
  delay(500);
  pinMode(LED_PIN, OUTPUT);
  Serial.println("ESP32 board connected and running.");
}

void loop() {
  digitalWrite(LED_PIN, HIGH);
  Serial.println("heartbeat - board alive");
  delay(500);
  digitalWrite(LED_PIN, LOW);
  delay(500);
}
