// TinyML test — a small 2-8-1 neural network trained offline (Python/numpy),
// running its forward pass directly on the ESP32. No TFLite Micro library
// needed — for a model this small, hand-rolled inference is simpler and
// avoids ESP32 library/version friction.
//
// This predicts "time to reach a node" from (distance, speed) — the same
// role this will later play in the dynamic priority / conflict-resolution
// logic, replacing the static robot-ID priority rule.
//
// Type two numbers in Serial Monitor: <distance> <speed>  e.g. "10 2"

#include <Arduino.h>

#define H 8  // hidden units

const float W1[2*H] = {0.050292f, -0.052842f, 0.287045f, 0.341791f, -0.214268f, 0.257751f, 0.350387f, 0.409182f, -0.281494f, -0.506169f, -0.620676f, -0.170087f, -0.930012f, -0.864398f, -0.564021f, -0.455853f};
const float b1[H]   = {0.000000f, 0.000000f, 0.166552f, 0.135517f, 0.000000f, 0.170846f, -0.199786f, 0.173833f};
const float W2[H]   = {-0.217704f, -0.126520f, 0.628202f, 0.578702f, -0.051414f, 1.054180f, -0.175518f, 0.442703f};
const float b2      = 0.037455f;

// Normalization constants — must match what the model was trained with
const float DISTANCE_SCALE = 30.0f;
const float SPEED_SCALE = 4.0f;
const float TIME_SCALE = 30.0f;

float predictETA(float distance, float speed) {
  float x1 = distance / DISTANCE_SCALE;
  float x2 = speed / SPEED_SCALE;

  float hidden[H];
  for (int j = 0; j < H; j++) {
    float z = x1 * W1[0*H + j] + x2 * W1[1*H + j] + b1[j];
    hidden[j] = z > 0 ? z : 0;  // ReLU
  }

  float out = b2;
  for (int j = 0; j < H; j++) {
    out += hidden[j] * W2[j];
  }

  float predicted_time = out * TIME_SCALE;
  return predicted_time > 0 ? predicted_time : 0;
}

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("TinyML ETA model ready.");
  Serial.println("Type: <distance> <speed>   e.g. 10 2");
}

void loop() {
  if (Serial.available()) {
    float distance = Serial.parseFloat();
    float speed = Serial.parseFloat();
    while (Serial.available()) Serial.read();

    if (distance <= 0 || speed <= 0) {
      Serial.println("Enter positive numbers, e.g. 10 2");
      return;
    }

    float eta = predictETA(distance, speed);
    Serial.print("distance=");
    Serial.print(distance);
    Serial.print(" speed=");
    Serial.print(speed);
    Serial.print(" -> predicted ETA: ");
    Serial.println(eta, 2);
  }
}
