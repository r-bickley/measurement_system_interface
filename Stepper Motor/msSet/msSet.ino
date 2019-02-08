// Function for setting microstepping mode
// A4988 Stepper Motor Driver
// Author: Matt Kramer, Team ME46

void msSet(int MS1_PIN = 10, int MS2_PIN = 11, int MS3_PIN = 12, int ms) {
  if (ms == 1) {
    digitalWrite(MS1_PIN, LOW);
    digitalWrite(MS2_PIN, LOW);
    digitalWrite(MS3_PIN, LOW);
  } else if (ms == 2) {
    digitalWrite(MS1_PIN, HIGH);
    digitalWrite(MS2_PIN, LOW);
    digitalWrite(MS3_PIN, LOW);
  } else if (ms == 4) {
    digitalWrite(MS1_PIN, LOW);
    digitalWrite(MS2_PIN, HIGH);
    digitalWrite(MS3_PIN, LOW);
  } else if (ms == 8) {
    digitalWrite(MS1_PIN, HIGH);
    digitalWrite(MS2_PIN, HIGH);
    digitalWrite(MS3_PIN, LOW);
  } else if (ms == 16) {
    digitalWrite(MS1_PIN, HIGH);
    digitalWrite(MS2_PIN, HIGH);
    digitalWrite(MS3_PIN, HIGH);
  }
}

void setup() {}

void loop() {}
