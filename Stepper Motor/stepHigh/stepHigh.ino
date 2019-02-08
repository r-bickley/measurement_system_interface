// Send constant HIGH to step pin
// Used for checking current limit on driver
// NEMA 17 stepper motor
// A4988 driver
// Author: Matthew Krmaer, Team ME46

#define STEP_PIN 3

void setup() {
  pinMode(STEP_PIN, OUTPUT);
}

void loop() {
  digitalWrite(STEP_PIN, HIGH);
  while(true);
}
