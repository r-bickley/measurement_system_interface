// Function for manually jogging a stepper motor
// NEMA 17 stepper motor
// A4988 driver
// Author: Matthew Kramer, Team ME46

#define DIR_PIN     2
#define STEP_PIN    3
#define ENABLE_PIN  4
#include Keyboard.h

void setup() {
  pinMode(STEP_PIN,   OUTPUT);
  pinMode(DIR_PIN,    OUTPUT);
  pinMode(ENABLE_PIN, OUTPUT);
  Serial.begin(9600);
  Keyboard.begin();
}

void jog(int steps) {
  int interval = 2000;
  for (int i = 0; i < steps; i++) {
    digitalWrite(STEP_PIN, HIGH);
    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(interval)
  }
}

void loop() {
  // Listen for button inputs
  // If input given, jog
  if (Serial.available() > 0) {
    char inChar = Serial.read();
    if (inChar == 'a') {
      digitalWrite(DIR_PIN, HIGH);
      jog(20);
    }
    else if (inChar = 'd') {
      digitalWrite(DIR_PIN, LOW);
      jog(20);
    }
  }
}
