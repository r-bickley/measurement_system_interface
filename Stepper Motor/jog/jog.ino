// Function for manually jogging a stepper motor
// NEMA 17 stepper motor
// A4988 driver
// Author: Matthew Kramer, Team ME46

#define DIR_PIN     2
#define STEP_PIN    3
#define ENABLE_PIN  4

void setup() {
  pinMode(STEP_PIN,   OUTPUT);
  pinMode(DIR_PIN,    OUTPUT);
  pinMode(ENABLE_PIN, OUTPUT);
}

void jog(int steps) {
  
}

void loop() {
  // Listen for button inputs
  // If input given, jog

  while (true);
}
