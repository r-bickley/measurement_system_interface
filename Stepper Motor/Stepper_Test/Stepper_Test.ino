// Function testing file for stepper motor
// NEMA 17 stepper motor
// A4988 driver
// Author: Matthew Kramer, Team ME46
// Reference: youtu.be/fHAO7SW-SZI

#define DIR_PIN       2
#define STEP_PIN      3
#define ENABLE_PIN    4
#define SLEEP_PIN     5
#define RESET_PIN     6
#define BUTTON_FOR    8
#define BUTTON_REV    9
#define MS1_PIN       10
#define MS2_PIN       11
#define MS3_PIN       12

void setup() {
  pinMode(STEP_PIN,     OUTPUT);
  pinMode(DIR_PIN,      OUTPUT);
  pinMode(ENABLE_PIN,   OUTPUT);
  pinMode(SLEEP_PIN,    OUTPUT);
  pinMode(RESET_PIN,    OUTPUT);
  pinMode(BUTTON_FOR,   INPUT);
  pinMode(BUTTON_REV,   INPUT);
  pinMode(MS1_PIN,      OUTPUT);
  pinMode(MS2_PIN,      OUTPUT);
  pinMode(MS3_PIN,      OUTPUT);
}

// Move motor 'steps' steps
void simpleMove(int steps) {
  int interval = 900; // Control speed with rest interval
  for (int i = 0; i < steps; i++) {
    digitalWrite(STEP_PIN, HIGH);
    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(interval);
  }
}

void loop() {

  // Test simpleMove
  digitalWrite(DIR_PIN, LOW);
  simpleMove(200);
  delay(1000);
  digitalWrite(DIR_PIN, HIGH);
  simpleMove(200);

  delay(500);

  //while (true);
}
