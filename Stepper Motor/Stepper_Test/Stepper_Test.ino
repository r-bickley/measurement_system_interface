// Function testing file for stepper motor
// NEMA 17 stepper motor
// A4988 driver
// Author: Matthew Kramer, Team ME46
// Reference: youtu.be/fHAO7SW-SZI

#define DIR_PIN     2
#define STEP_PIN    3
#define ENABLE_PIN  4

void setup() {
  pinMode(STEP_PIN,   OUTPUT);
  pinMode(DIR_PIN,    OUTPUT);
  pinMode(ENABLE_PIN, OUTPUT);
}

// Move motor 'steps' steps
void simpleMove(int steps) {
  int interval = 1200; // Control speed with rest interval
  for (int i = 0; i < steps; i++) {
    digitalWrite(STEP_PIN, HIGH);
    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(interval);
  }
}

// Move motor 'steps' steps
// Non-linear acceleration
void simpleAccel(int steps) {
  
  int lowSpeed = 2000;
  int highSpeed = 100;
  int change = 2;

  int rampUpStop = (lowSpeed - highSpeed) / change;
  if (rampUpStop > steps / 2)
    rampUpStop = steps / 2;
  int rampDownStart = steps = rampUpStop;

  int d = lowSpeed;

  for (i = 0; i < steps; i++) {
    digitalWrite(STEP_PIN, HIGH);
    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(d)

    if (i < rampUpStop)
      d -= change;
     else if (i > rampDownStart)
      d += change;
  }
}

// Move motor 'steps' steps
// Linear acceleration
void linearAccel(int steps) {
  int delays[steps];
  float angle = 1;
  float accel = 0.01;
  float c0 = 2000 * sqrt(2*angle/accel) * 0.67703;
  float lastDelay = 0;
  int highSpeed = 100;
  for (int i = 0; i < STEPS; i++) {
    float d = c0;
    if (d < highSpeed)
      d = highSpeed;
    delays[i] = d;
    lastDelay = d;
  }

  // ACCEL, read FORWARD through array
  for (int i = 0; i < STEPS; i++) {
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(delays[i]);
    digitalWrite(STEP_PIN, LOW);
  }

  // DECEL, read BACKWARD through array
  for (int i = 0; i < STEPS; i++) {
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(delays[STEPS-i-1]);
    digitalWrite(STEP_PIN, LOW);
  }
}

void loop() {

  // Test simpleMove
  digitalWrite(DIR_PIN, LOW);
  simpleMove(800);
  digitalWrite(DIR_PIN, HIGH);
  simpleMove(800);

  // Test simpleAccel
  digitalWrite(DIR_PIN, LOW);
  simpleAccel(2400);
  digitalWrite(DIR_PIN, HIGH);
  simpleAccel(2400);

  // Test linearAccel
  digitalWrite(DIR_PIN, LOW);
  linearAccel(800);
  digitalWrite(DIR_PIN, HIGH);
  linearAccel(800);

  while (true);
}
