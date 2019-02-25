// Script for controlling two bipolar stepper motors
// A4988 Stepper Motor Driver
// Author: Matt Kramer, Team ME46

// Serial Communication
// Input: Axis, target position, movement type
// Output: Movement done OR error

// Axis Types:
// X - X-axis
// Y - Y-axis

// Target Positions:
// [int] - target coordinate in global(?) coords

// Movement Types
// T - Transition: fast, full step
// M - Measuring: slow, 1/16th step
// C - Calibration: medium(?), full step

// Example Serial Input
// X 4000 T

#define XDIR_PIN          0
#define XSTEP_PIN         1
#define YDIR_PIN          2
#define YSTEP_PIN         3
#define ENABLE_PIN        4
#define SLEEP_PIN         5
#define RESET_PIN         6
#define MS1_PIN           8
#define MS2_PIN           9
#define MS3_PIN           10

#define STEP_HIGH        PORTD |=  0b00001000;
#define STEP_LOW         PORTD &= ~0b00001000;

#define TIMER1_INTERRUPTS_ON    TIMSK1 |=  (1 << OCIE1A);
#define TIMER1_INTERRUPTS_OFF   TIMSK1 &= ~(1 << OCIE1A);

unsigned int c0;

void setup() {
  pinMode(XDIR_PIN,       OUTPUT);
  pinMode(XSTEP_PIN,      OUTPUT);
  pinMode(YDIR_PIN,       OUTPUT);
  pinMode(YSTEP_PIN,      OUTPUT);
  pinMode(ENABLE_PIN,     OUTPUT);
  pinMode(SLEEP_PIN,      OUTPUT);
  pinMode(RESET_PIN,      OUTPUT);
  pinMode(MS1_PIN,        OUTPUT);
  pinMode(MS2_PIN,        OUTPUT);
  pinMode(MS3_PIN,        OUTPUT);
  
  digitalWrite(SLEEP_PIN, LOW);
  digitalWrite(RESET_PIN, HIGH);

  noInterrupts();
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1  = 0;
  OCR1A = 1000;                             
  TCCR1B |= (1 << WGM12);
  TCCR1B |= ((1 << CS11) | (1 << CS10));
  interrupts();

  c0 = 1600; // was 2000 * sqrt( 2 * angle / accel )

  Serial.begin(9600);
}

volatile int dir = 0;
volatile unsigned int maxSpeed = 10;
volatile unsigned long n = 0;
volatile float d;
volatile unsigned long stepCount = 0;
volatile unsigned long rampUpStepCount = 0;
volatile unsigned long totalSteps = 0;
volatile int stepPosition = 0;
volatile bool movementDone = false;

ISR(TIMER1_COMPA_vect)
{
  if (stepCount < totalSteps) {
    STEP_HIGH
    STEP_LOW
    stepCount++;
    stepPosition += dir;
  }
  else {
    movementDone = true;
    TIMER1_INTERRUPTS_OFF
  }

  if (rampUpStepCount == 0) { // ramp up phase
    n++;
    d = d - (2 * d) / (4 * n + 1);
    if (d <= maxSpeed) { // reached max speed
      d = maxSpeed;
      rampUpStepCount = stepCount;
    }
    if (stepCount >= totalSteps / 2) { // reached halfway point
      rampUpStepCount = stepCount;
    }
  }
  else if (stepCount >= totalSteps - rampUpStepCount) { // ramp down phase
    n--;
    d = (d * (4 * n + 1)) / (4 * n + 1 - 2);
  }

  OCR1A = d;
}

void sleep() {
  digitalWrite(SLEEP_PIN, LOW);
}

void wake() {
  digitalWrite(SLEEP_PIN, HIGH);
  delay(10); // Minimum 1ms delay for charge pump stabilization
}

void msSet(int ms) {
  switch (ms) {
    case 1:
      digitalWrite(MS1_PIN, LOW);
      digitalWrite(MS2_PIN, LOW);
      digitalWrite(MS3_PIN, LOW);
      break;
    case 2:
      digitalWrite(MS1_PIN, HIGH);
      digitalWrite(MS2_PIN, LOW);
      digitalWrite(MS3_PIN, LOW);
      break;
    case 4:
      digitalWrite(MS1_PIN, LOW);
      digitalWrite(MS2_PIN, HIGH);
      digitalWrite(MS3_PIN, LOW);
      break;
    case 8:
      digitalWrite(MS1_PIN, HIGH);
      digitalWrite(MS2_PIN, HIGH);
      digitalWrite(MS3_PIN, LOW);
      break;
    case 16:
      digitalWrite(MS1_PIN, HIGH);
      digitalWrite(MS2_PIN, HIGH);
      digitalWrite(MS3_PIN, HIGH);
      break;
  }
}

void moveNSteps(long steps) {
  digitalWrite(DIR_PIN, steps < 0 ? HIGH : LOW);
  dir = steps > 0 ? 1 : -1;
  totalSteps = abs(steps);
  d = c0;
  OCR1A = d;
  stepCount = 0;
  n = 0;
  rampUpStepCount = 0;
  movementDone = false;

  TIMER1_INTERRUPTS_ON
}

void moveToPosition(long p, bool wait = true) {
  moveNSteps(p - stepPosition);
  while ( wait && ! movementDone );
}

void serialJog(int jogSpeed, int jogSteps, int dir) {
  int oldSpeed = maxSpeed;
  maxSpeed = jogSpeed;
  wake();
  moveToPosition(
  if (dir == HIGH) {
    moveToPosition(stepPosition + jogSteps);
  } else {
    moveToPosition(stepPosition - jogSteps);
  }
  delay(500);
  sleep();
  maxSpeed = oldSpeed;
}

void serialMove(char axis, int targetPos, char mType) {
  // Set speed and microstepping
  switch (mType) {
    case 'T':
      msSet(1);
      maxSpeed = 200;
    case 'M':
      msSet(16);
      maxSpeed = 400;
    case 'C':
      msSet(1);
      maxSpeed = 400;
  }
  
  // Move desired axis
  if (axis == 'X') {
    moveToPosition(targetPos) // and X axis
  } else {
    moveToPosition(targetPos) // and Y axis
  }
}

void loop() {
  while (true) {
    if (Serial.available() > 0) {
      int inByte = Serial.read();
      // Change inByte to text, not unicode/ascii or whatever
      Serial.print("Received: ");
      Serial.println(inByte);
      char axis = 'X'// [First char in inByte]
      int targetPos = 1000 // [Number in inByte]
      char mType = 'T' // [Last char in inByte]
      serialMove(axis, targetPos, mType)
      // Serial.write("Movement Complete" or "Error")
      }
    }
  }
}
