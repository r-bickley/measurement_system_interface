// Script for controlling two bipolar stepper motors
// A4988 Stepper Motor Driver
// Author: Matt Kramer, Team ME46

// Serial Communication
// Input: Axis, target position, movement type
// Output: Movement done OR error

// Axis Types:
// 0 - X-axis
// 1 - Y-axis

// Target Positions:
// [int] - target coordinate in global(?) coords

// Movement Types
// 0 - Transition: fast, full step
// 1 - Measuring: slow, 1/16th step
// 2 - Calibration: medium(?), full step

// Example Serial Input
// <0,4000,1>

#define XDIR_PIN          2
#define XSTEP_PIN         3
#define YDIR_PIN          1 // CHANGE
#define YSTEP_PIN         7 // CHANGE
#define XENABLE_PIN       4
#define YENABLE_PIN       13
#define XSLEEP_PIN        5
#define YSLEEP_PIN        11
#define XRESET_PIN        6
#define YRESET_PIN        12
#define MS1_PIN           8
#define MS2_PIN           9
#define MS3_PIN           10
//#define XLIM1_PIN         11
//#define XLIM2_PIN         12
//#define YLIM1_PIN         13
//#define YLIM2_PIN         14
//#define ESTOP_PIN         15

#define STEP_HIGH        PORTD |=  0b00001000;
#define STEP_LOW         PORTD &= ~0b00001000;

#define TIMER1_INTERRUPTS_ON    TIMSK1 |=  (1 << OCIE1A);
#define TIMER1_INTERRUPTS_OFF   TIMSK1 &= ~(1 << OCIE1A);

unsigned int c0;
const byte numChars = 32;
char receivedChars[numChars];
bool newData = false;
int axis = 0;
int targetPos = 0;
int mType = 0;

void setup() {
  pinMode(XDIR_PIN,       OUTPUT);
  pinMode(XSTEP_PIN,      OUTPUT);
  pinMode(YDIR_PIN,       OUTPUT);
  pinMode(YSTEP_PIN,      OUTPUT);
  pinMode(XENABLE_PIN,    OUTPUT);
  pinMode(YENABLE_PIN,    OUTPUT);
  pinMode(XSLEEP_PIN,     OUTPUT);
  pinMode(YSLEEP_PIN,     OUTPUT);
  pinMode(XRESET_PIN,     OUTPUT);
  pinMode(YRESET_PIN,     OUTPUT);
  pinMode(MS1_PIN,        OUTPUT);
  pinMode(MS2_PIN,        OUTPUT);
  pinMode(MS3_PIN,        OUTPUT);
//  pinMode(XLIM1_PIN,      INPUT);
//  pinMode(XLIM2_PIN,      INPUT);
//  pinMode(YLIM1_PIN,      INPUT);
//  pinMode(YLIM2_PIN,      INPUT);
//  pinMode(ESTOP_PIN,      INPUT);
  
  digitalWrite(XSLEEP_PIN, LOW);
  digitalWrite(YSLEEP_PIN, LOW);
  digitalWrite(XRESET_PIN, HIGH);
  digitalWrite(YRESET_PIN, HIGH);

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
  delay(10);
  Serial.println("<Arduino is ready>");
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

void sleep(int axis) {
  if (axis == 0) {
    digitalWrite(XSLEEP_PIN, LOW);
  } else if (axis == 1) {
    digitalWrite(YSLEEP_PIN, LOW);
  }
}

void wake(int axis) {
  if (axis == 0) {
    digitalWrite(XSLEEP_PIN, HIGH);
  } else if (axis == 1) {
    digitalWrite(YSLEEP_PIN, HIGH);
  }
  delay(1); // Minimum 1ms delay for charge pump stabilization
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
  if (axis == 0) {
    digitalWrite(XDIR_PIN, steps < 0 ? HIGH : LOW);
  } else if (axis == 1) {
    digitalWrite(YDIR_PIN, steps < 0 ? HIGH : LOW);
  }
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

/*
void serialJog(int jogSpeed, int jogSteps, int dir) {
  int oldSpeed = maxSpeed;
  maxSpeed = jogSpeed;
  wake(axis);
  if (dir == HIGH) {
    moveToPosition(stepPosition + jogSteps, axis);
  } else {
    moveToPosition(stepPosition - jogSteps, axis);
  }
  delay(500);
  sleep(axis);
  maxSpeed = oldSpeed;
}
*/

void moveMotors() {
  // Set speed and microstepping
  switch (mType) {
    case 0:
      msSet(1);
      maxSpeed = 200;
      break;
    case 1:
      msSet(16);
      maxSpeed = 400;
      break;
    case 2:
      msSet(1);
      maxSpeed = 400;
      break;
  }
  
  // Move desired axis
  wake(axis);
  if (axis == 0) {
    moveToPosition(targetPos);
  } else if (axis == 1) {
    moveToPosition(targetPos);
  }
  sleep(axis);
}

void recv() {
  static bool recvInProgress = false;
  static byte ndx = 0;
  char startMarker = '<';
  char endMarker = '>';
  char rc;

  while (Serial.available() > 0 && newData == false) {
    rc = Serial.read();

    if (recvInProgress == true) {
      if (rc != endMarker) {
        receivedChars[ndx] = rc;
        ndx++;
        if (ndx >= numChars) {
          ndx = numChars - 1;
        }
      }
      else {
        receivedChars[ndx] = '\0'; // terminate string
        recvInProgress = false;
        ndx = 0;
        newData = true;
      }
    }
    else if (rc == startMarker) {
      recvInProgress = true;
    }
  }
}

void showNewData() {
  Serial.print("This just in ... ");
  Serial.println(receivedChars);
}

void parseData() {
  char * strtokIndx;
  
  strtokIndx = strtok(receivedChars, ",");
  axis = atoi(strtokIndx);

  strtokIndx = strtok(NULL, ",");
  targetPos = atoi(strtokIndx);

  strtokIndx = strtok(NULL, ",");
  mType = atoi(strtokIndx);
}

void showParsedData() {
  Serial.print("Moving Axis: ");
  Serial.println(axis);
  Serial.print("Target Position: ");
  Serial.println(targetPos);
  Serial.print("Movement Type: ");
  Serial.println(mType);
  Serial.println("\n");
  newData = false;
}

void serialInput() {
  recv();
  if (newData == true) {
    showNewData();
    parseData();
    showParsedData();
    moveMotors();
  }
}

void loop() {
  serialInput();
  delay(2000);
  // Serial.write("Movement Complete" or "Error")
  /*
  mType = 1;
  targetPos = 1000;
  moveMotors();
  mType = 0;
  targetPos = 2000;
  moveMotors();
  delay(2000);
  */
}
