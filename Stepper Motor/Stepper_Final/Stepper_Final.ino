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
// Move X-axis to position 4000 for measurement

#include <Nextion.h>

#define XDIR_PIN          23
#define XSTEP_PIN         3
#define YDIR_PIN          39
#define YSTEP_PIN         2
#define XENABLE_PIN       35
#define YENABLE_PIN       51
#define XSLEEP_PIN        25
#define YSLEEP_PIN        41
#define XRESET_PIN        27
#define YRESET_PIN        43
#define XMS1_PIN          33
#define XMS2_PIN          31
#define XMS3_PIN          29
#define YMS1_PIN          49
#define YMS2_PIN          47
#define YMS3_PIN          45
#define XLIM1_PIN         11
#define XLIM2_PIN         12
#define YLIM1_PIN         13
#define YLIM2_PIN         14
#define ESTOP_PIN         15
#define SENSOR_PIN        A0

#define XSTEP_HIGH        PORTE |=  0b00100000;
#define XSTEP_LOW         PORTE &= ~0b00100000;

#define YSTEP_HIGH        PORTE |=  0b00010000;
#define YSTEP_LOW         PORTE &= ~0b00010000;

#define TIMER1_INTERRUPTS_ON    TIMSK1 |=  (1 << OCIE1A);
#define TIMER1_INTERRUPTS_OFF   TIMSK1 &= ~(1 << OCIE1A);

unsigned int c0;
bool measuring = false;
int pos = 0;

NexButton bCal =      NexButton(0,1,  "bCal");
NexButton bMeasure =  NexButton(0,2,  "bMeasure");
NexButton bSelect =   NexButton(0,3,  "bSelect");
NexButton bPause =    NexButton(1,1,  "bPause");
NexButton bMCancel =  NexButton(1,2,  "bMCancel");
NexButton bFSelect =  NexButton(2,1,  "bFSelect");
NexButton bFBack =    NexButton(2,2,  "bFBack");
NexButton bJogXPos =  NexButton(3,1,  "bJogXPos");
NexButton bJogXNeg =  NexButton(3,2,  "bJogXNeg");
NexButton bJogYPos =  NexButton(3,3,  "bJogYPos");
NexButton bJogYNeg =  NexButton(3,4,  "bJogYNeg");
NexButton bCConfirm =  NexButton(3,5, "bCConfirm");
NexButton bCCancel =  NexButton(3,9,  "bCCancel");
NexButton bJogDist =  NexButton(3,10, "bJogDist");

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
  pinMode(XMS1_PIN,       OUTPUT);
  pinMode(XMS2_PIN,       OUTPUT);
  pinMode(XMS3_PIN,       OUTPUT);
  pinMode(YMS1_PIN,       OUTPUT);
  pinMode(YMS2_PIN,       OUTPUT);
  pinMode(YMS3_PIN,       OUTPUT);
  pinMode(XLIM1_PIN,      INPUT);
  pinMode(XLIM2_PIN,      INPUT);
  pinMode(YLIM1_PIN,      INPUT);
  pinMode(YLIM2_PIN,      INPUT);
  pinMode(ESTOP_PIN,      INPUT);
  pinMode(SENSOR_PIN,     INPUT);
  
  digitalWrite(XSLEEP_PIN, LOW);
  digitalWrite(YSLEEP_PIN, LOW);
  digitalWrite(XRESET_PIN, HIGH);
  digitalWrite(YRESET_PIN, HIGH);

  analogReference(DEFAULT);

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
  //Serial.println("<Arduino is ready>");

  bCal.attachPop(bCalPopCallback,           &bCal);
  bMeasure.attachPop(bMeasurePopCallback,   &bMeasure);
  bSelect.attachPop(bSelectPopCallback,     &bSelect);
  bPause.attachPop(bPausePopCallback,       &bPause);
  bMCancel.attachPop(bMCancelPopCallback,   &bMCancel);
  bFSelect.attachPop(bFSelectPopCallback,   &bFSelect);
  bFBack.attachPop(bFBackPopCallback,       &bFBack);
  bJogXPos.attachPop(bJogXPosPopCallback,   &bJogXPos);
  bJogXNeg.attachPop(bJogXNegPopCallback,   &bJogXNeg);
  bJogYPos.attachPop(bJogYPosPopCallback,   &bJogYPos);
  bJogYNeg.attachPop(bJogYNegPopCallback,   &bJogYNeg);
  bCConfirm.attachPop(bCConfirmPopCallback, &bCConfirm);
  bCCancel.attachPop(bCCancelPopCallback,   &bCCancel);
  bJogDist.attachPop(bJogDistPopCallback,   &bJogDist);
}

////////////////////////////////////////////////////////////////////////
//                                                                    //
//                            SCREEN FUNCTIONS                        //
//                                                                    //
////////////////////////////////////////////////////////////////////////

NexTouch *nex_listen_list[] = {
  &bCal,
  &bMeasure,
  &bSelect,
  &bPause,
  &bMCancel,
  &bFSelect,
  &bFBack,
  &bJogXPos,
  &bJogXNeg,
  &bJogYPos,
  &bJogYNeg,
  &bCConfirm,
  &bCCancel,
  &bJogDist,
  NULL
};

void bCalPopCallback(void *ptr) {
  // Move to calibration screen
  // Start calibration
}

void bMeasurePopCallback(void *ptr) {
  // Move to measurement screen
  // Start measurement
}

void bSelectPopCallback(void *ptr) {
  // Move to file select screen
  // Read from USB input
}

void bPausePopCallback(void *ptr) {
  // Pause measurement
}

void bMCancelPopCallback(void *ptr) {
  // Stop measurement
}

void bFSelectPopCallback(void *ptr) {
  // Import info from chosen
  // Move to home screen
}

void bFBackPopCallback(void *ptr) {
  // Move to home screen
}

void bJogXPosPopCallback(void *ptr) {
  // Move X plus steps
}

void bJogXNegPopCallback(void *ptr) {
  // Move X minus steps
}

void bJogYPosPopCallback(void *ptr) {
  // Move Y plus steps
}

void bJogYNegPopCallback(void *ptr) {
  // Move Y minus steps
}

void bCConfirmPopCallback(void *ptr) {
  // Move to next fid
}

void bCCancelPopCallback(void *ptr) {
  // Move to home screen
}

void bJogDistPopCallback(void *ptr) {
  // Change jog dist??
}

////////////////////////////////////////////////////////////////////////
//                                                                    //
//                             MOTOR FUNCTIONS                        //
//                                                                    //
////////////////////////////////////////////////////////////////////////

int axis = 0;
int targetPos = 0;
int mType = 0;

volatile int dir = 0;
volatile unsigned int maxSpeed = 10;
volatile unsigned long n = 0;
volatile float d;
volatile unsigned long stepCount = 0;
volatile unsigned long rampUpStepCount = 0;
volatile unsigned long totalSteps = 0;
volatile int xStepPosition = 0;
volatile int yStepPosition = 0;
int xMaxStepPosition = 0;
int yMaxStepPosition = 0;
volatile bool movementDone = false;
unsigned long jogDist = 0;

ISR(TIMER1_COMPA_vect)
{
  if (stepCount < totalSteps) {
    stepAxis();
    if (measuring) {
      readDouble();
      logValues(pos);
    }
  } else if (XLIM1_PIN == LOW || XLIM2_PIN == LOW || YLIM1_PIN == LOW || YLIM2_PIN == LOW) {
    movementDone = true;
    TIMER1_INTERRUPTS_OFF
  }
  else {
    movementDone = true;
    TIMER1_INTERRUPTS_OFF
  }

  if (rampUpStepCount == 0) { // ramp up phase
    n++;
    d = d-(2*d) / (4*n+1);
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
    d = (d*(4*n+1)) / (4*n+1-2);
  }

  OCR1A = d;
}

void stepAxis() {
  if (axis == 0) {
    XSTEP_HIGH
    XSTEP_LOW
    stepCount++;
    xStepPosition += dir;
  } else if (axis == 1) {
    YSTEP_HIGH
    YSTEP_LOW
    stepCount++;
    yStepPosition += dir;
  }
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
  if (axis == 0) {
    if (ms == 1) {
      digitalWrite(XMS1_PIN, LOW);
      digitalWrite(XMS2_PIN, LOW);
      digitalWrite(XMS3_PIN, LOW);
    } else if (ms == 2) {
      digitalWrite(XMS1_PIN, HIGH);
      digitalWrite(XMS2_PIN, LOW);
      digitalWrite(XMS3_PIN, LOW);
    } else if (ms == 4) {
      digitalWrite(XMS1_PIN, LOW);
      digitalWrite(XMS2_PIN, HIGH);
      digitalWrite(XMS3_PIN, LOW);
    } else if (ms == 8) {
      digitalWrite(XMS1_PIN, HIGH);
      digitalWrite(XMS2_PIN, HIGH);
      digitalWrite(XMS3_PIN, LOW);
    } else if (ms == 16) {
      digitalWrite(XMS1_PIN, HIGH);
      digitalWrite(XMS2_PIN, HIGH);
      digitalWrite(XMS3_PIN, HIGH);
    }
  } else if (axis == 1) {
    if (ms == 1) {
      digitalWrite(YMS1_PIN, LOW);
      digitalWrite(YMS2_PIN, LOW);
      digitalWrite(YMS3_PIN, LOW);
    } else if (ms == 2) {
      digitalWrite(YMS1_PIN, HIGH);
      digitalWrite(YMS2_PIN, LOW);
      digitalWrite(YMS3_PIN, LOW);
    } else if (ms == 4) {
      digitalWrite(YMS1_PIN, LOW);
      digitalWrite(YMS2_PIN, HIGH);
      digitalWrite(YMS3_PIN, LOW);
    } else if (ms == 8) {
      digitalWrite(YMS1_PIN, HIGH);
      digitalWrite(YMS2_PIN, HIGH);
      digitalWrite(YMS3_PIN, LOW);
    } else if (ms == 16) {
      digitalWrite(YMS1_PIN, HIGH);
      digitalWrite(YMS2_PIN, HIGH);
      digitalWrite(YMS3_PIN, HIGH);
    }
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
  if (axis == 0) {
    moveNSteps(p - xStepPosition);
  } else if (axis == 1) {
    moveNSteps(p - yStepPosition);
  }
  while ( wait && ! movementDone );
}

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
  Serial.println("Movement Done");
}

////////////////////////////////////////////////////////////////////////
//                                                                    //
//                            LASER FUNCTIONS                         //
//                                                                    //
////////////////////////////////////////////////////////////////////////

int sensorValue = 0;
float voltage = 0.0;
float dist = 0.0;

const int numSamples = 1000;
float dists[numSamples];

void readDouble() {
  sensorValue = analogRead(SENSOR_PIN);
  sensorValue = analogRead(SENSOR_PIN);
  voltage = sensorValue * (5.0 / 1023.0); // change for 16-bit ADC
  dist = voltage * 60 + 50; // measured distance in mm
  Serial.println(dist, DEC);
  //delay(1);
}

void logValues(int pos) {
  dists[pos] = dist;
  pos++;
}

////////////////////////////////////////////////////////////////////////
//                                                                    //
//                            DRIVER FUNCTIONS                        //
//                                                                    //
////////////////////////////////////////////////////////////////////////

void calibrate() {
  mType = 2;

  axis = 1;
  // XLIM1
  targetPos = -100000;
  moveMotors();
  xStepPosition = 0;
  // XLIM2
  targetPos = 100000;
  moveMotors();
  xMaxStepPosition = xStepPosition;

  axis = 2;
  // YLIM1
  targetPos = -100000;
  moveMotors();
  yStepPosition = 0;
  // YLIM2
  targetPos = 100000;
  moveMotors();
  yMaxStepPosition = yStepPosition;
}

void measureOne(float x, float y, int featureType) {
  // Function to measure one feature

  // move next to feature

  // start measurement and logging
  measuring = true;
  pos = 0;

  // move across feature
  axis = 1;
  targetPos = yStepPosition + numSamples;
  mType = 0;
  moveMotors();

  // stop measurement and logging
  measuring = false;
  pos = 0;

  // output data to appropriate location
}

////////////////////////////////////////////////////////////////////////
//                                                                    //
//                            SERIAL FUNCTIONS                        //
//                                                                    //
////////////////////////////////////////////////////////////////////////

const byte numChars = 32;
char receivedChars[numChars];
bool newData = false;

void recv() {
  static bool recvInProgress = false;
  static byte ndx = 0;
  char startMarker = 'p';
  char endMarker = 'ÿ';
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
  Serial.print("Received Data: ");
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

  newData = false;
}

void showParsedData() {
  Serial.print("Moving Axis: ");
  Serial.println(axis);
  Serial.print("Target Position: ");
  Serial.println(targetPos);
  Serial.print("Movement Type: ");
  Serial.println(mType);
  Serial.println("\n");
}

void sendReady() {
  Serial.println(1);
}

void serialInput() {
  recv();
  if (newData == true) {
    //showNewData();
    parseData();
    //showParsedData();
    
    if ((axis == -1) && (targetPos == -1) && (mType == -1)) {
      sendReady();
    } else {
      moveMotors();
    }
  }
}

////////////////////////////////////////////////////////////////////////
//                                                                    //
//                               MAIN LOOP                            //
//                                                                    //
////////////////////////////////////////////////////////////////////////

void loop() {
  //nexLoop(nex_listen_list);
  //serialInput();
  delay(10);

  measureOne(0,0,0);
  while (true);
}
