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
#include <Adafruit_ADS1015.h>

#define XDIR_PIN          35
#define XSTEP_PIN         3
#define YDIR_PIN          51
#define YSTEP_PIN         2
#define XENABLE_PIN       23
#define YENABLE_PIN       39
#define XSLEEP_PIN        33
#define YSLEEP_PIN        49
#define XRESET_PIN        31
#define YRESET_PIN        47
#define XMS1_PIN          25
#define XMS2_PIN          27
#define XMS3_PIN          29
#define YMS1_PIN          41
#define YMS2_PIN          43
#define YMS3_PIN          45
#define XLIM1_PIN         22
#define XLIM2_PIN         24
#define YLIM1_PIN         50
#define YLIM2_PIN         52
#define SENSOR_PIN        15

int axis = 0;
int targetPos = 0;
int mType = 0;
unsigned long jogDist = 100;

volatile int xStepPosition = 0;
volatile int yStepPosition = 0;
int xMaxStepPosition = 0;
int yMaxStepPosition = 0;

int xFilterStepPosition = 0;
int yFilterStepPosition = 0;

int X1;
int X2;
int Y1;
int Y2;
bool ignoreLims = false;

#define XSTEP_HIGH        PORTE |=  0b00010000;
#define XSTEP_LOW         PORTE &= ~0b00010000;

#define YSTEP_HIGH        PORTE |=  0b00100000;
#define YSTEP_LOW         PORTE &= ~0b00100000;

#define TIMER1_INTERRUPTS_ON    TIMSK1 |=  (1 << OCIE1A);
#define TIMER1_INTERRUPTS_OFF   TIMSK1 &= ~(1 << OCIE1A);

const int numSamples = 100;
float dists[numSamples] = {0};

int fid1XStep = 900;
int fid1YStep = 1750;
int fid2XStep = 900;
int fid2YStep = 9230;
int fid3XStep = 6130;
int fid3YStep = 9230;

float fid1Dist;
float fid2Dist;
float fid3Dist;

typedef struct {
  char refdes;
  float x;
  float y;
} ddt;

typedef struct {
  char refdes;
  float x;
  float y;
} fid;

// Example ddts and fids
ddt te12 = {"TE12", 2410, 8150};
ddt te7 = {"TE13", 3910, 8170};
ddt te6 = {"TE14", 4620, 8180};

fid fid1 = {"FID1", 900, 1750};
fid fid2 = {"FID2", 900, 9230};
fid fid3 = {"FID3", 6110, 9250};

const int numTuners = 3;
const int numFiducials = 3;

ddt tuners[numTuners] = {te12, te7, te6};
fid fiducials[numFiducials] = {fid1, fid2, fid3};

unsigned int c0;
bool measuring = false;
int pos = 0;

Adafruit_ADS1115 adc;

// NexButton(pageID, componentID, componentName);
NexButton bh0 =       NexButton(0,1,  "bh0");
NexButton bh1 =       NexButton(0,2,  "bh1");
NexButton bh2 =       NexButton(0,3,  "bh2");
NexButton bhHome =    NexButton(0,6,  "bhHome");
NexText   tgcState =  NexText(1,3,    "tgcState");
NexButton bJogXP1 =   NexButton(2,2,  "bJogXP1");
NexButton bJogXN1 =   NexButton(2,3,  "bJogXN1");
NexButton bJogYP1 =   NexButton(2,4,  "bJogYP1");
NexButton bJogYN1 =   NexButton(2,5,  "bJogYN1");
NexButton bc1 =       NexButton(2,6,  "bc1");
NexButton bcCancel1 = NexButton(2,7,  "bcCancel1");
NexButton bJogXP2 =   NexButton(3,2,  "bJogXP2");
NexButton bJogXN2 =   NexButton(3,3,  "bJogXN2");
NexButton bJogYP2 =   NexButton(3,4,  "bJogYP2");
NexButton bJogYN2 =   NexButton(3,5,  "bJogYN2");
NexButton bc2 =       NexButton(3,6,  "bc2");
NexButton bcCancel2 = NexButton(3,7,  "bcCancel2");
NexButton bc3 =       NexButton(4,2,  "bc3");
NexButton bcCancel3 = NexButton(4,3,  "bcCancel3");
NexButton bcRetry =   NexButton(4,4,  "bcRetry");
NexButton bmContinue =NexButton(5,3,  "bmContinue");
NexText   tmState =   NexText(5,3,    "tmState");

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
  pinMode(XLIM1_PIN,      INPUT_PULLUP);
  pinMode(XLIM2_PIN,      INPUT_PULLUP);
  pinMode(YLIM1_PIN,      INPUT_PULLUP);
  pinMode(YLIM2_PIN,      INPUT_PULLUP);
  pinMode(SENSOR_PIN,     INPUT);
  
  digitalWrite(XSLEEP_PIN, LOW);
  digitalWrite(YSLEEP_PIN, LOW);
  digitalWrite(XRESET_PIN, HIGH);
  digitalWrite(YRESET_PIN, HIGH);
  digitalWrite(XENABLE_PIN,LOW);
  digitalWrite(YENABLE_PIN,LOW);

  analogReference(DEFAULT);
  adc.begin();

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

  nexInit();
  bh0.attachPop(bh0PopCallback,             &bh0);
  bh1.attachPop(bh1PopCallback,             &bh1);
  bh2.attachPop(bh2PopCallback,             &bh2);
  bhHome.attachPop(bhHomePopCallback,          &bhHome);
  bJogXP1.attachPop(bJogXP1PopCallback,     &bJogXP1);
  bJogXN1.attachPop(bJogXN1PopCallback,     &bJogXN1);
  bJogYP1.attachPop(bJogYP1PopCallback,     &bJogYP1);
  bJogYN1.attachPop(bJogYN1PopCallback,     &bJogYN1);
  bc1.attachPop(bc1PopCallback,             &bc1);
  bcCancel1.attachPop(bcCancel1PopCallback, &bcCancel1);
  bJogXP2.attachPop(bJogXP2PopCallback,     &bJogXP2);
  bJogXN2.attachPop(bJogXN2PopCallback,     &bJogXN2);
  bJogYP2.attachPop(bJogYP2PopCallback,     &bJogYP2);
  bJogYN2.attachPop(bJogYN2PopCallback,     &bJogYN2);
  bc2.attachPop(bc2PopCallback,             &bc2);
  bcCancel2.attachPop(bcCancel2PopCallback, &bcCancel2);
  bc3.attachPop(bc3PopCallback,             &bc3);
  bcCancel3.attachPop(bcCancel3PopCallback, &bcCancel3);
  bcRetry.attachPop(bcRetryPopCallback,     &bcRetry);
  bmContinue.attachPop(bmContinuePopCallback,&bmContinue);
}

////////////////////////////////////////////////////////////////////////
//                                                                    //
//                            SCREEN FUNCTIONS                        //
//                                                                    //
////////////////////////////////////////////////////////////////////////

NexTouch *nex_listen_list[] = {
  &bh0,
  &bh1,
  &bh2,
  &bhHome,
  &bJogXP1,
  &bJogXN1,
  &bJogYP1,
  &bJogYN1,
  &bc1,
  &bcCancel1,
  &bJogXP2,
  &bJogXN2,
  &bJogYP2,
  &bJogYN2,
  &bc2,
  &bcCancel2,
  &bc3,
  &bcCancel3,
  &bcRetry,
  &bmContinue,
  NULL
};

void bh0PopCallback(void *ptr) {
  tgcState.setText("Calibrating...");
  globalCal();
  tgcState.setText("Calibration Complete");
}

void bh1PopCallback(void *ptr) {
  //mmToStep();
  //localCal();
}

void bh2PopCallback(void *ptr) {
  tmState.setText("Measuring...");
  measureAll();
  tmState.setText("Measurement Complete");
}

void bhHomePopCallback(void *ptr) {
  axis = 0;
  targetPos = -30000;
  mType = 0;
  moveMotors();
  axis = 1;
  moveMotors();
}

void bJogXP1PopCallback(void *ptr) {
  // Move X plus steps
  axis = 0;
  targetPos = xStepPosition + jogDist;
  mType = 0;
  moveMotors();
  Serial.print("X: ");
  Serial.println(xStepPosition,DEC);
}

void bJogXN1PopCallback(void *ptr) {
  // Move X minus steps
  axis = 0;
  targetPos = xStepPosition - jogDist;
  mType = 0;
  moveMotors();
  Serial.print("X: ");
  Serial.println(xStepPosition,DEC);
}

void bJogYP1PopCallback(void *ptr) {
  // Move Y plus steps
  axis = 1;
  targetPos = yStepPosition + jogDist;
  mType = 0;
  moveMotors();
  Serial.print("Y: ");
  Serial.println(yStepPosition,DEC);
}

void bJogYN1PopCallback(void *ptr) {
  // Move Y minus steps
  axis = 1;
  targetPos = yStepPosition - jogDist;
  mType = 0;
  moveMotors();
  Serial.print("Y: ");
  Serial.println(yStepPosition,DEC);
}

void bc1PopCallback(void *ptr) {
  fid1XStep = xStepPosition;
  fid1YStep = yStepPosition;
  fid1Dist = measureFid();
}

void bcCancel1PopCallback(void *ptr) {
  
}

void bJogXP2PopCallback(void *ptr) {
  // Move X plus steps
  axis = 0;
  targetPos = xStepPosition + jogDist;
  mType = 0;
  moveMotors();
}

void bJogXN2PopCallback(void *ptr) {
  // Move X minus steps
  axis = 0;
  targetPos = xStepPosition - jogDist;
  mType = 0;
  moveMotors();
}

void bJogYP2PopCallback(void *ptr) {
  // Move Y plus steps
  axis = 1;
  targetPos = yStepPosition + jogDist;
  mType = 0;
  moveMotors();
}

void bJogYN2PopCallback(void *ptr) {
  // Move Y minus steps
  axis = 1;
  targetPos = yStepPosition - jogDist;
  mType = 0;
  moveMotors();
}

void bc2PopCallback(void *ptr) {
  fid2XStep = xStepPosition;
  fid2YStep = yStepPosition;
  fid2Dist = measureFid();
  moveToFid(fid3);
}

void bcCancel2PopCallback(void *ptr) {
  
}

void bc3PopCallback(void *ptr) {
  fid3XStep = xStepPosition;
  fid3YStep = yStepPosition;
  fid3Dist = measureFid();
  localCoords();
}

void bcCancel3PopCallback(void *ptr) {
  
}

void bcRetryPopCallback(void *ptr) {
  axis = 0;
  targetPos = xStepPosition - 100;
  moveMotors();
  targetPos = xStepPosition + 100;
  moveMotors();
}

void bmContinuePopCallback(void *ptr) {
  
}

////////////////////////////////////////////////////////////////////////
//                                                                    //
//                             DATA FUNCTIONS                         //
//                                                                    //
////////////////////////////////////////////////////////////////////////

void mmToStep() {
  // DDTs
  for (int i = 0; i < numTuners; i++) {
    float mmX = tuners[i].x;
    float mmY = tuners[i].y;
    tuners[i].x = floor(mmX/0.02);
    tuners[i].y = floor(mmY/0.02);
  }

  // Fids
  for (int i = 0; i < numFiducials; i++) {
    float mmX = fiducials[i].x;
    float mmY = fiducials[i].y;
    fiducials[i].x = floor(mmX/0.02);
    fiducials[i].y = floor(mmY/0.02);
  }  
}

void localCoords() {
  //xFilterStepPosition
}

////////////////////////////////////////////////////////////////////////
//                                                                    //
//                             MOTOR FUNCTIONS                        //
//                                                                    //
////////////////////////////////////////////////////////////////////////

volatile int dir = 0;
volatile unsigned int maxSpeed = 10;
volatile unsigned long n = 0;
volatile float d;
volatile unsigned long stepCount = 0;
volatile unsigned long rampUpStepCount = 0;
volatile unsigned long totalSteps = 0;
volatile bool movementDone = false;

ISR(TIMER1_COMPA_vect)
{
  X1 = digitalRead(XLIM1_PIN);
  X2 = digitalRead(XLIM2_PIN);
  Y1 = digitalRead(YLIM1_PIN);
  Y2 = digitalRead(YLIM2_PIN);

  if ((X1 == LOW || X2 == LOW || Y1 == LOW || Y2 == LOW) && ignoreLims == false) {
    movementDone = true;
    TIMER1_INTERRUPTS_OFF
  } else if (stepCount < totalSteps) {
    stepAxis();
    //if (measuring) {
    //  //readDouble();
    //  readADC();
    //  logValues(pos);
    //}
  } else {
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

void sleep() {
  if (axis == 0) {
    digitalWrite(XSLEEP_PIN, LOW);
  } else if (axis == 1) {
    digitalWrite(YSLEEP_PIN, LOW);
  }
}

void wake() {
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
    digitalWrite(XDIR_PIN, steps < 0 ? LOW : HIGH);
  } else if (axis == 1) {
    digitalWrite(YDIR_PIN, steps < 0 ? LOW : HIGH);
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
  wake();
  moveToPosition(targetPos);
  
  // Check for limit switches hit
  if (X1 == LOW || X2 == LOW || Y1 == LOW || Y2 == LOW) {
    limitHit();
  }
  sleep();
  //Serial.println("Movement Done");
}

void limitHit() {
  ignoreLims = true;
  mType = 0;
    
  if (X1 == LOW) {
    moveToPosition(xStepPosition + 200);
  } else if (X2 == LOW) {
    moveToPosition(xStepPosition - 200);
  } else if (Y1 == LOW) {
    moveToPosition(yStepPosition + 200);
  } else if (Y2 == LOW) {
    moveToPosition(yStepPosition - 200);
  }
  
  ignoreLims = false;
}

////////////////////////////////////////////////////////////////////////
//                                                                    //
//                            LASER FUNCTIONS                         //
//                                                                    //
////////////////////////////////////////////////////////////////////////

int16_t sensorValue = 0;
float voltage = 0.0;
float dist = 0.0;
int16_t adcValue = 0;

void readADC() {
  adcValue = adc.readADC_SingleEnded(0);
  //adcValue = adc.readADC_SingleEnded(0);
  voltage = adcValue * (5.0 / 65535.0); // 16-bit external ADC
  dist = (voltage * 60 + 50) / 25.4; // measured distance in inches
  Serial.println(dist, DEC);
}

void readDouble() {
  sensorValue = analogRead(SENSOR_PIN);
  sensorValue = analogRead(SENSOR_PIN);
  voltage = sensorValue * (5.0 / 1023.0); // 10-bit internal ADC
  dist = (voltage * 60 + 50) / 25.4; // measured distance in inches
  Serial.println(dist, DEC);
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

void moveToTuner(ddt t) {
  mType = 0;
  int tunerRadius = 0;
  
  // move x axis
  axis = 0;
  targetPos = t.x;
  moveMotors();
  
  // move y axis
  axis = 1;
  targetPos = t.y - tunerRadius;
  moveMotors();
}

void moveToFid(fid f) {
  mType = 0;

  // move x axis
  axis = 0;
  targetPos = f.x;
  moveMotors();

  // move y axis
  axis = 1;
  targetPos = f.y;
  moveMotors();
}

void localCal(fid f) {
  mType = 1;
  
  if (f.refdes == "FID1") {
    fid1XStep = xStepPosition;
    fid1YStep = yStepPosition;
    // take dist measurement, store
  } else if (f.refdes == "FID2") {
    fid2XStep = xStepPosition;
    fid2YStep = yStepPosition;
    // take dist measurement, store
  } else if (f.refdes == "FID3") {
    moveToFid(f);
    // take dist measurement, store
  }
}

void globalCal() {
  mType = 0;

  axis = 0;
  // XLIM1
  targetPos = -30000;
  moveMotors();
  xStepPosition = 0;
  // XLIM2
  targetPos = 30000;
  moveMotors();
  xMaxStepPosition = xStepPosition;

  axis = 1;
  // YLIM1
  targetPos = -30000;
  moveMotors();
  yStepPosition = 0;
  // YLIM2
  targetPos = 30000;
  moveMotors();
  yMaxStepPosition = yStepPosition;

  // Return to (0,0)
  targetPos = 0;
  axis = 0;
  moveMotors();
  axis = 1;
  moveMotors();
}

void measureDDT() {
  // start measurement and logging
  measuring = true;
  pos = 0;

  // move across feature
  axis = 0;
  //targetPos = xStepPosition + numSamples;
  mType = 1;
  //moveMotors();

  for (int i = 0; i <= numSamples; i++) {
    readADC();
    logValues(i);
    targetPos = xStepPosition + 40;
    moveMotors();
  }

  // stop measurement and logging
  measuring = false;
  pos = 0;
  xStepPosition = xStepPosition - 3700;
  // output data to appropriate location
}

float measureFid() {
  readADC();
  float fidDist = dist;
  return fidDist;
}

void measureAll() {
  //moveToFid(fid1);
  //delay(5000);
  //moveToFid(fid2);
  //delay(5000);
  //moveToFid(fid3);
  //delay(5000);
  //moveToFid(fid1);
  //delay(500);
  moveToTuner(te12);
  Serial.print("Before: ");
  Serial.println(xStepPosition,DEC);
  measureDDT();
  Serial.print("After: ");
  Serial.println(xStepPosition,DEC);
  delay(5000);
  moveToTuner(te7);
  measureDDT();
  delay(5000);
  moveToTuner(te6);
  measureDDT();

  //for (int i = 0; i <= numTuners; i++) {
  //  moveToTuner(tuners[i]);
  //  measureDDT();
  //}
}

////////////////////////////////////////////////////////////////////////
//                                                                    //
//                            SERIAL FUNCTIONS                        //
//                                                                    //
////////////////////////////////////////////////////////////////////////

const byte numChars = 32;
char receivedChars[numChars];
bool newData = false;
bool screenData = false;

void recv() {
  static bool recvInProgress = false;
  static byte ndx = 0;
  char startMarker = '<';
  char endMarker = '>';
  char screenStartMarker = 'p';
  char screenEndMarker = 'ÿ';
  char rc;

  while (Serial.available() > 0 && newData == false) {
    rc = Serial.read();

    if (recvInProgress == true) {
      if (rc != endMarker && rc != screenEndMarker) {
        receivedChars[ndx] = rc;
        ndx++;
        if (ndx >= numChars) {
          ndx = numChars - 1;
       }
      } else {
        receivedChars[ndx] = '\0'; // terminate string
        recvInProgress = false;
        ndx = 0;
        newData = true;
      }
    } else if (rc == startMarker) {
      recvInProgress = true;
      screenData = false;
    } else if (rc == screenStartMarker) {
      recvInProgress = true;
      screenData = true;
    }
  }
}

void showNewData() {
  Serial.print("Received Data: ");
  Serial.println(receivedChars);
}

void parseData() {
  char * strtokIndx;

  if (screenData == true) {
    strtokIndx = strtok(receivedChars, ",");
    
  } else {
    strtokIndx = strtok(receivedChars, ",");
    axis = atoi(strtokIndx);

    strtokIndx = strtok(NULL, ",");
    targetPos = atoi(strtokIndx);

    strtokIndx = strtok(NULL, ",");
    mType = atoi(strtokIndx);
  }

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

void screenRecv() {
  //
}

void parseScreenData() {
  //
}

void screenInput() {
  screenRecv();
  if (newData == true) {
    //showNewData();
    parseScreenData();
    //showParsedData();
  }
}

////////////////////////////////////////////////////////////////////////
//                                                                    //
//                               MAIN LOOP                            //
//                                                                    //
////////////////////////////////////////////////////////////////////////

void loop() {
  //nexLoop(nex_listen_list);
  //globalCal();
  //mmToStep();
  //measureDDT();
  while (true) {
  //  serialInput();
  //  readADC();
  //  delay(10);
    nexLoop(nex_listen_list);
  }
  
  //while (true);
}
