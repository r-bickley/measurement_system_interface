#define DIR_PIN          2
#define STEP_PIN         3
#define ENABLE_PIN       4
#define SLEEP_PIN        5
#define RESET_PIN        6
#define MS1_PIN          10
#define MS2_PIN          11
#define MS3_PIN          12
#define LED_PIN          13

#define STEP_HIGH        PORTD |=  0b00001000;
#define STEP_LOW         PORTD &= ~0b00001000;

#define TIMER1_INTERRUPTS_ON    TIMSK1 |=  (1 << OCIE1A);
#define TIMER1_INTERRUPTS_OFF   TIMSK1 &= ~(1 << OCIE1A);

unsigned int c0;
const int buttonPinFor = 8;
const int buttonPinRev = 9;
int buttonStateFor = 0;
int buttonStateRev = 0;
int ledState = LOW;

void setup() {
  pinMode(STEP_PIN,       OUTPUT);
  pinMode(DIR_PIN,        OUTPUT);
  pinMode(ENABLE_PIN,     OUTPUT);
  pinMode(SLEEP_PIN,      OUTPUT);
  pinMode(RESET_PIN,      OUTPUT);
  pinMode(buttonPinFor,   INPUT);
  pinMode(buttonPinRev,   INPUT);
  pinMode(MS1_PIN,        OUTPUT);
  pinMode(MS2_PIN,        OUTPUT);
  pinMode(MS3_PIN,        OUTPUT);
  pinMode(LED_PIN,        OUTPUT);

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
  if ( stepCount < totalSteps ) {
    STEP_HIGH
    STEP_LOW
    stepCount++;
    stepPosition += dir;
  }
  else {
    movementDone = true;
    TIMER1_INTERRUPTS_OFF
  }

  if ( rampUpStepCount == 0 ) { // ramp up phase
    n++;
    d = d - (2 * d) / (4 * n + 1);
    if ( d <= maxSpeed ) { // reached max speed
      d = maxSpeed;
      rampUpStepCount = stepCount;
    }
    if ( stepCount >= totalSteps / 2 ) { // reached halfway point
      rampUpStepCount = stepCount;
    }
  }
  else if ( stepCount >= totalSteps - rampUpStepCount ) { // ramp down phase
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
  delay(10);  // Minimum 1ms delay for charge pump stabilization
}

void msSet(int ms) {
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

void buttonJog(int jogSpeed, int jogSteps) {  
  int oldSpeed = maxSpeed;
  maxSpeed = jogSpeed;
  buttonStateFor = digitalRead(buttonPinFor);
  buttonStateRev = digitalRead(buttonPinRev);
  
  if ((buttonStateFor == HIGH) ^ (buttonStateRev == HIGH)) {
    wake();
    if (buttonStateFor == HIGH) {
      moveToPosition(stepPosition + jogSteps);
    } else {
      moveToPosition(stepPosition - jogSteps);
    }
    delay(500);
    sleep();
  }
  
  maxSpeed = oldSpeed;
}

void serialJog(int jogSpeed, int jogSteps, int dir) {
  int oldSpeed = maxSpeed;
  maxSpeed = jogSpeed;
  wake();
  if (dir == HIGH) {
    moveToPosition(stepPosition + jogSteps);
  } else {
    moveToPosition(stepPosition - jogSteps);
  }
  delay(500);
  sleep();
  maxSpeed = oldSpeed;
}

void moveTest() {
  wake();
  
  moveToPosition( 0 );
  delay(1000);

  moveToPosition( 200 );
  moveToPosition( 400 );
  moveToPosition( 600 );
  moveToPosition( 800 );

  delay(1000);

  moveToPosition( 400 );
  moveToPosition( 600 );
  moveToPosition( 200 );
  moveToPosition( 400 );
  moveToPosition( 0 );

  delay(1000);

  maxSpeed = 600;
  moveToPosition( 200 );
  moveToPosition( 400 );

  delay(1000);

  maxSpeed = 400;
  moveToPosition( 600 );
  moveToPosition( 800 );
  moveToPosition( 2000 );

  delay(1000);

  maxSpeed = 200;
  moveToPosition( 1000 );
  moveToPosition( 1200 );

  delay(1000);

  maxSpeed = 100;
  moveToPosition( 0 );

  sleep();
}

void microstepTest(int msSpeed) {
  wake();
  
  // Full Step
  msSet(1);

  moveToPosition(0);
  moveToPosition(200);
  moveToPosition(400);
  moveToPosition(200);
  moveToPosition(0);
  
  delay(1000);

  // Half Step
  msSet(2);

  moveToPosition(0);
  moveToPosition(400);
  moveToPosition(800);
  moveToPosition(400);
  moveToPosition(0);
  
  delay(1000);

  // Quarter Step
  msSet(4);

  moveToPosition(0);
  moveToPosition(800);
  moveToPosition(1600);
  moveToPosition(800);
  moveToPosition(0);
  
  delay(1000);

  // Eighth Step
  msSet(8);

  moveToPosition(0);
  moveToPosition(1600);
  moveToPosition(3200);
  moveToPosition(1600);
  moveToPosition(0);
  
  delay(1000);

  // Sixteenth Step
  msSet(16);

  moveToPosition(0);
  moveToPosition(3200);
  moveToPosition(6400);
  moveToPosition(3200);
  moveToPosition(0);
  
  delay(1000);

  // Return to Full Step
  msSet(1);

  sleep();
}

void loop() {

  //moveTest();

  //microstepTest(400);

  msSet(1);

  while (true) {
    if (Serial.available() > 0) {
      int inByte = Serial.read();
      Serial.print("Received: ");
      Serial.println(inByte);
      switch (inByte) {
        case 48:
          serialJog(200, 800, LOW);
          break;
        case 49:
          serialJog(200, 800, HIGH);
          break;
      }
    }
    //buttonJog(200, 800);
  }

}
