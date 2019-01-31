#define DIR_PIN          2
#define STEP_PIN         3
#define ENABLE_PIN       4
#define SLEEP_PIN        5
#define RESET_PIN        6

#define STEP_HIGH        PORTD |=  0b00001000;
#define STEP_LOW         PORTD &= ~0b00001000;

#define TIMER1_INTERRUPTS_ON    TIMSK1 |=  (1 << OCIE1A);
#define TIMER1_INTERRUPTS_OFF   TIMSK1 &= ~(1 << OCIE1A);

unsigned int c0;
const int buttonPinFor = 8;
const int buttonPinRev = 9;
int buttonStateFor = 0;
int buttonStateRev = 0;

void setup() {
  pinMode(STEP_PIN,   OUTPUT);
  pinMode(DIR_PIN,    OUTPUT);
  pinMode(ENABLE_PIN, OUTPUT);
  pinMode(SLEEP_PIN,  OUTPUT);
  pinMode(RESET_PIN,  OUTPUT);
  pinMode(buttonPinFor,  INPUT);
  pinMode(buttonPinRev,  INPUT);

  digitalWrite(SLEEP_PIN, HIGH);
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

void jogPosition() {
  buttonStateFor = digitalRead(buttonPinFor);
  buttonStateRev = digitalRead(buttonPinRev);
  if ((buttonStateFor == HIGH) ^ (buttonStateRev == HIGH)) {
    digitalWrite(SLEEP_PIN, HIGH);
    if (buttonStateFor == HIGH) {
      moveToPosition(stepPosition + 200);
    } else {
      moveToPosition(stepPosition - 200);
    }
    delay(500);
    digitalWrite(SLEEP_PIN, LOW);
  }
}

void loop() {

  //moveToPosition(  1600 );
  //moveToPosition( -1600 );
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

  digitalWrite(SLEEP_PIN, LOW);

  maxSpeed = 200;
  while (true) {
    jogPosition();
  }

}
