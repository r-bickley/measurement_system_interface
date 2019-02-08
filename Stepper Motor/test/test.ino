const int button1Pin = 8;
const int button2Pin = 9;
const int ledPin = 13;

int button1State = 0;
int button2State = 0;

void setup() {
  pinMode(ledPin,     OUTPUT);
  pinMode(button1Pin, INPUT);
  pinMode(button2Pin, INPUT);
}

void loop() {
  button1State = digitalRead(button1Pin);
  button2State = digitalRead(button2Pin);

  if ((button1State == HIGH) ^ (button2State == HIGH)) {
    digitalWrite(ledPin, HIGH);
  } else {
    digitalWrite(ledPin, LOW);
  }
}
