int sensorPin = A0;

int sensorValue = 0;
float voltage = 0.0;
float dist = 0.0;

const int numSamples = 256;
float dists[numSamples];

void setup() {
  pinMode(sensorPin, INPUT);
  analogReference(DEFAULT);
  Serial.begin(9600);
}

void loop() {
  serialCountdown();
  for (int i = 0; i < numSamples; i++) {
    readDouble();
    logValues(i);
  }
  serialPrintArray();
  while (true);
}

void readDouble() {
  sensorValue = analogRead(sensorPin);
  sensorValue = analogRead(sensorPin);
  voltage = sensorValue * (5.0 / 1023.0);
  dist = voltage * 60 + 5; // Measured distance in mm
  Serial.println(dist, DEC);
  delay(1);
}

void logValues(int pos) {
  numSamples[pos] = dist;
}

void serialCountdown () {
  Serial.println("Starting logging in 5 seconds");
  delay(2000);
  Serial.println("5...");
  delay(1000);
  Serial.println("4...");
  delay(1000);
  Serial.println("3...");
  delay(1000);
  Serial.println("2...");
  delay(1000);
  Serial.println("1...");
  delay(1000);
  Serial.println("Starting now");
  delay(1000);
}

void serialPrintArray() {
  for (int i = 0; i < numSamples; i++) {
    Serial.print(dists[i]);
    Serial.print(", ");
  }
}
