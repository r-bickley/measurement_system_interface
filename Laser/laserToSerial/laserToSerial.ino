int sensorPin = A0;
int sensorValue = 0;
float voltage = 0.0;

const int numReadings = 10;
int readings[numReadings];
int readIndex = 0;
int total = 0;
int average = 0;

void setup() {
  Serial.begin(9600);
  // init readings to 0
  for (int thisReading = 0; thisReading < numReadings; thisReading++) {
    readings[thisReading] = 0;
  }
}

void loop() {
  readDouble();
  //readAvg();
}

void readDouble() {
  sensorValue = analogRead(A0);
  sensorValue = analogRead(A0);
  voltage = sensorValue * (5.0 / 1023.0);
  Serial.println(voltage);
  delay(1);
}

void readAvg() {
  // subtract the last reading
  total = total - readings[readIndex];
  // read from the sensor
  readings[readIndex] = analogRead(sensorPin);
  // add the reading to the total
  total = total + readings[readIndex];
  // advance to the next position in the array
  readIndex = readIndex + 1;

  // if at the end of the array, go to beginning
  if (readIndex >= numReadings) {
    readIndex = 0;
  }

  // calc avg
  average = total / numReadings;
  // send over serial
  Serial.println(average);
  delay(1);
}
