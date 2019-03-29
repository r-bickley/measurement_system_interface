int sensorPin = A0;

void setup() {
  Serial.begin(9600);
}

void loop() {
  //int sensorValue = analogRead(A0); // For reading twice to eliminate stray voltages
  // Or try Smoothing
  int sensorValue = analogRead(A0);
  float voltage = sensorValue * (5.0 / 1023.0);
  Serial.println(voltage);
  delay(100); // delay between samples in milliseconds
}
