int sensorPin = A0;

void setup() {
  Serial.begin(9600);
}

void loop() {
  Serial.println(analogRead(sensorPin));
  delay(0); // delay between samples
}
