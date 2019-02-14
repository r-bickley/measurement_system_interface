// Write to serial, arduino reads from serial to turn on or off LED
const int LED_PIN = 13;
int ledState = LOW;

void setup() {
  pinMode(LED_PIN,  OUTPUT);
  Serial.begin(9600);
}

void loop() {
  if (Serial.available() > 0) {
    int inByte = Serial.read();
    Serial.print("Received: ");
    Serial.println(inByte);
    switch (inByte) {
      case 48:
        digitalWrite(LED_PIN, LOW);
        Serial.println("Turn off");
        break;
      case 49:
        digitalWrite(LED_PIN, HIGH);
        Serial.println("Turn on");
        break;
    }
  }
}
