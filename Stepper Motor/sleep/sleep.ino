// Functions for utilizing sleep features
// A4988 Stepper Motor Driver
// Author: Matt Kramer, Team ME46

void sleep(SLEEP_PIN) {
  digitalWrite(SLEEP_PIN, LOW);
}

void wake(SLEEP_PIN) {
  digitalWrite(SLEEP_PIN, HIGH);
  delay(10); // Minimum 1ms delay for charge pump stabilization
}

void setup() {
  
}

void loop() {
  
}
