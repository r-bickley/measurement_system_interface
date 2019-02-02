void simpleMove(int steps, STEP_PIN) {
  int interval = 900; // Control speed with rest interval
  for (int i = 0; i < steps; i++) {
    digitalWrite(STEP_PIN, HIGH);
    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(interval);
  }
}

void setup() {}

void loop() {}
