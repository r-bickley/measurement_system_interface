// Function for manually jogging a stepper motor
// A4988 driver
// Author: Matthew Kramer, Team ME46

//#include Stepper4_linearSpeedInterrupt
//#include sleep

void jogPosition(int jogSpeed, int jogSteps) {
  int oldSpeed = maxSpeed;
  maxSpeed = jogSpeed;
  buttonStateFor = digitalRead(BUTTON_FOR);
  buttonStateRev = digitalRead(BUTTON_REV);

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

void setup() {}

void loop() {}
