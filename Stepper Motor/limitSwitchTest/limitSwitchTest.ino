#define XLIM1_PIN         22
#define XLIM2_PIN         24
#define YLIM1_PIN         50
#define YLIM2_PIN         52

int X1;
int X2;
int Y1;
int Y2;

void setup() {
  pinMode(XLIM1_PIN,      INPUT_PULLUP);
  pinMode(XLIM2_PIN,      INPUT_PULLUP);
  pinMode(YLIM1_PIN,      INPUT_PULLUP);
  pinMode(YLIM2_PIN,      INPUT_PULLUP);

  Serial.begin(9600);
}

void loop() {
  limitSwitchTest();
  delay(500);
}

void limitSwitchTest() {
  X1 = digitalRead(XLIM1_PIN);
  X2 = digitalRead(XLIM2_PIN);
  Y1 = digitalRead(YLIM1_PIN);
  Y2 = digitalRead(YLIM2_PIN);
  
  if (X1 == LOW) {
    Serial.println("XLIM_1");
  } else if (X2 == LOW) {
    Serial.println("XLIM_2");
  } else if (Y1 == LOW) {
    Serial.println("YLIM_1");
  } else if (Y2 == LOW) {
    Serial.println("YLIM_2");
  }
}
