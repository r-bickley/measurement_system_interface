#include <Nextion.h>

// Declare Nextion Objects
// (page, component id, component name)
NexButton bOn = NexButton(0, 1, "bOn");
NexButton bOff = NexButton(0, 2, "bOff");

// Register button object to touch even list
NexTouch *nex_listen_list[] = {
  &bOn,
  &bOff,
  NULL
};

void bOnPopCallback(void *ptr) {
  digitalWrite(LED_BUILTIN, HIGH);
}

void bOffPopCallback(void *ptr) {
  digitalWrite(LED_BUILTIN, LOW);
}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);

  Serial.begin(9600);

  nexInit();

  bOn.attachPop(bOnPopCallback, &bOn);
  bOff.attachPop(bOffPopCallback, &bOff);
}

void loop() {
  nexLoop(nex_listen_list);
}
