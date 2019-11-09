#include <Arduino.h>
#include "pindef.h"
#include <LED.h>

LED led = LED(LEDR, LEDG, LEDB);

void setup() {
  // put your setup code here, to run once:
}

void loop() {
  // put your main code here, to run repeatedly:
  led.on();
  delay(500);
  led.red();
  delay(500);
  led.green();
  delay(500);
  led.blue();
  delay(500);
  led.purple();
  delay(500);
  led.white();
  delay(500);
  led.rgb(0, 100, 100);
  delay(500);
  led.on();
  delay(500);
  led.off();
}