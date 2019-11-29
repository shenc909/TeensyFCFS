#include <Arduino.h>
#include "pindef.h"
#include <LED.h>

LED led = LED(LEDR, LEDB, LEDG);

void setup() {
  // put your setup code here, to run once:
}

void loop() {
  // put your main code here, to run repeatedly:
  led.on();
  delay(1000);
  led.red();
  delay(1000);
  led.green();
  delay(1000);
  led.blue();
  delay(1000);
  led.purple();
  delay(1000);
  led.white();
  delay(1000);
  led.rgb(100, 100, 0);
  delay(1000);
  led.off();
  delay(1000);
}