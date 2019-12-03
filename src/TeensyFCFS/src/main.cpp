#include <Arduino.h>
#include "pindef.h"

unsigned int throttleTop, throttleBtm, rollTop, rollBtm, pitchTop, pitchBtm, yawTop, yawBtm, switch1Top, switch1Btm, switch2Top, switch2Btm;
volatile unsigned int throttle, prev_throttle, roll, prev_roll, pitch, prev_pitch, yaw, prev_yaw, switch1, prev_switch1, switch2, prev_switch2;
unsigned int throttleOut, rollOut, pitchOut, yawOut, switch1Out, switch2Out;

void rising1();
void rising2();
void rising3();
void rising4();
void rising5();
void rising6();
void falling1();
void falling2();
void falling3();
void falling4();
void falling5();
void falling6();


void setup() {
  // put your setup code here, to run once:

  attachInterrupt(digitalPinToInterrupt(CH1), rising1, RISING);
  attachInterrupt(digitalPinToInterrupt(CH2), rising2, RISING);
  attachInterrupt(digitalPinToInterrupt(CH3), rising3, RISING);
  attachInterrupt(digitalPinToInterrupt(CH4), rising4, RISING);
  attachInterrupt(digitalPinToInterrupt(CH5), rising5, RISING);
  attachInterrupt(digitalPinToInterrupt(CH6), rising6, RISING);

}

void loop() {
  // put your main code here, to run repeatedly:

  throttleOut = map(throttle, throttleBtm, throttleTop, 0, 100);
  rollOut = map(roll, rollBtm, rollTop, -100, 100);
  pitchOut = map(pitch, pitchBtm, pitchTop, -100, 100);
  yawOut = map(yaw, yawBtm, yawTop, -100, 100);
  switch1Out = map(switch1, switch1Btm, switch1Top, 0, 1);
  rollOut = map(switch2, switch2Btm, switch2Top, 0, 1);
  
}

/*///////////////////////////////////////////////////////////////
                        REMOTE CODE
*////////////////////////////////////////////////////////////////
void calibrate() {
  throttleTop = throttleBtm = throttle;
  rollTop = rollBtm = roll;
  pitchTop = pitchBtm = pitch;
  yawTop = yawBtm = yaw;

  Serial.println("Calibrating... Move the sticks around...");
  Serial.println("Press any key to continue...");
  while(!Serial.available()){
      if(throttle > throttleTop){
      throttleTop = throttle;
      }
      if(throttle < throttleBtm){
      throttleBtm = throttle;
      }
      if(roll > rollTop){
      rollTop = roll;
      }
      if(roll < rollBtm){
      rollBtm = roll;
      }
      if(pitch > pitchTop){
      pitchTop = pitch;
      }
      if(pitch < pitchBtm){
      pitchBtm = pitch;
      }
      if(yaw > yawTop){
      yawTop = yaw;
      }
      if(yaw < yawBtm){
      yawBtm = yaw;
      }
  }
  while(Serial.available()){
      Serial.read();
  }
}

/*///////////////////////////////////////////////////////////////
                        REMOTE ISRs
*////////////////////////////////////////////////////////////////
void rising1() {
  attachInterrupt(digitalPinToInterrupt(CH1), falling1, FALLING);
  prev_throttle = micros();
}
 
void falling1() {
  attachInterrupt(digitalPinToInterrupt(CH1), rising1, RISING);
  throttle = micros()-prev_throttle;
}

void rising2() {
  attachInterrupt(digitalPinToInterrupt(CH2), falling2, FALLING);
  prev_roll = micros();
}
 
void falling2() {
  attachInterrupt(digitalPinToInterrupt(CH2), rising2, RISING);
  roll = micros()-prev_roll;
}

void rising3() {
  attachInterrupt(digitalPinToInterrupt(CH3), falling3, FALLING);
  prev_pitch = micros();
}
 
void falling3() {
  attachInterrupt(digitalPinToInterrupt(CH3), rising3, RISING);
  pitch = micros()-prev_pitch;
}

void rising4() {
  attachInterrupt(digitalPinToInterrupt(CH4), falling4, FALLING);
  prev_yaw = micros();
}
 
void falling4() {
  attachInterrupt(digitalPinToInterrupt(CH4), rising4, RISING);
  yaw = micros()-prev_yaw;
}

void rising5() {
  attachInterrupt(digitalPinToInterrupt(CH5), falling5, FALLING);
  prev_switch1 = micros();
}
 
void falling5() {
  attachInterrupt(digitalPinToInterrupt(CH5), rising5, RISING);
  switch1 = micros()-prev_switch1;
}

void rising6() {
  attachInterrupt(digitalPinToInterrupt(CH6), falling6, FALLING);
  prev_switch2 = micros();
}
 
void falling6() {
  attachInterrupt(digitalPinToInterrupt(CH6), rising6, RISING);
  switch2 = micros()-prev_switch2;
}