#include <Arduino.h>
#include "pindef.h"

int throttleTop, throttleBtm, rollTop, rollBtm, pitchTop, pitchBtm, yawTop, yawBtm;
volatile int throttle, roll, pitch, yaw;

void setup() {
  // put your setup code here, to run once:

  attachInterrupt(digitalPinToInterrupt(CH1), rising1, RISING);
  attachInterrupt(digitalPinToInterrupt(CH2), rising2, RISING);
  attachInterrupt(digitalPinToInterrupt(CH3), rising3, RISING);
  attachInterrupt(digitalPinToInterrupt(CH4), rising4, RISING);

}

void loop() {
  // put your main code here, to run repeatedly:
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
  attachInterrupt(0, falling1, FALLING);
  prev_throttle = micros();
}
 
void falling1() {
  attachInterrupt(0, rising1, RISING);
  throttle = micros()-prev_throttle;
}

void rising2() {
  attachInterrupt(1, falling2, FALLING);
  prev_roll = micros();
}
 
void falling2() {
  attachInterrupt(1, rising2, RISING);
  roll = micros()-prev_roll;
}

void rising3() {
  attachInterrupt(2, falling3, FALLING);
  prev_pitch = micros();
}
 
void falling3() {
  attachInterrupt(2, rising3, RISING);
  pitch = micros()-prev_pitch;
}

void rising4() {
  attachInterrupt(3, falling4, FALLING);
  prev_yaw = micros();
}
 
void falling4() {
  attachInterrupt(3, rising4, RISING);
  yaw = micros()-prev_yaw;
}