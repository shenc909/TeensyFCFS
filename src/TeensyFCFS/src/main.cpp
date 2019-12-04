#include <Arduino.h>
#include "pindef.h"
#include <ESC.h>
#include <quaternionFilters.h>
#include <MPU9250.h>
#include <PID_v1.h>

//Define Variables we'll be connecting to
double rollSetpoint, rollInput, rollOutput;
double pitchSetpoint, pitchInput, pitchOutput;

//Define the aggressive and conservative Tuning Parameters
double aggKp=0.4, aggKi=0.02, aggKd=0.1;
double consKp=0.1, consKi=0.005, consKd=0.025;

//Specify the links and initial tuning parameters
PID rollPID(&rollInput, &rollOutput, &rollSetpoint, consKp, consKi, consKd, DIRECT);
PID pitchPID(&pitchInput, &pitchOutput, &pitchSetpoint, consKp, consKi, consKd, DIRECT);

#define PIDMIN -200
#define PIDMAX 200


#define AHRS true         // Set to false for basic data read
#define SerialDebug false  // Set to true to get Serial output for debugging

#define I2Cclock 400000
#define I2Cport Wire
//#define MPU9250_ADDRESS MPU9250_ADDRESS_AD0   // Use either this line or the next to select which I2C address your device is using
// #define MPU9250_ADDRESS MPU9250_ADDRESS_AD1
#define MPU9250_ADDRESS 0x68

int intPin = 12;  // These can be changed, 2 and 3 are the Arduinos ext int pins

MPU9250 myIMU(MPU9250_ADDRESS, I2Cport, I2Cclock);
float imuTemp = 0;

#define SPEED_MIN (1000)                                  // Set the Minimum Speed in microseconds
#define SPEED_MAX (2000)                                  // Set the Minimum Speed in microseconds
#define LED_PIN (13)

ESC esc1 (ESC1, SPEED_MIN, SPEED_MAX, 500);                 // ESC_Name (ESC PIN, Minimum Value, Maximum Value, Default Speed, Arm Value)
ESC esc2 (ESC2, SPEED_MIN, SPEED_MAX, 500);
ESC esc3 (ESC3, SPEED_MIN, SPEED_MAX, 500);
ESC esc4 (ESC4, SPEED_MIN, SPEED_MAX, 500);

int throttleTop, throttleBtm, rollTop, rollBtm, pitchTop, pitchBtm, yawTop, yawBtm, switch1Top, switch1Btm, switch2Top, switch2Btm;
volatile int throttle, prev_throttle, roll, prev_roll, pitch, prev_pitch, yaw, prev_yaw, switch1, prev_switch1, switch2, prev_switch2;
int throttleOut, rollOut, pitchOut, yawOut, switch1Out, switch2Out;
bool armed = false;
int globalSpeed = 0;

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


/*///////////////////////////////////////////////////////////////
                        AHRS Code
*////////////////////////////////////////////////////////////////
void AHRSInit() {
  Wire.begin();
  Wire.setSDA(8);
  Wire.setSCL(7);
  // TWBR = 12;  // 400 kbit/sec I2C speed

  while(!Serial){};

  // Set up the interrupt pin, its set as active high, push-pull
  pinMode(intPin, INPUT);
  digitalWrite(intPin, LOW);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

  // Read the WHO_AM_I register, this is a good test of communication
  byte c = myIMU.readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);
  Serial.print(F("MPU9250 I AM 0x"));
  Serial.print(c, HEX);
  Serial.print(F(" I should be 0x"));
  Serial.println(0x71, HEX);

  if (c == 0x71) // WHO_AM_I should always be 0x71
  {
    Serial.println(F("MPU9250 is online..."));

    // Start by performing self test and reporting values
    myIMU.MPU9250SelfTest(myIMU.selfTest);
    Serial.print(F("x-axis self test: acceleration trim within : "));
    Serial.print(myIMU.selfTest[0],1); Serial.println("% of factory value");
    Serial.print(F("y-axis self test: acceleration trim within : "));
    Serial.print(myIMU.selfTest[1],1); Serial.println("% of factory value");
    Serial.print(F("z-axis self test: acceleration trim within : "));
    Serial.print(myIMU.selfTest[2],1); Serial.println("% of factory value");
    Serial.print(F("x-axis self test: gyration trim within : "));
    Serial.print(myIMU.selfTest[3],1); Serial.println("% of factory value");
    Serial.print(F("y-axis self test: gyration trim within : "));
    Serial.print(myIMU.selfTest[4],1); Serial.println("% of factory value");
    Serial.print(F("z-axis self test: gyration trim within : "));
    Serial.print(myIMU.selfTest[5],1); Serial.println("% of factory value");

    // Calibrate gyro and accelerometers, load biases in bias registers
    myIMU.calibrateMPU9250(myIMU.gyroBias, myIMU.accelBias);

    myIMU.initMPU9250();
    // Initialize device for active mode read of acclerometer, gyroscope, and
    // temperature
    Serial.println("MPU9250 initialized for active data mode....");

    // Read the WHO_AM_I register of the magnetometer, this is a good test of
    // communication
    byte d = myIMU.readByte(AK8963_ADDRESS, WHO_AM_I_AK8963);
    Serial.print("AK8963 ");
    Serial.print("I AM 0x");
    Serial.print(d, HEX);
    Serial.print(" I should be 0x");
    Serial.println(0x48, HEX);

    if (d != 0x48)
    {
      // Communication failed, stop here
      Serial.println(F("Communication failed, abort!"));
      Serial.flush();
      abort();
    }

    // Get magnetometer calibration from AK8963 ROM
    myIMU.initAK8963(myIMU.factoryMagCalibration);
    // Initialize device for active mode read of magnetometer
    Serial.println("AK8963 initialized for active data mode....");

    if (SerialDebug)
    {
      //  Serial.println("Calibration values: ");
      Serial.print("X-Axis factory sensitivity adjustment value ");
      Serial.println(myIMU.factoryMagCalibration[0], 2);
      Serial.print("Y-Axis factory sensitivity adjustment value ");
      Serial.println(myIMU.factoryMagCalibration[1], 2);
      Serial.print("Z-Axis factory sensitivity adjustment value ");
      Serial.println(myIMU.factoryMagCalibration[2], 2);
    }

    // Get sensor resolutions, only need to do this once
    myIMU.getAres();
    myIMU.getGres();
    myIMU.getMres();

    // The next call delays for 4 seconds, and then records about 15 seconds of
    // data to calculate bias and scale.
//    myIMU.magCalMPU9250(myIMU.magBias, myIMU.magScale);
    Serial.println("AK8963 mag biases (mG)");
    Serial.println(myIMU.magBias[0]);
    Serial.println(myIMU.magBias[1]);
    Serial.println(myIMU.magBias[2]);

    Serial.println("AK8963 mag scale (mG)");
    Serial.println(myIMU.magScale[0]);
    Serial.println(myIMU.magScale[1]);
    Serial.println(myIMU.magScale[2]);
//    delay(2000); // Add delay to see results before serial spew of data

    if(SerialDebug)
    {
      Serial.println("Magnetometer:");
      Serial.print("X-Axis sensitivity adjustment value ");
      Serial.println(myIMU.factoryMagCalibration[0], 2);
      Serial.print("Y-Axis sensitivity adjustment value ");
      Serial.println(myIMU.factoryMagCalibration[1], 2);
      Serial.print("Z-Axis sensitivity adjustment value ");
      Serial.println(myIMU.factoryMagCalibration[2], 2);
    }

    } // if (c == 0x71)
  else
  {
    Serial.print("Could not connect to MPU9250: 0x");
    Serial.println(c, HEX);

    // Communication failed, stop here
    Serial.println(F("Communication failed, abort!"));
    Serial.flush();
    abort();
  }
}

void AHRSLoop() {
  // If intPin goes high, all data registers have new data
  // On interrupt, check if data ready interrupt
  if (myIMU.readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01)
  {
    myIMU.readAccelData(myIMU.accelCount);  // Read the x/y/z adc values

    // Now we'll calculate the accleration value into actual g's
    // This depends on scale being set
    myIMU.ax = (float)myIMU.accelCount[0] * myIMU.aRes; // - myIMU.accelBias[0];
    myIMU.ay = (float)myIMU.accelCount[1] * myIMU.aRes; // - myIMU.accelBias[1];
    myIMU.az = (float)myIMU.accelCount[2] * myIMU.aRes; // - myIMU.accelBias[2];

    myIMU.readGyroData(myIMU.gyroCount);  // Read the x/y/z adc values

    // Calculate the gyro value into actual degrees per second
    // This depends on scale being set
    myIMU.gx = (float)myIMU.gyroCount[0] * myIMU.gRes;
    myIMU.gy = (float)myIMU.gyroCount[1] * myIMU.gRes;
    myIMU.gz = (float)myIMU.gyroCount[2] * myIMU.gRes;

    myIMU.readMagData(myIMU.magCount);  // Read the x/y/z adc values

    // Calculate the magnetometer values in milliGauss
    // Include factory calibration per data sheet and user environmental
    // corrections
    // Get actual magnetometer value, this depends on scale being set
    myIMU.mx = (float)myIMU.magCount[0] * myIMU.mRes
               * myIMU.factoryMagCalibration[0] - myIMU.magBias[0];
    myIMU.my = (float)myIMU.magCount[1] * myIMU.mRes
               * myIMU.factoryMagCalibration[1] - myIMU.magBias[1];
    myIMU.mz = (float)myIMU.magCount[2] * myIMU.mRes
               * myIMU.factoryMagCalibration[2] - myIMU.magBias[2];
  } // if (readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01)

  // Must be called before updating quaternions!
  myIMU.updateTime();

  // Sensors x (y)-axis of the accelerometer is aligned with the y (x)-axis of
  // the magnetometer; the magnetometer z-axis (+ down) is opposite to z-axis
  // (+ up) of accelerometer and gyro! We have to make some allowance for this
  // orientationmismatch in feeding the output to the quaternion filter. For the
  // MPU-9250, we have chosen a magnetic rotation that keeps the sensor forward
  // along the x-axis just like in the LSM9DS0 sensor. This rotation can be
  // modified to allow any convenient orientation convention. This is ok by
  // aircraft orientation standards! Pass gyro rate as rad/s
  MahonyQuaternionUpdate(myIMU.ax, myIMU.ay, myIMU.az, myIMU.gx * DEG_TO_RAD,
                         myIMU.gy * DEG_TO_RAD, myIMU.gz * DEG_TO_RAD, myIMU.my,
                         myIMU.mx, myIMU.mz, myIMU.deltat);

  if (!AHRS)
  {
    myIMU.delt_t = millis() - myIMU.count;
    if (myIMU.delt_t > 500)
    {
      if(SerialDebug)
      {
        // Print acceleration values in milligs!
        Serial.print("X-acceleration: "); Serial.print(1000 * myIMU.ax);
        Serial.print(" mg ");
        Serial.print("Y-acceleration: "); Serial.print(1000 * myIMU.ay);
        Serial.print(" mg ");
        Serial.print("Z-acceleration: "); Serial.print(1000 * myIMU.az);
        Serial.println(" mg ");

        // Print gyro values in degree/sec
        Serial.print("X-gyro rate: "); Serial.print(myIMU.gx, 3);
        Serial.print(" degrees/sec ");
        Serial.print("Y-gyro rate: "); Serial.print(myIMU.gy, 3);
        Serial.print(" degrees/sec ");
        Serial.print("Z-gyro rate: "); Serial.print(myIMU.gz, 3);
        Serial.println(" degrees/sec");

        // Print mag values in degree/sec
        Serial.print("X-mag field: "); Serial.print(myIMU.mx);
        Serial.print(" mG ");
        Serial.print("Y-mag field: "); Serial.print(myIMU.my);
        Serial.print(" mG ");
        Serial.print("Z-mag field: "); Serial.print(myIMU.mz);
        Serial.println(" mG");

        myIMU.tempCount = myIMU.readTempData();  // Read the adc values
        // Temperature in degrees Centigrade
        myIMU.temperature = ((float) myIMU.tempCount) / 333.87 + 21.0;
        // Print temperature in degrees Centigrade
        Serial.print("Temperature is ");  Serial.print(myIMU.temperature, 1);
        Serial.println(" degrees C");
      }

      myIMU.count = millis();
      digitalWrite(LED_PIN, !digitalRead(LED_PIN));  // toggle led
    } // if (myIMU.delt_t > 500)
  } // if (!AHRS)
  else
  {
    // Serial print and/or display at 0.5 s rate independent of data rates
    myIMU.delt_t = millis() - myIMU.count;

    // update LCD once per half-second independent of read rate
    if (myIMU.delt_t > 500)
    {
      if(SerialDebug)
      {
        Serial.print("ax = ");  Serial.print((int)1000 * myIMU.ax);
        Serial.print(" ay = "); Serial.print((int)1000 * myIMU.ay);
        Serial.print(" az = "); Serial.print((int)1000 * myIMU.az);
        Serial.println(" mg");

        Serial.print("gx = ");  Serial.print(myIMU.gx, 2);
        Serial.print(" gy = "); Serial.print(myIMU.gy, 2);
        Serial.print(" gz = "); Serial.print(myIMU.gz, 2);
        Serial.println(" deg/s");

        Serial.print("mx = ");  Serial.print((int)myIMU.mx);
        Serial.print(" my = "); Serial.print((int)myIMU.my);
        Serial.print(" mz = "); Serial.print((int)myIMU.mz);
        Serial.println(" mG");

        Serial.print("q0 = ");  Serial.print(*getQ());
        Serial.print(" qx = "); Serial.print(*(getQ() + 1));
        Serial.print(" qy = "); Serial.print(*(getQ() + 2));
        Serial.print(" qz = "); Serial.println(*(getQ() + 3));
      }

// Define output variables from updated quaternion---these are Tait-Bryan
// angles, commonly used in aircraft orientation. In this coordinate system,
// the positive z-axis is down toward Earth. Yaw is the angle between Sensor
// x-axis and Earth magnetic North (or true North if corrected for local
// declination, looking down on the sensor positive yaw is counterclockwise.
// Pitch is angle between sensor x-axis and Earth ground plane, toward the
// Earth is positive, up toward the sky is negative. Roll is angle between
// sensor y-axis and Earth ground plane, y-axis up is positive roll. These
// arise from the definition of the homogeneous rotation matrix constructed
// from quaternions. Tait-Bryan angles as well as Euler angles are
// non-commutative; that is, the get the correct orientation the rotations
// must be applied in the correct order which for this configuration is yaw,
// pitch, and then roll.
// For more see
// http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
// which has additional links.
      myIMU.yaw   = atan2(2.0f * (*(getQ()+1) * *(getQ()+2) + *getQ()
                    * *(getQ()+3)), *getQ() * *getQ() + *(getQ()+1)
                    * *(getQ()+1) - *(getQ()+2) * *(getQ()+2) - *(getQ()+3)
                    * *(getQ()+3));
      myIMU.pitch = -asin(2.0f * (*(getQ()+1) * *(getQ()+3) - *getQ()
                    * *(getQ()+2)));
      myIMU.roll  = atan2(2.0f * (*getQ() * *(getQ()+1) + *(getQ()+2)
                    * *(getQ()+3)), *getQ() * *getQ() - *(getQ()+1)
                    * *(getQ()+1) - *(getQ()+2) * *(getQ()+2) + *(getQ()+3)
                    * *(getQ()+3));
      myIMU.pitch *= RAD_TO_DEG;
      myIMU.yaw   *= RAD_TO_DEG;

      // Declination of SparkFun Electronics (40°05'26.6"N 105°11'05.9"W) is
      // 	8° 30' E  ± 0° 21' (or 8.5°) on 2016-07-19
      // - http://www.ngdc.noaa.gov/geomag-web/#declination
      myIMU.yaw  -= 8.5;
      myIMU.roll *= RAD_TO_DEG;

      imuTemp = myIMU.roll;
      myIMU.roll = myIMU.pitch;
      myIMU.pitch = -imuTemp;

      if(SerialDebug)
      {
        Serial.print("Yaw, Pitch, Roll: ");
        Serial.print(myIMU.yaw, 2);
        Serial.print(", ");
        Serial.print(myIMU.pitch, 2);
        Serial.print(", ");
        Serial.println(myIMU.roll, 2);

        Serial.print("rate = ");
        Serial.print((float)myIMU.sumCount / myIMU.sum, 2);
        Serial.println(" Hz");
      }

      myIMU.count = millis();
      myIMU.sumCount = 0;
      myIMU.sum = 0;
    } // if (myIMU.delt_t > 500)
  } // if (AHRS)
}

/*///////////////////////////////////////////////////////////////
                        PID Code
*////////////////////////////////////////////////////////////////

void PIDInit() {
  rollSetpoint = 0;
  pitchSetpoint = 0;
  rollInput = myIMU.roll;
  pitchInput = myIMU.pitch;
  rollPID.SetSampleTime(50);
  pitchPID.SetSampleTime(50);
  rollPID.SetOutputLimits(PIDMIN, PIDMAX);
  pitchPID.SetOutputLimits(PIDMIN, PIDMAX);
  rollPID.SetMode(AUTOMATIC);
  pitchPID.SetMode(AUTOMATIC);
}

void PIDLoop() {
  rollInput = myIMU.roll;
  pitchInput = myIMU.pitch;

  double rollError = abs(rollSetpoint-rollInput); //distance away from setpoint
  if(rollError<10)
  {  //we're close to setpoint, use conservative tuning parameters
    rollPID.SetTunings(consKp, consKi, consKd);
  }
  else
  {
     //we're far from setpoint, use aggressive tuning parameters
     rollPID.SetTunings(aggKp, aggKi, aggKd);
  }

  double pitchError = abs(pitchSetpoint-pitchInput); //distance away from setpoint
  if(pitchError<10)
  {  //we're close to setpoint, use conservative tuning parameters
    pitchPID.SetTunings(consKp, consKi, consKd);
  }
  else
  {
     //we're far from setpoint, use aggressive tuning parameters
     pitchPID.SetTunings(aggKp, aggKi, aggKd);
  }

  rollPID.Compute();
  pitchPID.Compute();
}


/*///////////////////////////////////////////////////////////////
                        MAIN CODE
*////////////////////////////////////////////////////////////////
void setup() {
  // put your setup code here, to run once:

  throttleBtm = 1108;
  throttleTop = 1763;
  rollBtm = 1087;
  rollTop = 1907;
  pitchBtm = 1164;
  pitchTop = 1809;
  yawBtm = 1118;
  yawTop = 1883;
  switch1Btm = 1001;
  switch1Top = 2017;
  switch2Btm = 1001;
  switch2Top = 2017;

  attachInterrupt(digitalPinToInterrupt(CH1), rising1, RISING);
  attachInterrupt(digitalPinToInterrupt(CH2), rising2, RISING);
  attachInterrupt(digitalPinToInterrupt(CH3), rising3, RISING);
  attachInterrupt(digitalPinToInterrupt(CH4), rising4, RISING);
  attachInterrupt(digitalPinToInterrupt(CH5), rising5, RISING);
  attachInterrupt(digitalPinToInterrupt(CH6), rising6, RISING);
  Serial.begin(115200);

  pinMode(LED_PIN, OUTPUT);

  delay(2000);

  AHRSInit();
  PIDInit();

  // calibrate();

}

void loop() {
  // put your main code here, to run repeatedly:

  throttleOut = map(throttle, throttleBtm, throttleTop, 0, 100);
  rollOut = map(roll, rollBtm, rollTop, -100, 100);
  pitchOut = map(pitch, pitchBtm, pitchTop, -100, 100);
  yawOut = map(yaw, yawBtm, yawTop, -100, 100);
  switch1Out = map(switch1, switch1Btm, switch1Top, 0, 100);
  switch2Out = map(switch2, switch2Btm, switch2Top, 0, 100);
  
  // Serial.print(throttleBtm);
  // Serial.print("\t");
  // Serial.print(throttleTop);
  // Serial.print("\t");
  // Serial.print(rollBtm);
  // Serial.print("\t");
  // Serial.print(rollTop);
  // Serial.print("\t");
  // Serial.print(pitchBtm);
  // Serial.print("\t");
  // Serial.print(pitchTop);
  // Serial.print("\t");
  // Serial.println(yawBtm);

  // Serial.print(throttleOut);
  // Serial.print("\t | \t");
  // Serial.print(rollOut);
  // Serial.print("\t | \t");
  // Serial.print(pitchOut);
  // Serial.print("\t | \t");
  // Serial.print(yawOut);
  // Serial.print("\t | \t");
  // Serial.print(switch1Out);
  // Serial.print("\t | \t");
  // Serial.println(switch2Out);

  // delay(100);

  // if (throttleOut < 5 && yawOut < -95){
  //   if(armed == false){
  //     esc1.arm();
  //     esc2.arm();
  //     esc3.arm();
  //     esc4.arm();
  //     digitalWrite(LED_PIN, HIGH);
  //     delay(1000);
  //     armed = true;
  //   }else{
  //     esc1.stop();
  //     esc2.stop();
  //     esc3.stop();
  //     esc4.stop();
  //     digitalWrite(LED_PIN, LOW);
  //     delay(1000);
  //     armed = false;
  //   }
    
  // }

  // if(armed == true){
  //   globalSpeed = map(throttleOut, 0, 100, SPEED_MIN, SPEED_MAX);
  //   if(globalSpeed<0){globalSpeed = 0;}
  //   esc1.speed(globalSpeed);
  //   esc2.speed(globalSpeed);
  //   esc3.speed(globalSpeed);
  //   esc4.speed(globalSpeed);
  // }

  AHRSLoop();

  globalSpeed = map(throttleOut, 0, 100, SPEED_MIN, SPEED_MAX);

  PIDLoop();

  // Serial.print(myIMU.yaw);
  // Serial.print("\t");
  // Serial.print(myIMU.pitch);
  // Serial.print("\t");
  // Serial.print(myIMU.roll);
  // Serial.print("\n");

  esc1.speed(globalSpeed - rollOutput - pitchOutput);
  esc2.speed(globalSpeed + rollOutput - pitchOutput);
  esc3.speed(globalSpeed + rollOutput + pitchOutput);
  esc4.speed(globalSpeed - rollOutput + pitchOutput);
  

}