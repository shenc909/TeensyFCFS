#include <Arduino.h>

#include "quaternionFilters.h"
#include "MPU9250.h"

#define AHRS false         // Set to false for basic data read
#define SerialDebug true  // Set to true to get Serial output for debugging

// Pin definitions
int intPin = 12;  // These can be changed, 2 and 3 are the Arduinos ext int pins
int myLed  = 13;  // Set up pin 13 led for toggling

#define I2Cclock 400000
#define I2Cport Wire
//#define MPU9250_ADDRESS MPU9250_ADDRESS_AD0   // Use either this line or the next to select which I2C address your device is using
#define MPU9250_ADDRESS MPU9250_ADDRESS_AD1

MPU9250 myIMU(MPU9250_ADDRESS, I2Cport, I2Cclock);


void setup() {
    Wire.begin();
    // TWBR = 12;  // 400 kbit/sec I2C speed
    Serial.begin(38400);

    while(!Serial){};

    // Set up the interrupt pin, its set as active high, push-pull
    pinMode(intPin, INPUT);
    digitalWrite(intPin, LOW);
    pinMode(myLed, OUTPUT);
    digitalWrite(myLed, HIGH);

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

void loop() {
  // put your main code here, to run repeatedly:
}