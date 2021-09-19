#ifndef _INIT_
#define _INIT_

#if (ARDUINO >= 100)
  #include "Arduino.h"
#else
  #include "WProgram.h"
#endif

#include <Wire.h>
#include <Servo.h>
#include <BME280I2C.h>         // import wire library for i2c, servo library for motors
#include "MPU9250.h"           // BME library for barometer, MPU library for IMU

class Initialise               // create initialisation class
{
  public:
    int M1 = 3;                // Front Left
    int M2 = 5;                // Front Right
    int M3 = 6;                // Rear Left
    int M4 = 9;                // Rear Right        Motor Pin assignments

    void initSensors();        // initialise the sensors
    void initMotors();         // initialise the motors
};

#endif
