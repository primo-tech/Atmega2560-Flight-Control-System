#ifndef _SENSOR_
#define _SENSOR_

#if (ARDUINO >= 100)
  #include "Arduino.h"
#else
  #include "WProgram.h"
#endif

#include <Wire.h>
#include <BME280I2C.h>         // import wire library for i2c
#include "MPU9250.h"           // BME library for barometer, MPU library for IMU

class Sensors
{
  public:
    double ALT();           // calculated the current altitude above sea level
    double *IMU();          // calculate the current pose roll, pitch, yaw(x,y,z)
  private:
    /*
    * BAROMETER VALUES
    */
    double R = 8.3144598;
    double g = 9.80665;
    double M = 0.0289644;        
    double Pb = 101325;
    double num=0,dnum=0,h=0,hb = 0;
    double T;
};

#endif
