#ifndef _MOTORS_
#define _MOTORS_

#if (ARDUINO >= 100)
  #include "Arduino.h"
#else
  #include "WProgram.h"
#endif

#include <Wire.h>
#include <Servo.h>
#include <BME280I2C.h>         // import wire library for i2c, servo library for motors
#include <MPU6050.h>           // BME library for barometer, MPU library for IMU

class Motors                   // create motor control class
{
  public:
    double AltitudeControl(double input,double sensorVal,double initial);// altitude hold algorithm
    void RunMotors(Servo* Motor,int Gain);                // transmits commands to motors
    void MotorMix(Servo x, int y, int lower, int upper);  // motor mixing algorithm for roll, pitch and yaw motion
    void FlightControl(double m1,double m2,double m3,double m4,double m5,double m6);   // transmitts controller gains to motors
    void FullStop();                                      // all motors set to lowest command value or off position
    void StartUp();                                       // write an intial value to all motors                                      
};

#endif _MOTORS_
