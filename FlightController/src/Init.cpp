#include "init.h"
#include "motors.h"

#include <Wire.h>
#include <Servo.h>
#include <BME280I2C.h>
#include "MPU9250.h"

Servo Motor1,Motor2,Motor3,Motor4;  // create instances of the servo class for motor control
MPU9250 mpu;
BME280I2C bme;    // Default : forced mode, standby time = 1000 ms
                  // Oversampling = pressure ×1, temperature ×1, humidity ×1, filter off,
                  
void Initialise::initSensors()
{
  while(!bme.begin())
  {
    Serial.println("Could not find BME280 sensor!");
    delay(1000);
  }

  switch(bme.chipModel())
  {
     case BME280::ChipModel_BME280:
       Serial.println("Found BME280 sensor! Success.");
       break;
     case BME280::ChipModel_BMP280:
       Serial.println("Found BMP280 sensor! No Humidity available.");
       break;
     default:
       Serial.println("Found UNKNOWN sensor! Error!");
  }
}

void Initialise::initMotors()
{
  Motors motor;
  
  mpu.setup();
  delay(5000);
  mpu.calibrateAccelGyro();
  mpu.calibrateMag();                   // calibrate the IMU
  
  Motor1.attach(M1);
  Motor2.attach(M2);
  Motor3.attach(M3);
  Motor4.attach(M4);                    // intialise the motors to pins
  
  motor.RunMotors(&Motor1,1000);
  motor.RunMotors(&Motor2,1000);
  motor.RunMotors(&Motor3,1000);
  motor.RunMotors(&Motor4,1000);        // set motors to lowest command value
  
  delay(10000);
}
