#include "sensors.h"

#include <Wire.h>
#include <BME280I2C.h>
#include "MPU9250.h"

extern MPU9250 mpu;        // extern allows other files to use these values
extern BME280I2C bme;      // Default : forced mode, standby time = 1000 ms
                           // Oversampling = pressure ×1, temperature ×1, humidity ×1, filter off,
                                          
double Sensors::ALT()
{
  float temp(NAN), hum(NAN), pres(NAN);
  
  BME280::TempUnit tempUnit(BME280::TempUnit_Celsius);
  BME280::PresUnit presUnit(BME280::PresUnit_Pa); 
    
  bme.read(pres, temp, hum, tempUnit, presUnit); // read the current tempurature and atmospheric pressure values
  
  T = temp + 273;
  
  num = log(pres/Pb) * T * R;
  dnum = g * M * -1;                             // used predefined constants to calculated altitude.
  h = (num/dnum)+ hb;
  
  return(h);                                     // return altitude value
}

double *Sensors::IMU()
{
  static double Axis[3];                         // created a static array to hold output
  
  mpu.update();
  mpu.print();
  
  Axis[0] = mpu.getRoll();
  Axis[1] = mpu.getPitch();                      // save pitch roll and yaw values to array
  Axis[2] = mpu.getYaw();
  
  return(Axis);                                  // return array
}
