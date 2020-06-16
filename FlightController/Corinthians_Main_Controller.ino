//--------------------------------------------------------------------------------------------------------------------
/*
 *                                            CLASS HEADER FILES
 */
//--------------------------------------------------------------------------------------------------------------------
#include "init.h"
#include "motors.h"
#include "sensor.h"
#include <StateSpaceControl.h>
//---------------------------------------------------------------------------------------------------------------
/*
 *                                    VARIABLE/CONSTANT DEFINITIONS
 */
//---------------------------------------------------------------------------------------------------------------

bool breakout = 0;
/*
 * RECEIVER VARIABLES
 */
unsigned long int aa,bb,cc;
int x[15],ch1[15],ch[7],ii; //specifing arrays and variables to store values
/*
 *  CONTROL VARIABLES
 */

//--------------------------------------------------------------------------------------------------------------------
/*
 *                                         Model/Controller
 */
//--------------------------------------------------------------------------------------------------------------------
Model<14,6,6> HexaModel;
StateSpaceController<14,6,6,true,true> controller(HexaModel);
Matrix<6> y;
//--------------------------------------------------------------------------------------------------------------------
/*
 *                                         CLASS OBJECT INSTANTIATIONS
 */
//--------------------------------------------------------------------------------------------------------------------
Motors motor;                   // Instantiate motor control class
Initialise inital;              // Instantiate initialisation class
Sensors sensor;                  // Instantiate Sensor class
//--------------------------------------------------------------------------------------------------------------
/*
 *                                   COMPONENT INITIALISATION LOOP 
 */
//--------------------------------------------------------------------------------------------------------------
void setup() 
{
  Serial.begin(9600);
  Wire.begin();                // join i2c bus with address #1
  
  //Serial.println("Initialize BME280");
  //Serial.println("Initialize MPU6050");
  
HexaModel.A << 1, 0.01, 0, 0,       0, 0,    0, 0,      5.756e-08,      5.756e-08,      5.756e-08,     5.7562e-08,  5.7562e-08,  5.7562e-08,
               0, 1,    0, 0,       0, 0,    0, 0,      1.072e-05,      1.072e-05,      1.072e-05,     1.0724e-05,  1.0724e-05,  1.0724e-05,
               0, 0,    1, 0.01,    0, 0,    0, 0,      0.0001152,      0.000115,      -0.000115,     -0.0001,      0,           0,
               0, 0,    0, 1,       0, 0,    0, 0,      0.021458,       0.0214,        -0.021458,     -0.0214,      0,           0,
               0, 0,    0, 0,       1, 0.01, 0, 0,      -4.0162e-06,   -4.016e-06,     -4.016227e-06, -4.0162e-06,  8.0324e-06,  8.03245e-06,
               0, 0,    0, 0,       0, 1,    0, 0,      -0.000748,     -0.000748,      -0.000748,     -0.000748,    0.0015,      0.00149,
               0, 0,    0, 0,       0, 0,    1, 0.01,   -9.6098e-08,    9.60983e-08,    9.609833e-08, -9.6098e-08, -9.6098e-08,  9.60983e-08,
               0, 0,    0, 0,       0, 0,    0, 1,      -1.79e-05,      1.7904e-05,     1.790439e-05, -1.7904e-05, -1.7904e-05,  1.7904e-05,
               0, 0,    0, 0,       0, 0,    0, 0,      0.6426,         0,              0,             0,           0,           0,
               0, 0,    0, 0,       0, 0,    0, 0,      0,              0.6426,         0,             0,           0,           0,
               0, 0,    0, 0,       0, 0,    0, 0,      0,              0,              0.6426,        0,           0,           0,
               0, 0,    0, 0,       0, 0,    0, 0,      0,              0,              0,             0.6426,      0,           0,
               0, 0,    0, 0,       0, 0,    0, 0,      0,              0,              0,             0,           0.6426,      0,
               0, 0,    0, 0,       0, 0,    0, 0,      0,              0,              0,             0,           0,           0.6426;

HexaModel.B << 1.024e-07,   1.02444e-07,  1.02445e-07,     1.02444e-07,  1.02444e-07, 1.024e-07,
               2.967e-05,   2.9673e-05,   2.96732e-05,     2.9673e-05,   2.967e-05,   2.967e-05,
               0.0002049,   0.000205,    -0.000205,       -0.0002,       0,           0,
               0.05937,     0.05937,     -0.059,          -0.0593,       0,           0,
              -7.14778e-06,-7.14778e-06, -7.147e-06,      -7.1477e-06,   1.42955e-05, 1.429e-05,
              -0.00207,    -0.00207,     -0.00207,        -0.00207,      0.00414,     0.0041,
              -1.7103e-07,  1.7103e-07,   1.7103e-07,     -1.71028e-07, -1.71028e-07, 1.7102e-07,
              -4.9539e-05,  4.954e-05,    4.9539e-05,     -4.953e-05,   -4.953e-05,   4.95e-05,
               4.16618,     0,        0,        0,     0,        0,
               0,           4.166,    0,        0,     0,        0,
               0,           0,        4.1661,   0,     0,        0,
               0,           0,        0,        4.166, 0,        0,
               0,           0,        0,        0,     4.166,    0,
               0,           0,        0,        0,     0,        4.166;
                                     
HexaModel.C << 1,0,0,0,0,0,0,0,0,0,0,0,0,0,
               0,0,1,0,0,0,0,0,0,0,0,0,0,0,
               0,0,0,0,1,0,0,0,0,0,0,0,0,0,     // Sensor Matrix
               0,0,0,0,0,0,1,0,0,0,0,0,0,0,
               0,0,0,0,0,0,0,0,1,0,0,0,0,0,
               0,0,0,0,0,0,0,0,0,0,0,1,0,0;

 HexaModel.D << 0,0,0,0,0,0,
                0,0,0,0,0,0,       // FeedForward Matrix
                0,0,0,0,0,0,
                0,0,0,0,0,0,
                0,0,0,0,0,0,
                0,0,0,0,0,0;
                 
controller.K << 2517.22, 714.52,  4983.86,  223.36, -3235.34, -155.75,  -1858.08, -1042.08,  0.05,  0.04, -0.02,  -0.01,  -0.01,  -0.01,
                2188.17, 623.34,  4299.93,  193.86, -2782.58, -133.03,   2144.9,   1204.13,  0.04,  0.04, -0.01,  -0.01,  -0.01,  -0.01,
                2545.06, 724.6,  -5010.58, -224.47, -3243.92, -155.2,    1834.49,  1029.95, -0.02, -0.01,  0.05,   0.04,  -0.01,  -0.01,
                2148.91, 609.62, -4274.49, -192.65, -2768.47, -133.37,  -2164.07, -1213.94, -0.01, -0.01,  0.04,   0.04,  -0.01,  -0.01,
                2649.5,  751.37, -5.69,    -0.91,    6084.12,  287.61,  -2127.62, -1193.28, -0.01, -0.01, -0.01,  -0.01,   0.05,   0.04,
                2299.89, 654.77,  1.9,      0.85,    5235.98,  248.58,   2462.35,  1382.31, -0.01, -0.01, -0.01,  -0.01,   0.04,   0.04,
                
controller.L << 0.88,-0,-0,0,0,0,
                16.97,-0,-0,0,0,0,
                -0,0.77,0,-0,0,-0,
                -0,21.08,0,-0,0,-0,
                 0,0,0.77,0,-0,-0,
                -0,-0,21.08,0,-0,-0,
                -0,-0,0,0.77,-0,-0,
                -0,-0,-0,21.08,-0,-0,
                 0,0,-0,-0,0,0,
                 0,0,-0,-0,0,-0,
                 0,-0,-0,0,-0,-0,
                 0,-0,-0,-0,-0,0,
                 0,-0,0,-0,-0,0,
                 0,-0,0,-0,0,0;

controller.I << -34.52, -548.77,  336.72, 15.55,
                -29.95, -472.94,  290,   -17.93,
                -34.84,  551.84,  338.11,-15.33,
                -29.47,  470.24,  288.16, 18.1,
                -36.35,  0.3,    -637.88, 17.8,
                -31.49,  0.11,   -548.49,-20.58;          // Integral Gains
                
  controller.initialise();     // intitalise state espace controller
  
  pinMode(2, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(2), read_me, FALLING); // enabling interrupt at pin 2
}
//------------------------------------------------------------------------------------------------------------------
/*
 *                                           MAIN CONTROL LOOP
 */
//------------------------------------------------------------------------------------------------------------------
void loop()
{
  motor.FullStop();  // otherwise set all motor values to 0
  breakout = 0;
  read_rc();             // read receiver values 
  //digitalWrite(4,1);   
  
  if(ch[1]< 1100 && ch[2] > 1800 && ch[3] < 1300 && ch[4] < 1100)
  {
    //digitalWrite(4,0);
    inital.InitSensors();          // intialise IMU and Barometer
    inital.InitMotors();           // intialise motors and calibrate IMU

    while(breakout != 1)
    {
      motor.StartUp();
      read_rc();
      
      if(ch[1] > 1200)
      {
        MainLoop();            // run main flight controll loop        
      }
    }
  }
}
//-------------------------------------------------------------------------------------------------------------
/*
 *                                                 FUNCTIONS
 */
//-------------------------------------------------------------------------------------------------------------
/*
 *   MAIN FLIGHT FUNCTIONALITY                             
 */                                   
void MainLoop()
{
  /*
 * TIMERS
 */
  unsigned long timer = 0;
  unsigned long timeBetFrames = 0;
  unsigned long shutdowntime;
  float timeStep = 0.01;

  /*
   * Reference INPUTS
   */
  double alt;
  double *xA;
  double *yA;
  double *zA;

  double Ainput,Rinput,Pinput;
  double initialAlt;

  double ThrottleSetPoint = 0;
  double AltitudeSetPoint = 0;
  double PitchSetPoint = 0;
  double RollSetPoint = 0;
  double YawSetPoint = 0;;
  
  initialAlt = 0;
  for(int counter = 0; counter < 10; counter++)
  {
    initialAlt += readIn.Altitude();
  }
  initialAlt = initialAlt/10;
  
  while(breakout != 1)
  {
    timer = millis();
  
    read_rc();                      // begin decoding PPM values
    
    xA = sensor.IMU();
    yA = sensor.IMU()+1;       // read in roll, pitch and yaw IMU values
    zA = sensor.IMU()+2;          
    
	  y(0) = sensor.ALT();       // read in current altitude value
    y(1) = *xA -1.5;                          // Read in Systems Outputs
    y(2) = *yA;
    y(3) = *zA;
    y(4) = 2.0529e+03;
    y(5) = 2.0704e+03;
    
    ThrottleSetPoint =  map(ch[1],1040,2020,1000,1800);            // read in throttle setpoint
    
    if(ThrottleSetPoint > 1050)
    {
        Ainput = y(0);
        AltitudeSetPoint = motor.AltitudeControl(ThrottleSetPoint,Ainput,initialAlt); // calcute the altitude setpoint from throttle commands
        PitchSetPoint = map(ch[4],1000,1900,10,-10);
        RollSetPoint = map(ch[3],1000,1900,10,-10);   // read in roll pitch and yaw setpoint values from receiver
                                                      // and map to between 0 and 10 degrees 
        YawSetPoint = map(ch[2],1070,1930,-200,200);       // non feedback rate control for yaw
        
        controller.r << AltitudeSetPoint,PitchSetPoint,RollSetPoint,YawSetPoint;   // set controller references
        
        shutdowntime = 0;                             // keep a running count of time within loop
    }
    else
    {
        AltitudeSetPoint = 0;
        PitchSetPoint = 0;
        RollSetPoint = 0;
        YawSetPoint = 0;
        
        controller.r << AltitudeSetPoint,PitchSetPoint,RollSetPoint,YawSetPoint;  // set controller references
        
        shutdowntime += (millis()- timer)*10;           
        
        if( shutdowntime > 4000)                  // if running count exceeds 4000 counts break out of main loop
        {                                         // and reset all setpoints to zero
          //digitalWrite(12,0);
          //digitalWrite(13,0);
          breakout = 1;
        }
    }

    controller.update(y,timeStep);                        // Update Outputs values
 
    motor.FlightControl(controller.u(0),controller.u(1),controller.u(2),
                        controller.u(3),controller.u(4),controller.u(5));         // send controller output tomotors
    
    timeBetFrames = millis() - timer;
    delay((timeStep*1000) - timeBetFrames);   // run Loop at 100Hz
  }
}                    
/*
 *   READ PPM VALUES FROM PIN 2
 */
  // this code reads value from RC reciever from PPM pin (Pin 2 or 3)
  // this code gives channel values from 0-1000 values 
  //    -: ABHILASH :-    //
void read_me() 
{
  int j;
  
  aa=micros();   // store time value a when pin value falling
  cc=aa-bb;      // calculating time inbetween two peaks
  bb=aa;         
  x[ii]=cc;      // storing 15 value in array
  ii=ii+1;       

  if(ii==15)
  {
    for(j=0;j<15;j++) 
    {
      ch1[j]=x[j];
    }
    ii=0;
  }
}  // copy store all values from temporary array another array after 15 reading 

void read_rc()
{
  int j;
  
  for(int k=14;k>-1;k--)
  {
    if(ch1[k]>10000)
    {
      j=k;
    }
  }  // detecting separation space 10000us in that another array
                    
  for(int i=1;i<=6;i++)
  {
    ch[i]=(ch1[i+j]);
  }
}     // assign 6 channel values after separation space
