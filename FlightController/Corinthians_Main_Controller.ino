//--------------------------------------------------------------------------------------------------------------------
/*
 *                                            CLASS HEADER FILES
 */
//--------------------------------------------------------------------------------------------------------------------
#include "init.h"
#include "motors.h"
#include "sensors.h"
#include <StateSpaceControl.h>
#include "modelParameters.h"

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
Model<8,6,4> HexaModel;
StateSpaceController<8,6,4,true,true> controller(HexaModel);
Matrix<4> y;
//--------------------------------------------------------------------------------------------------------------------
/*
 *                                         CLASS OBJECT INSTANTIATIONS
 */
//--------------------------------------------------------------------------------------------------------------------
Motors motor;                   // Instantiate motor control class
Initialise inital;              // Instantiate initialisation class
Sensor sensor;                  // Instantiate Sensor class
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
  
  HexaModel.A << ;
  HexaModel.B << ;
                                     
  HexaModel.C << 1,0,0,0,0,0,0,0,
                 0,0,1,0,0,0,0,0,
                 0,0,0,0,1,0,0,0,     // Sensor Matrix
                 0,0,0,0,0,0,1,0;

  HexaModel.D << 0,0,0,0,0,0,
                 0,0,0,0,0,0,       // FeedForward Matrix
                 0,0,0,0,0,0,
                 0,0,0,0,0,0;
                 
  controller.K << ;
  controller.L << ;
  controller.I << ;         // Integral Gains
                
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

  if(ch[1]< 1100 && ch[2] > 1800 && ch[3] < 1300 && ch[4] < 1100)
  {
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
    initialAlt += sensor.ALT();
  }
  initialAlt = initialAlt/10;
  
  while(breakout != 1)
  {
    timer = millis();
  
    read_rc();                 // begin decoding PPM values
    
    xA = sensor.IMU();
    yA = sensor.IMU()+1;       // read in roll, pitch and yaw IMU values
    zA = sensor.IMU()+2;          
    
	  y(0) = sensor.ALT();       // read in current altitude value
    y(1) = *xA - 1.5;          // Read in Systems Outputs
    y(2) = *yA;
    y(3) = *zA;
    
    ThrottleSetPoint =  map(ch[1],1040,2020,1000,1800);            // read in throttle setpoint
    
    if(ThrottleSetPoint > 1050)
    {
        Ainput = y(0);
        AltitudeSetPoint = motor.ALTControl(ThrottleSetPoint,Ainput,initialAlt); // calcute the altitude setpoint from throttle commands
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
