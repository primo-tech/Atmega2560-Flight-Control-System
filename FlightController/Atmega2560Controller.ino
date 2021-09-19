//--------------------------------------------------------------------------------------------------------------------
/*
 *                                            CLASS HEADER FILES
 */
//--------------------------------------------------------------------------------------------------------------------
#include "init.h"
#include "motors.h"
#include "sensors.h"
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
Model<12,4,4> QuadModel;
StateSpaceController<12,4,4,true,true> controller(QuadModel);
Matrix<4> y;
Matrix<4> ue = {166.468,166.468,166.468,166.468};
//--------------------------------------------------------------------------------------------------------------------
/*
 *                                         CLASS OBJECT INSTANTIATIONS
 */
//--------------------------------------------------------------------------------------------------------------------
Motors motor;                  // Instantiate motor control class
Initialise inital;             // Instantiate initialisation class
Sensors sensor;                // Instantiate Sensor class
//--------------------------------------------------------------------------------------------------------------
/*
 *                                   COMPONENT INITIALISATION LOOP 
 */
//--------------------------------------------------------------------------------------------------------------
void setup() 
{
  Serial.begin(9600);
  Wire.begin();                // join i2c bus with address #1
  
  QuadModel.A << 1,0.01,0,0,0,0,0,0,-3.555801184840882e-08,-3.555801184840882e-08,-3.555801184840882e-08,-3.555801184840882e-08,
                 0,1,0,0,0,0,0,0,-6.62492896954863e-06,-6.62492896954863e-06,-6.62492896954863e-06,-6.62492896954863e-06,
                 0,0,1,0.01,0,0,0,0,4.692184455213431e-07,4.692184455213431e-07,-4.692184455213431e-07,-4.692184455213431e-07,
                 0,0,0,1,0,0,0,0,8.742161642876062e-05,8.742161642876062e-05,-8.742161642876062e-05,-8.742161642876062e-05,
                 0,0,0,0,1,0.01,0,0,4.692184455213431e-07,-4.692184455213431e-07,4.692184455213431e-07,-4.692184455213431e-07,
                 0,0,0,0,0,1,0,0,8.742161642876062e-05,-8.742161642876062e-05,8.742161642876062e-05,-8.742161642876062e-05,
                 0,0,0,0,0,0,1,0.01,1.290619895253161e-08,-1.290619895253161e-08,-1.290619895253161e-08,1.290619895253161e-08,
                 0,0,0,0,0,0,0,1,2.404595951311913e-06,-2.404595951311913e-06,-2.404595951311913e-06,2.404595951311913e-06,
                 0,0,0,0,0,0,0,0,0.6426210983825759,0,0,0,
                 0,0,0,0,0,0,0,0,0,0.6426210983825759,0,0,
                 0,0,0,0,0,0,0,0,0,0,0.6426210983825759,0,
                 0,0,0,0,0,0,0,0,0,0,0,0.6426210983825759;

  QuadModel.B << -6.328350532572434e-08,-6.328350532572434e-08,-6.328350532572434e-08,-6.328350532572434e-08,
                 -1.833015510785475e-05,-1.833015510785475e-05,-1.833015510785475e-05,-1.833015510785475e-05,
                 8.350800973538338e-07,8.350800973538338e-07,-8.350800973538338e-07,-8.350800973538338e-07,
                 0.0002418821086662523,0.0002418821086662523,-0.0002418821086662523,-0.0002418821086662523,
                 8.350800973538338e-07,-8.350800973538338e-07,8.350800973538338e-07,-8.350800973538338e-07,
                 0.0002418821086662523,-0.0002418821086662523,0.0002418821086662523,-0.0002418821086662523,
                 2.296949316596679e-08,-2.296949316596679e-08,-2.296949316596679e-08,2.296949316596679e-08,
                 6.653145560030046e-06,-6.653145560030046e-06,-6.653145560030046e-06,6.653145560030046e-06,
                 4.166187783441478,0,0,0,
                 0,4.166187783441478,0,0,
                 0,0,4.166187783441478,0,
                 0,0,0,4.166187783441478;
                                     
  QuadModel.C << 1,0,0,0,0,0,0,0,0,0,0,0,
                 0,0,1,0,0,0,0,0,0,0,0,0,
                 0,0,0,0,1,0,0,0,0,0,0,0,     // Sensor Matrix
                 0,0,0,0,0,0,1,0,0,0,0,0;

  QuadModel.D << 0,0,0,0,
                 0,0,0,0,       // FeedForward Matrix
                 0,0,0,0,
                 0,0,0,0;
                 
  controller.K << -4644.133079713855,-576.3630899492625,2397.71010855017,133.3184259799572,2397.710157235878,133.3184298323706,4018.651205726411,840.6199069371121,0.05674317012341944,0.003781539207099473,0.00378153955917026,-0.02895279985970016,
                  -4644.13362832505,-576.3631842456095,2397.710174890897,133.3184290755883,-2397.709525397865,-133.3184032152591,-4018.651426579603,-840.6199716039347,0.003781540847118074,0.05674317456093293,-0.02895279483529509,0.003781541698908982,
                  -4644.133855170044,-576.3632145038208,-2397.709539680398,-133.3184052484524,2397.710172826298,133.3184301987022,-4018.651437532371,-840.6199731389341,0.003781540895831137,-0.02895279487728855,0.05674317417412463,0.003781541513972165,
                  -4644.134759045391,-576.3633012711695,-2397.709529198793,-133.3184050759467,-2397.709464113178,-133.3184003141364,4018.651213389123,840.6199082800731,-0.02895279675356426,0.003781543179261148,0.003781543297596708,0.05674317386740481;
                                
  controller.L << 0.9905002378172342,-2.5644184757344e-16,-1.145970952393551e-16,-2.439600375655657e-17,
                  3.082170426840124,-2.624631197920426e-14,-4.317449972414643e-14,6.256663049036548e-15,
                  -2.56441847573409e-16,0.9990051497224506,1.058863305954064e-19,-6.740047753633948e-18,
                  8.026446563333578e-13,0.315787558055033,6.585389773517792e-15,4.74404060040635e-14,
                  -1.145970952393304e-16,1.058863305953926e-19,0.9990051497224508,-7.137404547489076e-18,
                  7.823378928147604e-13,5.299714542203728e-14,0.3157875580547714,-5.765945622734376e-14,
                  -2.439600375655826e-17,-6.74004775363486e-18,-7.137404547491564e-18,0.9990051459896653,
                  -4.699301901434998e-14,1.193337159103447e-14,-9.509312570559311e-15,0.3154133480930926,
                  -0.001532994372114739,0.002211806177125162,0.002211806177191284,6.083799492874253e-05,
                  -0.001532994372349487,0.00221180617724801,-0.002211806177230313,-6.083799500082945e-05,
                  -0.001532994372236872,-0.00221180617704342,0.002211806177271831,-6.083799505369908e-05,
                  -0.001532994372109905,-0.002211806177122893,-0.002211806177068465,6.083799495148398e-05;

  controller.I << 101.6177093442602,-174.3103048021012,-174.3103075092017,-29.97417067092887,
                  101.6177197609227,-174.3103099388304,174.3102603128503,29.97417219339999,
                  101.6177245762841,174.3102608914629,-174.3103093260622,29.97417227576808,
                  101.6177454969845,174.310259601554,174.3102556979963,-29.97417072520372;          // Integral Gains
                
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
  read_rc();         // read receiver values  
  
  if(ch[1]< 1100 && ch[2] > 1800 && ch[3] < 1300 && ch[4] < 1100)
  {
    inital.initSensors();      // intialise IMU and Barometer
    inital.initMotors();       // intialise motors and calibrate IMU

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

  double ThrottleSetPoint = 0;
  double AltitudeSetPoint = 0;
  double PitchSetPoint = 0;
  double RollSetPoint = 0;
  double YawSetPoint = 0;;
  
  double initialAlt = 0;
  for(int counter = 0; counter < 10; counter++)
  {
    initialAlt += sensor.ALT();
  }
  initialAlt = initialAlt/10;
  
  while(breakout != 1)
  {
    timer = millis();
  
    read_rc();                 // begin decoding PPM values
    
	  y(0) = sensor.ALT() - initialAlt;     // read in current altitude value
    y(1) = *(sensor.IMU());               // read in Systems Outputs
    y(2) = *(sensor.IMU()+1);             // read in roll, pitch and yaw IMU values
    y(3) = *(sensor.IMU()+2); 
    
    ThrottleSetPoint =  map(ch[1],1040,2020,1000,1800);            // read in throttle setpoint
    
    if(ThrottleSetPoint > 1050)
    {
        double Ainput = y(0);
        AltitudeSetPoint = motor.AltitudeControl(ThrottleSetPoint,Ainput,initialAlt); // calcute the altitude setpoint from throttle commands
        PitchSetPoint = map(ch[4],1000,1900,10,-10);
        RollSetPoint = map(ch[3],1000,1900,10,-10);   // read in roll pitch and yaw setpoint values from receiver
                                                      // and map to between 0 and 10 degrees 
        YawSetPoint += map(ch[2],1070,1930,-5,5);     // feedback ramp input for yaw
        
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
        
        if( shutdowntime > 4000)                 // if running count exceeds 4000 counts break out of main loop
        {                                        // and reset all setpoints to zero
          breakout = 1;
        }
    }

    controller.update(y,timeStep);               // Update Outputs values
 
    motor.FlightControl(controller.u(0) + ue(0),controller.u(1) + ue(1),controller.u(2) + ue(2),controller.u(3) + ue(3));  // send controller output tomotors
    
    timeBetFrames = millis() - timer;
    delay((timeStep*1000) - timeBetFrames);      // run Loop at 100Hz
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
  
  aa = micros();   // store time value a when pin value falling
  cc = aa-bb;      // calculating time inbetween two peaks
  bb = aa;         
  x[ii] = cc;      // storing 15 value in array
  ii = ii + 1;       

  if(ii == 15)
  {
    for(j=0;j<15;j++) 
    {
      ch1[j] = x[j];
    }
    ii = 0;
  }
}  // copy store all values from temporary array another array after 15 reading 

void read_rc()
{
  int j;
  
  for(int k = 14;k > -1;k--)
  {
    if(ch1[k] > 10000)
    {
      j = k;
    }
  }  // detecting separation space 10000us in that another array
                    
  for(int i = 1;i <= 6;i++)
  {
    ch[i] = (ch1[i+j]);
  }
}     // assign 6 channel values after separation space
