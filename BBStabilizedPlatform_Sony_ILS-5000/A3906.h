/*
  Copyright Bluebird Aero Systems Engineering 2011

  $Id: A3906.h,v 1.13 2011/07/20 06:49:47 EranS Exp $
  $Log: A3906.h,v $
  Revision 1.13  2011/07/20 06:49:47  EranS
  changed ITG rate to 100
  encoder init ar higher PWM
  telemetry changed

  Revision 1.12  2011/06/18 09:13:50  EranS
  before france changes

  Revision 1.11  2011/05/03 06:46:07  EranS
  After Eran rewire - pay attention to encoder pins

  Revision 1.10  2011/04/17 08:44:11  EranS
  add proper reset of encoders via GPIO

  Revision 1.9  2011/04/06 11:59:48  EranS
  unified stabilize code for pitch/roll

  Revision 1.8  2011/04/06 06:44:20  EranS
  cleaning up, add some constants, move gyro_at_rest code

  Revision 1.7  2011/04/06 05:44:34  EranS
  add stabilizing code

  Revision 1.6  2011/03/30 06:58:34  EranS
  added more telemetry
  simplified and improved motor driver

  Revision 1.5  2011/03/24 12:31:38  EranS
  Added more commands for telemetry
  added power parameter to motor commands
  calibrated parameters for motor commands

  Revision 1.4  2011/03/23 12:15:32  EranS
  PWM test works, added Stow and Pilot functions

  Revision 1.3  2011/03/22 15:42:49  EranS
  added telemetry
  fixed adc code

  Revision 1.2  2011/03/21 13:42:03  EranS
  fix adc code, add telemtry

  Revision 1.1  2011/03/15 12:23:45  EranS
  First save

  
  A3906SESTR Motor Controller
*/


// See datasheet page 5 for these values

#define PWMRATE 50000           // 50KHz

// useful enums
// DO NOT change the order of MOTOR as all code depends on it
enum MOTOR {PITCH_MOTOR=0, ROLL_MOTOR, NEITHER_MOTOR};
//enum A3906Logic {A3906_DISABLE, A3906_FORWARD, A3906_REVERSE ,A3906_BRAKE};
enum A3906Logic {A3906_DISABLE, A3906_FORWARD, A3906_REVERSE ,A3906_BRAKE};

// function prototypes
void A3906Init(void);
//void GeneralPitchRoll(enum MOTOR, enum A3906Logic,unsigned);
void GeneralPitchRoll(enum MOTOR motor, enum A3906Logic logic,unsigned long dutycycle);
tBoolean GimbalsBoundsCheck(enum MOTOR motor);
void SetValueEncoder(enum MOTOR motor,int newencoder,unsigned dutycycle);
void MoveToEncoder(enum MOTOR motor);
// prototype interrupt handler
void A3906Action(int motor);
void PWMUpdateDutyCycle(void);
void PWMOutputOff(void);


#ifdef A3906_MAIN
unsigned upperlimits[2] = { 280, 950};  // pitch,roll
unsigned lowerlimits[2] = { 10,25};
int desiredencoder[2];
unsigned long dcycle[2];       // current duty cycle
unsigned mdirection[2];   // current direction
tBoolean brakeOnChange; // policy on change direction
#else
extern unsigned upperlimits[];
extern unsigned lowerlimits[];
extern int desiredencoder[];
extern unsigned dcycle[];
extern unsigned mdirection[];
extern tBoolean brakeOnChange; // policy on change direction
#endif

// minimum dutycycle to move motor
#define PITCH_MINIMUM_DUTYCYCLE 10
#define ROLL_MINIMUM_DUTYCYCLE 10
