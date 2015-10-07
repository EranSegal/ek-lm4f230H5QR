/*
   Copyright Bluebird Aero Systems Engineering 2012

  $Id: bluebird.h,v 1.14 2011/07/14 07:19:48 EranS Exp $
  $Log: bluebird.h,v $
  Revision 1.14  2011/07/14 07:19:48  EranS
  added script engine

  Revision 1.13  2011/05/03 06:46:07  EranS
  After Eran rewire - pay attention to encoder pins

  Revision 1.12  2011/04/17 08:44:11  EranS
  add proper reset of encoders via GPIO

  Revision 1.11  2011/04/12 15:03:35  EranS
  added joystick interface

  Revision 1.10  2011/04/10 10:37:29  EranS
  Add filer3.c and AHRS from Lumus project

  Revision 1.9  2011/04/06 13:09:03  EranS
  modified BB application to work with stabilizer

  Revision 1.8  2011/04/06 11:59:48  EranS
  unified stabilize code for pitch/roll

  Revision 1.7  2011/04/06 06:44:20  EranS
  cleaning up, add some constants, move gyro_at_rest code

  Revision 1.6  2011/04/06 05:44:34  EranS
  add stabilizing code

  Revision 1.5  2011/03/30 06:58:34  EranS
  added more telemetry
  simplified and improved motor driver

  Revision 1.4  2011/03/24 12:31:38  EranS
  Added more commands for telemetry
  added power parameter to motor commands
  calibrated parameters for motor commands

  Revision 1.3  2011/03/23 12:15:32  EranS
  PWM test works, added Stow and Pilot functions

  Revision 1.2  2011/03/15 15:00:45  EranS
  changed ADC functions

  Revision 1.1  2011/03/15 12:23:45  EranS
  First save

  Layout on Bluebird platform board
*/

/*
  include directory paths 

$PROJ_DIR$\..\..\..
$PROJ_DIR$\..
$PROJ_DIR$\.

  Set JTAG connection to SWD

  Link with correct *.icf file, dont use default

*/
#define RESET_ACCELEROMETER


#define USE_ITG3200_INTERRUPT
#define USE_MAG3110_INTERRUPT

#define LOBYTE(hw) ((BYTE)(hw))
#define HIBYTE(hw) ((BYTE)((HWORD)(hw)>>8))

#define LOHWORD(w) ((HWORD)(WORD)(w))
#define HIHWORD(w) ((HWORD)((WORD)(w)>>16))

#define LOWORD(w) ((WORD)(w))
#define HIWORD(w) (((WORD)(w)>>32))

#define MAKEHWORD(high, low) (((HWORD)((BYTE)(high))<<8) | (HWORD)((BYTE)(low)))
#define MAKEWORD(high, low) (((WORD)((HWORD)(high))<<16) | (WORD)((HWORD)(low)))
  

// BCP API Prototypes
void InitParams(void);
int InitUART(unsigned long Base, unsigned long UARTClk, unsigned long baud, unsigned long config);
int MessageLoop(unsigned long ulBase,unsigned char cid);
int SetARC(int fine, int coarse,int time);
int ToggleWhiteHot(void);
int FreezeVideo(void);
int ReservedFunction(void);
void Show_Label(void);
void ReportData();


// observations of encoder readings for pitch
#define PITCH_ENC_0_DEG 245.0
#define PITCH_ENC_90_DEG 750.0

//#define MOTOR_WIRING_OPPOSITE   // remove this when both motors wired in the same sense

#define PI 3.14159265f
#define RAD2DEG (180.0f/PI) // 57.295779
#define DEG2RAD (PI/180.0f)

/*
    Define how the encoders respond to motor action, 
    Comment out the incorrect one.
*/
//#define FORWARD_INCREASES_ENCODER
#define FORWARD_DECREASES_ENCODER
/*
  Define how to reset gimbals at startup
*/
//#define GIMBAL_RESET A3906_FORWARD
#define GIMBAL_RESET A3906_FORWARD
