/*
  Copyright Bluebird Aero Systems Engineering 2012

  $Id: PWMtest.c,v 1.20 2011/08/03 07:44:42 EranS Exp $
  $Log: PWMtest.c,v $
  Revision 1.20  2011/08/03 07:44:42  EranS
  Eugene changes

  Revision 1.19  2011/07/21 06:31:25  EranS
  Improved script engine to allow zero elapsed time scripts

  Revision 1.18  2011/07/20 06:49:47  EranS
  changed ITG rate to 100
  encoder init ar higher PWM
  telemetry changed

  Revision 1.17  2011/07/14 07:19:48  EranS
  added script engine

  Revision 1.16  2011/06/18 09:13:50  EranS
  before france changes

  Revision 1.15  2011/05/11 07:39:16  EranS
  Integrate with BB library, stow/pilot buttons now work

  Revision 1.14  2011/05/03 12:29:26  EranS
  add commands and telemetry for first flight

  Revision 1.13  2011/05/03 06:46:07  EranS
  After Eran rewire - pay attention to encoder pins

  Revision 1.12  2011/04/17 08:44:11  EranS
  add proper reset of encoders via GPIO

  Revision 1.11  2011/04/12 15:03:35  EranS
  added joystick interface

  Revision 1.10  2011/04/12 05:50:35  EranS
  calibrated gimbal / gyro / accelerometer

  Revision 1.9  2011/04/10 10:37:29  EranS
  Add filer3.c and AHRS from Lumus project

  Revision 1.8  2011/04/06 11:59:48  EranS
  unified stabilize code for pitch/roll

  Revision 1.7  2011/04/06 07:20:51  EranS
  Stabilize moved to separate file

  Revision 1.6  2011/04/06 06:44:20  EranS
  cleaning up, add some constants, move gyro_at_rest code

  Revision 1.5  2011/04/06 05:44:34  EranS
  add stabilizing code

  Revision 1.4  2011/03/30 06:58:34  EranS
  added more telemetry
  simplified and improved motor driver

  Revision 1.3  2011/03/24 12:31:38  EranS
  Added more commands for telemetry
  added power parameter to motor commands
  calibrated parameters for motor commands

  Revision 1.2  2011/03/23 12:15:32  EranS
  PWM test works, added Stow and Pilot functions

  Revision 1.1  2011/03/22 15:42:49  EranS
  added telemetry
  fixed adc code

  Revision 1.1  2011/03/21 13:42:03  EranS
  fix adc code, add telemtry

  Bluebird Stabilized Platform
*/

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>
#include <math.h>
#include "bluebird.h"
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/lm4f230h5qr.h"
#include "driverlib/debug.h"
#include "driverlib/sysctl.h"
#include "driverlib/interrupt.h"
#include "driverlib/uart.h"
#include "driverlib/timer.h"
#include "driverlib/pwm.h"
#include "driverlib/rom.h"
#include "driverlib/qei.h"
#include "driverlib/gpio.h"
#include "MasterI2C.h"
#include "SysTick.h"
#include "utilities.h"
#include "A3906.h"
#include "rmb20d01.h"
#include "bma180.h"
#include "ITG3200.h"
#include "utilities.h"
#include "Stabilize.h"
#include "filter3.h"
#include "gpiohandler.h"
#include "uart_echo.h"
#include "encoder.h"
#include "reg.h"
#include "led.h"
#include "pins.h"

#define WAIT_FOR_NEXT_STATE  1000// 5000
#define WAIT_FOR_NEXT_STATE1 1000// 10000
#define WAIT_FOR_MOTOR_TEST 1000// 30000

// for collecting S/Pnnn command
static char inbuff[10];
unsigned inbufptr;
tBoolean datain = false;
tBoolean dataready = false;
tBoolean traceflag = true;
//tBoolean startup = true;
tBoolean fullsweepswitch = false;
tBoolean oscillateswitch = false;
tBoolean sweeptolow;
tBoolean oldtrace;
tBoolean msgdone = false;
char *message1 = "Push both reset buttons, then any key\r\n";
unsigned long starttest;
unsigned long lasttime,lastitg;
unsigned localdcycle[2] = {PITCH_MINIMUM_DUTYCYCLE,ROLL_MINIMUM_DUTYCYCLE};
//tBoolean continueflag;
enum MOTOR oscillatemotor = NEITHER_MOTOR;
int oscillatecount = 0;
// right, down increases encoder; left, up decreases encoder;
enum eDirection {ROLLRIGHT,PITCHDOWN=ROLLRIGHT,ROLLLEFT,PITCHUP=ROLLLEFT };
enum eDirection motordirection = ROLLRIGHT;
enum A3906Logic dir;
char charin = 0;
#define PITCH_AXIS 'z'
#define ROLL_AXIS 'x'
int XINDEX,AXISINDEX;
unsigned long g_ulFlags;

///////////////////////////////////////
extern float pof;
extern float rof;
///////////////////////////////////////
//extern CircularBuffer* que;
//extern float stddev,average,sum,sumOfSquares;

extern float k_a_roll,k_g_roll,k_i_roll,k_7_roll;
extern float k_a_pitch,k_g_pitch,k_i_pitch,k_7_pitch;

extern float y_rate_roll,y_pos_roll,ref_pos_roll,e_pos_roll,e_rate_roll,ie_rate_roll,u_rate_roll;
extern float y_rate_pitch,y_pos_pitch,ref_pos_pitch,e_pos_pitch,e_rate_pitch,ie_rate_pitch,u_rate_pitch;

extern float Ax,Ay,Az;
extern float Gx,Gy,Gz;

extern int uroll_backlash_right;
extern int uroll_backlash_left;

extern float out_rate_roll;
extern float out_rate_pitch;
extern float Phic,Thetac,Phie,Thetae;
extern int pitchminv,rollminv;
extern float idegsec[];
extern unsigned long g_ulAverage;

#define MAX_SCENARIOS 16
int scenarioindex = 0;

tBoolean testpitch = false;
tBoolean direction_morot = false;

// dummy joystick data
extern float omega_vertical,omega_horizontal;  
unsigned tmlines;
#pragma pack(1)
struct telemblock {
  WORD header;
  BYTE gyro[6]; // 3 short
  BYTE accel[6]; // 3 short
  WORD encoder[2]; // 2 short
  float us[2]; // 2 float
  float nav[2]; // 2 float
} TelemBlock;
#pragma pack()
unsigned char pbuff[100];

#ifdef CURINT_ADC
unsigned long dutycycle = 450;
float g_fV = 0.0;
float g_fC = 0.0;
float g_fN = 0.5f;
float g_fC1 = 1.5614f;
float g_fC2 = 0.0f;
float g_fDPWM = 1;
float g_fRpmMotor = 0.0;
#endif

char stoff[7] = {0xAA, 0x32, 0xF0, 0x01, 0x00, 0x00, 0x33};
tBoolean stabilizeaxis[2];  // separate stabilise switch for each axis
extern int brakelimit;
unsigned int oscillaterate;
extern float g_fmindutycycle[2][4]; //PITCH_MOTOR=0
extern tBoolean scriptactive;
int eugenecounter = 0;

// script interface
void ScriptStart();
void ScriptStop();
void ScriptExecute();



//*****************************************************************************
//
// The error routine that is called if the driver library encounters an error.
//
//*****************************************************************************
void
__error__(char *pcFilename, unsigned long ulLine)
{
  sprintf(pbuff,"Driver %s %u \r\n",pcFilename,ulLine);
  UART0Send(pbuff, strlen(pbuff));
}

float roll_curent = 0,pitch_curent = 0;



void PrintTelemetry(unsigned index)
{
  int type = testpitch ? PITCH_MOTOR : ROLL_MOTOR; //  PITCH_MOTOR == 0  ROLL_MOTOR == 1  
  
  eugenecounter=1;
  //if (eugenecounter == 2) eugenecounter =1;
  
  if (traceflag && eugenecounter)
  {
    switch (scenarioindex)
    {

	  

		case 0:


		  //sprintf(pbuff,"%lu,%d,%u,%.4f,%.2f\r\n",
		  //g_ulTickCount,mdirection[1],dcycle[1],u_rate_roll,Motor[ROLL_MOTOR].deg2sec);
		  //UART0Send(pbuff, strlen(pbuff)); 

		  sprintf(pbuff,"%lu,%d,%u,%.2f\r\n",
		  g_ulTickCount,mdirection[1],dcycle[1],Motor[ROLL_MOTOR].deg2sec);
		  UART0Send(pbuff, strlen(pbuff)); 
		  
		  break;


	case 1:
	

		UART0Send(testpitch ? "P," : "R," ,2);		
		//UART0Send("G-,",3);		
		sprintf(pbuff,"%lu,%hd,%hd,%hd,%u\r\n",g_ulTickCount,
		ITG3200values[0],ITG3200values[1],ITG3200values[2],PitchRollAmp[type]);
		UART0Send(pbuff, strlen(pbuff)); 
		
		//UART0Send("Ac,",3);				
		//sprintf(pbuff,"%hd,%hd,%hd\r\n",
		//BMA180values[0],BMA180values[1],BMA180values[2]);
		//UART0Send(pbuff, strlen(pbuff)); 

	  break;	  


    }
  }
}



void HardwareInit()
{
  IntMasterDisable();
  
  // Set the system clock to run at 50MHz from the PLL. // PLL=400MHz
  // sysc2000000lk = 400MHz/2/4 = 50MHz
  //SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);

  ROM_SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);

  // Set the system clock to run at 20MHz from the PLL. // PLL=100MHz
  // sysc2000000lk = 100MHz/2/4 = 20MHz
  //SysCtlClockSet(SYSCTL_SYSDIV_10| SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);
   
  if (InitUART(UART0_BASE,SysCtlClockGet(),19200,
      	(UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE)) == -1)
      	  while (1);    // hang

  if (InitUART(UART1_BASE,SysCtlClockGet(),9600,
      	(UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE)) == -1)
      	  while (1);    // hang
      
  SysTickInit(1000);    // 1msec counter
  //TimerInit(1000);    	 	// 100ms counter
  
  BBLedInit();  
  DEBUG_LED1(1);	 
  DEBUG_LED2(1);	 
  DEBUG_LED2(0);	 

  MasterI2C0Init(SysCtlClockGet() >= 40000000L);  // Need 40MHz for 400K
  A3906Init(); 
  RMBD01Init();
  Encoder_Init(QEI0_BASE);   // J6 PITCH Encoder 
  Encoder_Init(QEI1_BASE);   // J8 ROLL Encoder
  
  EncoderLinesSet(QEI0_BASE,64);
  EncoderLinesSet(QEI1_BASE,64);

  //cpReset(0);
  //cpReset(1);
  //init_ctrl();
      
  IntMasterEnable();
}


//*****************************************************************************
//
// A set of flags used to track the state of the application.
//
//*****************************************************************************
#define FLAG_CLOCK_TICK         0           // A timer interrupt has occurred
#define FLAG_CLOCK_COUNT_LOW    1           // The low bit of the clock count
#define FLAG_CLOCK_COUNT_HIGH   2           // The high bit of the clock count
#define FLAG_UPDATE             3           // The display should be updated
#define FLAG_BUTTON             4           // Debounced state of the button
#define FLAG_DEBOUNCE_LOW       5           // Low bit of the debounce clock
#define FLAG_DEBOUNCE_HIGH      6           // High bit of the debounce clock
#define FLAG_BUTTON_PRESS       7           // The button was just pressed

void SoftwareInit()
{

  //
  // initialization BlueBird parameters for communication protocol.
  //
  //InitParams();
  ITG3200Init();  
  #ifdef USE_BMA180_INTERRUPT
  BMA180Init();  
  #endif
  //MAG3110Init();

  //
  // Set the priorities of the interrupts.	
  // The following interrupts are used
  // (listed in priority from highest to lowest):
  //
  //	 QEI-ROLL	 - The input from the quadrature encoder.
  //
  //	 QEI-PITCH	 - The input from the quadrature encoder.
  //  
  //	 PWM0		 - The periodic control loop for the application.
  //
  //	 PWM1		 - The periodic control loop for the application.
  //  
  //	 UART0		 - The UART <-> BBGCS.  This must be the same
  //				   priority as the PAYLOAD interrupt in order to provide the
  //				   required mutual exclusion between the two.
  //
  //	 UART1		 - The UART <-> PAYLOAD.  This must be the same
  //				   priority as the BBGCS interrupt in order to provide the
  //				   required mutual exclusion between the two.
  //
  //
  // Set the priorities of the interrupts used by the application.
  //
  //
  // Enable the peripherals used by this example.
  //
  IntPrioritySet(INT_GPIOA, 0x00);
  IntPrioritySet(INT_GPIOE, 0x02);  
  IntPrioritySet(QEI_PITCH_PHA_INT, 0x40);
  IntPrioritySet(QEI_ROLL_PHA_INT, 0x40);
  IntPrioritySet(INT_PWM0, 0x60);
  IntPrioritySet(INT_PWM1, 0x60);
  IntPrioritySet(INT_UART0, 0x80);  
  IntPrioritySet(INT_UART1, 0x80);
  
  //
  // PITCH_MOTOR is PORTB & J6- PITCH Encoder 
  //
  // moves gimbal to extreme right and down
  MotorBrakePositionClean(PITCH_MOTOR);
  //EncoderInitReset(QEI0_BASE,PITCH_MOTOR,A3906_REVERSE);  // J6 PITCH Encoder  
  //EncoderInitReset(QEI0_BASE,PITCH_MOTOR,A3906_FORWARD);  // J6 PITCH Encoder  
 
  //
  // ROLL_MOTOR is PORTA & J8- ROLL Encoder
  //
  // moves gimbal to extreme right and down    
  MotorBrakePositionClean(ROLL_MOTOR);
  //EncoderInitReset(QEI1_BASE,ROLL_MOTOR,A3906_FORWARD); // J8 ROLL Encoder  
  //EncoderInitReset(QEI1_BASE,ROLL_MOTOR,A3906_REVERSE); // J8 ROLL Encoder
  //GeneralPitchRoll(ROLL_MOTOR,A3906_REVERSE,1000);
  #ifdef USE_BMA180_INTERRUPT
  AHRS_Init();
  #endif  
  // move gimbal to centre, half down - happens when stabilize flag set
  omega_vertical = 0.0;
  omega_horizontal = 0.0;
  #ifdef CURINT_ADC
  g_fC1 = g_fC1 * g_fN;
  #endif
  StabilizeInit();
  tmlines = 0;
  stabilizeaxis[PITCH_MOTOR] = true;
  stabilizeaxis[ROLL_MOTOR] = true;
  oscillaterate = 1;

}


int main(void) 
{
  unsigned long last_telemtry;
  //
  // The FPU should be enabled because some compilers will use floating-
  // point registers, even for non-floating-point code.  If the FPU is not
  // enabled this will cause a fault.  This also ensures that floating-
  // point operations could be added to this application and would work
  // correctly and use the hardware floating-point unit.  Finally, lazy
  // stacking is enabled for interrupt handlers.  This allows floating-
  // point instructions to be used within interrupt handlers, but at the
  // expense of extra stack usage.
  //
  FPUEnable();
  FPULazyStackingEnable();

  HardwareInit();
  SoftwareInit();
  last_telemtry = g_ulTickCount;
  //GeneralPitchRoll(ROLL_MOTOR,A3906_BRAKE,dutycycle);
  
  while(1)
  {
	    // Execute main loop to interpretation protocol command	  
	    MainModeLoop(); 
  
	    EncoderTick(QEI0_BASE); // PITCH Encoder		
	    EncoderTick(QEI1_BASE); // ROLL Encoder

		//itgready = false;
		if (itgready == true)
		{
		/*------------------------------------------------------------*/
		/* ITG3200getXYZ() time is 30~ microsecond						   */
		/*------------------------------------------------------------*/	
		#if 1
		ITG3200getXYZ();
		#ifdef USE_BMA180_INTERRUPT
		/*------------------------------------------------------------*/
		/* BMA180GetXYZ() time is 30~ microsecond						   */
		/*------------------------------------------------------------*/				
		BMA180GetXYZ(); 				
		/*------------------------------------------------------------*/
		/* AHRS() time is 25~ microsecond										 */
		/*------------------------------------------------------------*/					
		AHRS();
		#endif
		/*------------------------------------------------------------*/
		/* Stabilize() time is 4~ microsecond								  */
		/*------------------------------------------------------------*/						
		if (stabilizeaxis[ROLL_MOTOR])		
			Stabilize(ROLL_MOTOR);
		/*------------------------------------------------------------*/
		/* Stabilize() time is 4~ microsecond								  */
			
		/*------------------------------------------------------------*/									
   	    if (stabilizeaxis[PITCH_MOTOR])		
			Stabilize(PITCH_MOTOR);
		#endif
		
		// restrict telemtry to about 10Hz, 
		//if ((g_ulTickCount - last_telemtry) > 1)
		//{	 
			//last_telemtry = g_ulTickCount;
			//PrintTelemetry(tmlines%4);	// write at 200/4 = 50 Hz
		//}
		
		
		}
		/*------------------------------------------------------------*/
		/* Total time is 115~ microsecond test by EranS 						*/
		/* Gyro and Encoder dose not test yet. Need testing with new board			*/
		/*------------------------------------------------------------*/		
		#ifdef CURINT_ADC
		if(new_adc == true)
		{
    		new_adc=false;
			g_fC = g_ulAverage/2.0f;
					
			g_fV = (2.0f*dutycycle)/1000.0f;					
									
			g_fC2 = g_fV-(0.0115f*g_fC);
						
			if((g_fC2/g_fC1) < 0.5f)
				g_fDPWM = 1.0f;
			else
				g_fDPWM= 0.0f;

			g_fV = g_fC1+(0.0115f*g_fC);
			dutycycle = ((g_fV/2.0f)*1000)+g_fDPWM;
					
			GeneralPitchRoll(ROLL_MOTOR,A3906_REVERSE,dutycycle);
								
		}			
		#endif		
  	}
  }



  
// dummy for UART0 interrupt handler
/*
            'a' both motors to 'disable'
            'b' test pitch/roll 
            'c' cancel oscillate/sweep
            'd/D'  change KROLLd
            'e' start/stop stabilize
            'f'/'F' Full sweep pitch/roll
            'g'/'G' KPITCHPOS
            'h'/'H'  stow/pilot
            'i/I'  change KROLLi
            jhhh,vvv  joystick horizontal/vertical
                hhh   vvv
              +-----+-----+
          +ve |anti | down|
              +-----+-----+
          -ve |clock| up  |
              +-----+-----+
            'k'/'K'  stop/start script
            'l','L' decrement/increment symmetrydelta
            'm' Send string to UART1 (camera)
            'n'/'N' decrement/increment min voltage to motor
            'o'/'O' Oscillate pitch/roll 5Hz
            'p/P'  change KROLLp
            'q'/'Q' brakelimit
            'r'/'R' decrement/increment oscillaterate
            's/S'  change UROLLSAT
            't' trace on/off
            'u' set omega
            'v'/'V'  decrement/increment pitch/roll dutycycle
            'w/W'  change KROLLs
            'x'/'X' decrement/increment base voltage
            'y' toggle brakeOnChange
            'Y' increment scenario
            znnn  set pitch

telemtry   [R/P]:Kp,Kd,Ki,sat,Kw,pos,angle,minv
*/
int MessageLoop(unsigned long a,unsigned char b)
{
  charin = UARTCharGet(UART0_BASE);
  switch (charin)
  {
    int value;
    case 'a':
      GeneralPitchRoll(PITCH_MOTOR,A3906_DISABLE,0);
      GeneralPitchRoll(ROLL_MOTOR,A3906_DISABLE,0);
      break;
    case 'b':
      testpitch = !testpitch;
      break;
    case 'c':
      oscillateswitch = false;
      fullsweepswitch = false;
      break;
    case 'd':
      if (testpitch)
        k_a_pitch -=0.01;
      else
        k_a_roll -= 0.01;
      break;
    case 'D':
      if (testpitch)
        k_a_pitch += 0.01;
      else
        k_a_roll += 0.01;
        
      break;
    case 'e':
      if (testpitch)
      {
        stabilizeaxis[PITCH_MOTOR] = !stabilizeaxis[PITCH_MOTOR];
        GeneralPitchRoll(PITCH_MOTOR,A3906_DISABLE,0);
      }
      else
      {
        stabilizeaxis[ROLL_MOTOR] = !stabilizeaxis[ROLL_MOTOR];
        GeneralPitchRoll(ROLL_MOTOR,A3906_DISABLE,0);
      }
      break;
    case 'f':
    case 'F':
      oldtrace = traceflag;
      traceflag = true;
      fullsweepswitch = !fullsweepswitch;
      if (fullsweepswitch)
      {
        oscillatecount = 6; // do just 6 times
        traceflag = true;
      }
      oscillatemotor = (charin == 'f') ? PITCH_MOTOR : ROLL_MOTOR;
      value = lowerlimits[oscillatemotor];
      // kickoff to lower limit
      sweeptolow = true;
      SetValueEncoder(oscillatemotor,value,localdcycle[oscillatemotor]);
      break;
    case 'g':
      if (testpitch)
        k_g_pitch -= 0.01;
      else
        k_g_roll-= 0.01;
      break;
    case 'G':
      if (testpitch)
        k_g_pitch += 0.01;
      else
        k_g_roll += 0.01;
      break;
    case 'h':
		stabilizeaxis[ROLL_MOTOR] = !stabilizeaxis[ROLL_MOTOR];
		GeneralPitchRoll(ROLL_MOTOR,A3906_FORWARD ,50 );			

	break;

    case 'H':
		stabilizeaxis[ROLL_MOTOR] = !stabilizeaxis[ROLL_MOTOR];
		GeneralPitchRoll(ROLL_MOTOR,A3906_REVERSE ,50 );			
      //omega_vertical = (charin == 'h') ? 45.0 : 0;
      //omega_horizontal =  (charin == 'h') ? 45.0 : -45.0;
      break;
    case 'i':
      if (testpitch)
        k_i_pitch -= 0.01;
      else
        k_i_roll -= 0.01;
      break;
    case 'I':
      if (testpitch)
        k_i_pitch += 0.01;
      else
        k_i_roll += 0.01;
      break;

    case 'j':
    case 'J':
      omega_vertical = (charin == 'j') ? 0 : 0;
      omega_horizontal =  (charin == 'j') ? -89.0 : 89.0;
      break;

    case 'k':
    case 'K':
      omega_vertical = (charin == 'k') ? 0 : -10.0;
      omega_horizontal =  (charin == 'k') ? 0 : -65.0;
      break;
	  
//    case 'k':
//      ScriptStart();
   //   scriptactive = true;
//      break;
//    case 'K':
//      ScriptStop();
  //    scriptactive = false;
//      break;
 
    case 'm':
      // turn off stabilize on omap
      UART1Send(stoff,7);
      break;
    case 'n':
		if (testpitch)	
      		g_fmindutycycle[0][1]--;
		else
			g_fmindutycycle[1][1]--;
      break;
    case 'N':
		if (testpitch)	
      		g_fmindutycycle[0][1]++;
		else
			g_fmindutycycle[1][1]++;

      break;
    case 'o':
    case 'O':
      oldtrace = traceflag;
  //    traceflag = true;
      oscillateswitch = !oscillateswitch;
      if (oscillateswitch)
      {
        oscillatecount = 10; // do just 6 times
 //       traceflag = true;
      }
      oscillatemotor = testpitch/*(charin == 'o')*/ ? PITCH_MOTOR : ROLL_MOTOR;
      break;

    case 'q':
      brakelimit--;
      break;
    case 'Q':
      brakelimit++;
      break;
    case 'r':
		if (testpitch)		
			GeneralPitchRoll(PITCH_MOTOR,A3906_REVERSE,50);  
		else
			GeneralPitchRoll(ROLL_MOTOR,A3906_REVERSE,50);  
			
      //oscillaterate--;
      break;
	  
    case 'R':
		if (testpitch)				
			GeneralPitchRoll(PITCH_MOTOR,A3906_FORWARD,50);  
		else
			GeneralPitchRoll(ROLL_MOTOR,A3906_FORWARD,50);  			
		
      //oscillaterate++;
      break;
 
    case 't':
      traceflag = !traceflag;
      break;
    case 'v':

      localdcycle[testpitch ? 0 : 1]--;
      break;
    case 'V':

     localdcycle[testpitch ? 0 : 1]++;
	 
      break;
//    case PITCH_AXIS:
//    case ROLL_AXIS:
    //case 'j':
    case 'u':
      inbuff[0] = charin;
      inbufptr = 1;
      datain = true;
      break;

    case 'y':
      brakeOnChange = !brakeOnChange;
      break;
    case 'Y':
		
	  UART0Send("\r\n",4);		
	  UART0Send("\r\n",4);			  
      scenarioindex++;
      scenarioindex %= MAX_SCENARIOS;
      break;
  default:
      if (datain)  // start collection data
      {
        inbuff[inbufptr++] = charin;
        if (charin == 0x0d)
        {
          inbuff[inbufptr] = 0;  //terminate the string
          datain = false;
          dataready = true;
        }
      }  
      break;
  }
  charin = 0;
  return 0;
}

// chance to turn off motors
void  SysTickAction()
{
//  GimbalsBoundsCheck(PITCH_MOTOR);
 // GimbalsBoundsCheck(ROLL_MOTOR);
//  if (scriptactive)
//    ScriptExecute();
//  else
//    ScriptStart();
}

