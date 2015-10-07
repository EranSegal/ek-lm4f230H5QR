/*
  Copyright Bluebird Aero Systems Engineering 2012

  //////////////////////////////////////////
  Suitable for IAR ver 6.40.1
  //////////////////////////////////////////
  
  $Id: StabilizedPlatform.c,v 1.14 2011/07/14 07:19:48 EranS Exp $
  $Log: StabilizedPlatform.c,v $

  Revision 1.14.1  2012/02/12 06:51:48  EranS
  Add pitch&roll limitation
  
  Revision 1.14  2011/07/14 07:19:48  EranS
  added script engine

  Revision 1.13  2011/06/18 09:13:50  EranS
  before france changes

  Revision 1.12  2011/05/11 07:39:16  EranS
  Integrate with BB library, stow/pilot buttons now work

  Revision 1.11  2011/05/03 07:33:52  EranS
  Move omega initialization code, rename ADC init function

  Revision 1.10  2011/04/12 15:03:35  EranS
  added joystick interface

  Revision 1.9  2011/04/12 05:50:35  EranS
  calibrated gimbal / gyro / accelerometer

  Revision 1.8  2011/04/10 10:37:29  EranS
  Add filer3.c and AHRS from Lumus project

  Revision 1.7  2011/04/06 13:09:03  EranS
  modified BB application to work with stabilizer

  Revision 1.6  2011/04/06 11:59:48  EranS
  unified stabilize code for pitch/roll

  Revision 1.5  2011/03/23 12:15:32  EranS
  PWM test works, added Stow and Pilot functions

  Revision 1.4  2011/03/22 15:42:49  EranS
  added telemetry
  fixed adc code

  Revision 1.3  2011/03/21 13:42:03  EranS
  fix adc code, add telemtry

  Revision 1.2  2011/03/15 15:00:45  EranS
  changed ADC functions

  Revision 1.1  2011/03/15 12:23:45  EranS
  First save


  Bluebird Stabilized Platform
*/



#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "bluebird.h"
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "utils/ustdlib.h"
#include "driverlib/gpio.h"
#include "driverlib/adc.h"
#include "driverlib/debug.h"
#include "driverlib/fpu.h"
#include "driverlib/sysctl.h"
#include "driverlib/rom.h"
#include "driverlib/interrupt.h"
#include "driverlib/uart.h"
#include "driverlib/timer.h"
#include "driverlib/pwm.h"
#include "driverlib/qei.h"
#include "driverlib/pin_map.h"
#include "MasterI2C.h"
#include "ITG3200.h"
#include "bma180.h"
#include "uart_echo.h"
#include "SysTick.h"
#include "utilities.h"
#include "Led.h"
#include "A3906.h"
#include "rmb20d01.h"
#include "Stabilize.h"
#include "Interpret.h"
#include "comproc.h"
#include "pins.h"
#include "MAG3110.h"
#include "encoder.h"
#include "filter3.h"
#include "reg.h"
#include "comproc.h"

#define until(B) {volatile unsigned long long dd=0xffffffff; do{if(!B) break;}while(--dd);}

/* ---- Private Variables ------------------------------------------------ */
//#define TELEMETRY
// 2.6.2 for MINTRON Engine version

tBoolean StowBitSet = false;
tBoolean PilotBitSet = false;
tBoolean onetimepilot = false;
tBoolean onetimestow = false;
tBoolean stabilizepitch = true;
// output from joystick, from -128 to +128 each axis
unsigned long lastitg;
long g_lqei0dir,g_lqei1dir;
unsigned long g_ulqei0pos,g_ulqei1pos;
tBoolean g_bstow_pilot = false;
unsigned char pbuff[300];

// JOYSTICK ////////////////////////////////////////////////////////////////
// output from joystick, from -128 to +128 each axis
//int   joy_ver = 0, joy_hor=0;
////////////////////////////////////////////////////////////////////////////
void SetBitMode(void);
void AHRS_Init();
void AHRS();

tBoolean g_bInit = true;

extern float omega_vertical,omega_horizontal;  
extern float Phic,Thetac;
extern float u[2];
extern unsigned long g_ulqei0pos_left,g_ulqei0pos_right;
extern tBoolean g_tbstabilizer;
extern ctrl_type ctrl;
extern MINTRON_TXRX_S LensZoomWrite;
extern unsigned long g_ulAverage;
extern float y_rate_roll;
//*****************************************************************************
//
// The error routine that is called if the driver library encounters an error.
//
//*****************************************************************************
#ifdef DEBUG
void
__error__(char *pcFilename, unsigned long ulLine)
{
}
#endif


unsigned long ulSysClock;
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
  ulSysClock = ROM_SysCtlClockGet();
  
   
  if (InitUART(UART0_BASE,SysCtlClockGet(),19200,
      	(UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE)) == -1)
      	  while (1);    // hang

  if (InitUART(UART1_BASE,SysCtlClockGet(),9600,
      	(UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE)) == -1)
      	  while (1);    // hang
      
  SysTickInit(1000); // 1us counter
  //TimerInit(1000);    	 // 0.2ms counter
  
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

  //GPIOPinInit('E',4,false,false); /*TP5 */
  //GPIOPinInit('E',5,false,false); /*TP6 */

  IntMasterEnable();
}

void InitParams(void)
{

	SetDMCflag(0xFFFF, 0);	
	SetDMCflag(DMC_DISABLE_ZOOM,DMC_DISABLE_ZOOM); 	
	SetDMCflag(DMC_STABILIZE_OFF,DMC_STABILIZE_OFF);
	SetDMCflag(DMC_RATE,DMC_RATE); 
	
	cpReset(0);
	cpReset(1);
	init_ctrl();
		
}

void SoftwareInit()
{

  //
  // initialization BlueBird parameters for communication protocol.
  //
  InitParams();
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
  IntPrioritySet(INT_I2C0, 0x00);
  IntPrioritySet(INT_GPIOA, 0x02);
  IntPrioritySet(INT_GPIOE, 0x40);  
  IntPrioritySet(QEI_PITCH_PHA_INT, 0x60);
  IntPrioritySet(QEI_ROLL_PHA_INT, 0x60);
  IntPrioritySet(INT_PWM0, 0x80);
  IntPrioritySet(INT_PWM1, 0x80);
  IntPrioritySet(INT_UART0, 0xA0);  
  IntPrioritySet(INT_UART1, 0xA0);
 

  //
  // PITCH_MOTOR is PORTB & J6- PITCH Encoder 
  //
  // moves gimbal to extreme right and down
  MotorBrakePositionClean(PITCH_MOTOR);
  //GeneralPitchRoll(PITCH_MOTOR,A3906_FORWARD,500);	 
  //EncoderInitReset(QEI0_BASE,PITCH_MOTOR,A3906_REVERSE);  // J6 PITCH Encoder  
  //EncoderInitReset(QEI0_BASE,PITCH_MOTOR,A3906_FORWARD);  // J6 PITCH Encoder  
  //EncoderInitReset(QEI0_BASE,PITCH_MOTOR,A3906_REVERSE);  // J6 PITCH Encoder  

  //
  // ROLL_MOTOR is PORTA & J8- ROLL Encoder
  //
  // moves gimbal to extreme right and down    
  MotorBrakePositionClean(ROLL_MOTOR);
  //GeneralPitchRoll(ROLL_MOTOR,A3906_FORWARD,800);
  //EncoderInitReset(QEI1_BASE,ROLL_MOTOR,A3906_FORWARD); // J8 ROLL Encoder  
  //EncoderInitReset(QEI1_BASE,ROLL_MOTOR,A3906_REVERSE); // J8 ROLL Encoder
  //EncoderInitReset(QEI1_BASE,ROLL_MOTOR,A3906_FORWARD); // J8 ROLL Encoder  
  
  #ifdef USE_BMA180_INTERRUPT
  AHRS_Init();
  #endif
  
  lastitg = g_ulTickCount;  
  StabilizeInit();
  //SetBitMode();

  
}
unsigned long dmcflag;
unsigned int loopcount;
float PitchAmp = 0 ;

 int main(void)
{

  unsigned long last_telemtry = SysTickCountd;

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

  while(1)
  {

    	// Execute main loop to interpretation protocol command	    
    	//MainModeLoop();	

		if(itgready == true){
		//if ((SysTickCountd - last_telemtry) > 700){	
	
			//last_telemtry = SysTickCountd; 				
				    
			/*------------------------------------------------------------*/
			/* EncoderTick(QEI0_BASE); time is 2.5~ microsecond			  */
			/* EncoderTick(QEI0_BASE); time is 2.5~ microsecond			  */
			/*------------------------------------------------------------*/
			EncoderTick(QEI0_BASE); // PITCH Encoder	  
			EncoderTick(QEI1_BASE); // ROLL Encoder
			
			//DEBUG_TP6(1);
			/*------------------------------------------------------------*/
			/* ITG3200getXYZ() time is 230~ microsecond						   */
			/*------------------------------------------------------------*/	
			ITG3200getXYZ();
			//DEBUG_TP6(0);
			
			#ifdef USE_BMA180_INTERRUPT
			/*------------------------------------------------------------*/
			/* BMA180GetXYZ() time is 230~ microsecond						   */
			/*------------------------------------------------------------*/				
			BMA180GetXYZ(); 				
			/*------------------------------------------------------------*/
			/* AHRS() time is 36~ microsecond										 */
			/*------------------------------------------------------------*/					
			AHRS();
			#endif
			
			if (g_tbstabilizer == true)
			{
				/*------------------------------------------------------------*/
				/* Stabilize() time is 30~ microsecond								  */
				/*------------------------------------------------------------*/						
				Stabilize(ROLL_MOTOR);
				/*------------------------------------------------------------*/
				/* Stabilize() time is 30~ microsecond								  */
					
				/*------------------------------------------------------------*/									
				Stabilize(PITCH_MOTOR);
			}		
			
			
			//DEBUG_TP6(1);
			//sprintf(pbuff,"%.2f\r\n",y_rate_roll);
			//sprintf(pbuff,"%hd,%hd,%hd\r\n",
			//ITG3200values[0],ITG3200values[1],ITG3200values[2]);
			//UART0Send(pbuff, strlen(pbuff)); 
				
	}	
	//DEBUG_TP6(0);

	#if 0
	if(1)//new_adc == true)
	{
	    new_adc=false;
	    //PitchAmp = (float)PitchRollAmp[PITCH_MOTOR];
		//PitchAmp = (PitchAmp /FULL_SCALE_BIT)*3; // 2Exp12 * 3V
		//PitchAmp = PitchAmp / 5;
		//PitchAmp*=1000;
		//sprintf(pbuff,"%.2fmA\r\n",PitchAmp);
		//sprintf(pbuff,"%lu,%u,%u\r\n",g_ulTickCount,PitchRollAmp[PITCH_MOTOR],PitchRollAmp[ROLL_MOTOR]);	
		sprintf(pbuff,"%.2f\r\n",y_rate_roll);
		//sprintf(pbuff,"%lu,%lu\r\n",g_ulTickCount,g_ulAverage);
		
		UART0Send(pbuff, strlen(pbuff)); 
	
	}
	#endif
	
	

  }
}

// chance to turn off motors
void  SysTickAction()
{
  GimbalsBoundsCheck(PITCH_MOTOR);
  GimbalsBoundsCheck(ROLL_MOTOR);
}
