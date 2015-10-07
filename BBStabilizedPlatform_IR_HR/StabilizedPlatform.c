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
#include "GPIOhandler.h"
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
#include "modeid.h"

/* ---- Private Variables ------------------------------------------------ */
//#define TELEMETRY
// 1.4.x for SCD Engine version
char  *BBVersion  = "Build:1.4.7";
volatile unsigned char controlReg = 0;
volatile unsigned short heading = 0; 

tBoolean ignorejoystick = false;
tBoolean g_tbstabilizer = true;
tBoolean stabilizepitch = true;

void AHRS_Init();
void AHRS();
void SoftwareInit();

//*****************************************************************************
//
// The control table used by the uDMA controller.  This table must be aligned
// to a 1024 byte boundary.
//
//*****************************************************************************
#if defined(ewarm)
#pragma data_alignment=1024
unsigned char ucControlTable[1024];
#elif defined(ccs)
#pragma DATA_ALIGN(ucControlTable, 1024)
unsigned char ucControlTable[1024];
#else
unsigned char ucControlTable[1024] __attribute__ ((aligned(1024)));
#endif

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

   // Set the system clock to run at 10MHz from the PLL. // PLL=400MHz
   //ROM_SysCtlClockSet(SYSCTL_SYSDIV_10 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);
  
   // Set the system clock to run at 80MHz from the PLL. // PLL=400MHz
   ROM_SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);

   //
   // Initialize the uDMA UART transfers.
   //
   if (InitUART(UART0_BASE,SysCtlClockGet(),19200,
		 (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE)) == -1)
		   while (1);	 // hang
   
   if (InitUART(UART1_BASE,SysCtlClockGet(),115200,
		 (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE)) == -1)
		   while (1);	 // hang

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

  //IntPrioritySet(INT_TIMER0A,     0x00);    // Set the priorities of the interrupts used by the application.
  IntPrioritySet(INT_I2C0, 		  0x20);  
  IntPrioritySet(INT_GPIOA,       0x40);
  IntPrioritySet(INT_PWM0,        0x80);
  IntPrioritySet(INT_PWM1,        0xa0);
  IntPrioritySet(INT_UART0, 	  0xC0);
  IntPrioritySet(INT_UART1, 	  0xe0);      
  
  SysTickInit(1000);    // 1us counter
  //TimerInit(1000);    	 // 0.37ms counter
  //TimerInit1(1000);    	 // 1ms counter  
  BBLedInit();  

  //MasterI2C0Init(SysCtlClockGet() >= 40000000L);  // Need 40MHz for 400K
  MasterI2C0Init(true);  // Need 40MHz for 400K  
  A3906Init(); 

  Encoder_Init(QEI0_BASE);   // J6 PITCH Encoder 
  Encoder_Init(QEI1_BASE);   // J8 ROLL Encoder
  
  EncoderLinesSet(QEI0_BASE,64);
  EncoderLinesSet(QEI1_BASE,64);
  //GPIOPinInit('E',4,false,false); /*TP5 */
  //GPIOPinInit('E',5,false,false); /*TP6 */

  IntMasterEnable();
}

void SoftwareInit()
{


  //
  // initialization BlueBird parameters for communication protocol.
  //
  InitParams();
  ITG3200Init();  
  BMA180Init();  

}
unsigned long dmcflag;
unsigned int loopcount;
 int main(void)
{
  //unsigned long last_telemtry;

  //
  // Enable lazy stacking for interrupt handlers.  This allows floating-point
  // instructions to be used within interrupt handlers, but at the expense of
  // extra stack usage.
  //
  ROM_FPUEnable();
  ROM_FPULazyStackingEnable();

  HardwareInit();
  SoftwareInit();
         
  while(1)
  {
    	// Execute main loop to interpretation protocol command	    
    	MainModeLoop();	

		#if 1
		if(GetDMCmode() != BIT_MODE )
		{
			//if ((SysTickCountd - last_telemtry) > 700){	
			if(itgready){
			//last_telemtry = SysTickCountd; 	

			/*------------------------------------------------------------*/
			/* EncoderTick(QEI0_BASE); time is 2.5~ microsecond			  */
			/* EncoderTick(QEI0_BASE); time is 2.5~ microsecond			  */
			/*------------------------------------------------------------*/
			//DEBUG_TP6(1); 
			EncoderTick(QEI0_BASE); // PITCH Encoder	  
			EncoderTick(QEI1_BASE); // ROLL Encoder
			
			//DEBUG_TP6(1); 
			/*------------------------------------------------------------*/
			/* ITG3200getXYZ() time is 230~ microsecond						   */
			/*------------------------------------------------------------*/	
			ITG3200getXYZ();
			//DEBUG_TP6(0);	
			/*------------------------------------------------------------*/
			/* BMA180GetXYZ() time is 230~ microsecond						   */
			/*------------------------------------------------------------*/
			BMA180GetXYZ(); 
			
			/*------------------------------------------------------------*/
			/* AHRS() time is 36~ microsecond								    */
			/*------------------------------------------------------------*/					
			AHRS();

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
	
				/*------------------------------------------------------------*/
				/* Total time is 521~ microsecond test by EranS 						*/
				/* Gyro and Encoder dose not test yet. Need testing with new board			*/
				/*------------------------------------------------------------*/		
			}
			//DEBUG_TP6(0);
		}		
	}
	#endif	
  }		
}

