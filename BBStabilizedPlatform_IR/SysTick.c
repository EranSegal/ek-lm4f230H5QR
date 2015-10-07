/*
  Copyright Bluebird Aero Systems Engineering 2012

  $Id: SysTick.c,v 1.5 2011/03/30 06:58:34 EranS Exp $
  $Log: SysTick.c,v $
  Revision 1.5  2011/03/30 06:58:34  EranS
  added more telemetry
  simplified and improved motor driver

  Revision 1.4  2011/03/23 12:15:32  EranS
  PWM test works, added Stow and Pilot functions

  Revision 1.3  2011/03/22 15:42:49  EranS
  added telemetry
  fixed adc code

  Revision 1.2  2011/03/21 13:42:03  EranS
  fix adc code, add telemtry

  Revision 1.1  2011/03/15 12:23:45  EranS
  First save

 */
#include "globals.h"
#include "driverlib/fpu.h"
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/debug.h"
#include "driverlib/sysctl.h"
#include "driverlib/systick.h"
#include "driverlib/timer.h"
#include "driverlib/interrupt.h"
#define SYSTICK_MAIN
#include "SysTick.h"
#include "comproc.h"
#include "bluebird.h"
#include "A3906.h"
#include "rmb20d01.h"
#include "Interpret.h"
#include "encoder.h"

extern void MotorBrakePositionSet(enum MOTOR motor,unsigned long point);
extern void MotorBrakePositionClean(enum MOTOR motor);
extern int MotorPositionError(enum MOTOR motor);

//extern MINTRON_TXRX_S LensZoomWrite;


//*****************************************************************************
//
// The interrupt handler for the first timer interrupt.
//
//*****************************************************************************
void
Timer0IntHandler(void)
{

    //
    // Clear the timer interrupt.
    //
    TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);

    //
    // Update the interrupt status.
    //
    IntMasterDisable();

	//if(GetDMCflag(DMC_ENGINE_CONNECT))
	//{										

   	    //SetDMCflag(DMC_MOTOR_TEST,DMC_MOTOR_TEST);

		//if(!GetDMCflag(DMC_MOTOR_TEST))
		//{										
		
			//if(MotorPositionError(PITCH_MOTOR) == -1)
			//{
				//Set_OSD_Titel(0xF);			
				//cpPutMessage(UART1_BASE,0x10,sizeof(LensZoomWrite),(unsigned char *)&LensZoomWrite);  
			//}
			//else
			//if(MotorPositionError(ROLL_MOTOR) == -1)
			//{
				//Set_OSD_Titel(0xF);			
				//cpPutMessage(UART1_BASE,0x10,sizeof(LensZoomWrite),(unsigned char *)&LensZoomWrite);  		
			//}
		//}
	//}	


	Motor[ROLL_MOTOR].deg2sec = ((Motor[ROLL_MOTOR].aangle - Motor[ROLL_MOTOR].cur_angle)*Motor[ROLL_MOTOR].direction)*100.0f;			
	Motor[ROLL_MOTOR].cur_angle = Motor[ROLL_MOTOR].aangle;
	
	Motor[PITCH_MOTOR].deg2sec = ((Motor[PITCH_MOTOR].aangle - Motor[PITCH_MOTOR].cur_angle)*Motor[PITCH_MOTOR].direction)*100.0f;				
	Motor[PITCH_MOTOR].cur_angle = Motor[PITCH_MOTOR].aangle;
	
    IntMasterEnable();
}

//*****************************************************************************
//
// Handles the SysTick timeout interrupt.
//
//*****************************************************************************
void
SysTickIntHandler(void)
{
    //
    // Increment the tick count.
    //
    g_ulTickCount++;
}

unsigned long GetSysTickCount()
{
  return g_ulTickCount;
}

/*
  Set desired rate and register the interrupt handler
*/
void SysTickInit(unsigned long rate)
{
   SysTickPeriodSet(SysCtlClockGet()/rate);
   SysTickIntEnable();
   SysTickEnable();
}

//
// Initialize timer processor interrupts.
//
void TimerInit(unsigned long rate)
{

	SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);

	//
	// Enable processor interrupts.
	//
	IntMasterEnable();

	//
	// Configure the two 32-bit periodic timers.
	//
	TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);
	TimerLoadSet(TIMER0_BASE, TIMER_A, SysCtlClockGet()/rate);

	//
	// Setup the interrupts for the timer timeouts.
	//
	IntEnable(INT_TIMER0A);
	TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);

	//
	// Enable the timers.
	//
	TimerEnable(TIMER0_BASE, TIMER_A);
}
