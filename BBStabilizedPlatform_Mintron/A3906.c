/*
  Copyright Bluebird Aero Systems Engineering 2011

  $Id: A3906.c,v 1.18 2011/07/20 06:49:47 EranS Exp $
  $Log: A3906.c,v $
  Revision 1.18  2011/07/20 06:49:47  EranS
  changed ITG rate to 100
  encoder init ar higher PWM
  telemetry changed

  Revision 1.17  2011/06/18 09:13:50  EranS
  before france changes

  Revision 1.16  2011/05/11 07:39:16  EranS
  Integrate with BB library, stow/pilot buttons now work

  Revision 1.15  2011/05/03 07:32:34  EranS
  Add Brake in direction change

  Revision 1.14  2011/05/03 06:46:06  EranS
  After Eran rewire - pay attention to encoder pins

  Revision 1.13  2011/04/17 08:44:11  EranS
  add proper reset of encoders via GPIO

  Revision 1.12  2011/04/12 15:03:35  EranS
  added joystick interface

  Revision 1.11  2011/04/10 10:37:29  EranS
  Add filer3.c and AHRS from Lumus project

  Revision 1.10  2011/04/06 11:59:48  EranS
  unified stabilize code for pitch/roll

  Revision 1.9  2011/04/06 06:44:20  EranS
  cleaning up, add some constants, move gyro_at_rest code

  Revision 1.8  2011/04/06 05:44:34  EranS
  add stabilizing code

  Revision 1.7  2011/03/30 06:58:34  EranS
  added more telemetry
  simplified and improved motor driver

  Revision 1.6  2011/03/24 12:31:38  EranS
  Added more commands for telemetry
  added power parameter to motor commands
  calibrated parameters for motor commands

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


  A3906SESTR Motor Controller
*/

#include <stdlib.h>

#include "bluebird.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/debug.h"
#include "driverlib/gpio.h"
#include "driverlib/pwm.h"
#include "driverlib/sysctl.h"
#include "inc/hw_ints.h" 

#include "driverlib/adc.h"
#include "driverlib/interrupt.h"
#define A3906_MAIN
#include "A3906.h"
#include "SysTick.h"
#include "rmb20d01.h"
#include "utilities.h"
#include "GPIOhandler.h"
#include "Pins.h"

unsigned long ulPeriod;

void SetA3906Logic(enum MOTOR motor, enum A3906Logic direction);
///////////////////////////////////////
//unsigned long eulWidth;
///////////////////////////////////////
//***************************************************************************** 
//----------DEFINES 
#define    Encoder_Num_Pulse    207 
#define    PWM_Period           2000  //Hz (min 1300Hz) 

//*****************************************************************************
//
// Alias defines for the generators to relate them to the H-bridge hardware.
//
//*****************************************************************************
#define GEN_M_MINUS             PWM_GEN_0
#define GEN_M_PLUS              PWM_GEN_1
#define GEN_TIMING              PWM_GEN_2
#define GEN_M_MINUS_BIT         PWM_GEN_0_BIT
#define GEN_M_PLUS_BIT          PWM_GEN_1_BIT

//*****************************************************************************
//
// Alias defines for the comparator register offsets.
//
//*****************************************************************************
#define M_MINUS_CMP             PWM_O_0_CMPA
#define M_PLUS_CMP              PWM_O_1_CMPA
#define ADC_CMP                 PWM_O_0_CMPB

//*****************************************************************************
//
// Allias defines for the generator register offset.
//
//*****************************************************************************
#define M_MINUS_CTRL_GEN        PWM_O_0_GENB
#define M_MINUS_PWM_GEN         PWM_O_0_GENA
#define M_PLUS_CTRL_GEN         PWM_O_1_GENB
#define M_PLUS_PWM_GEN          PWM_O_1_GENA

//*****************************************************************************
//
//! The duty cycle of the waveform output to the Roll phase of the bridge.
//
//*****************************************************************************
static unsigned long g_ulPWMDutyCycleRoll;
//*****************************************************************************
//
//! The duty cycle of the waveform output to the Pitch phase of the bridge.
//
//*****************************************************************************
static unsigned long g_ulPWMDutyCyclePitch;

 
///////////////////////////////////////
//unsigned long eulWidth;
///////////////////////////////////////

/* ---- Private Function Prototypes -------------------------------------- */
void A3906Init()
{
    SysCtlPWMClockSet(SYSCTL_PWMDIV_1);	
	
    //
    // Enable the peripherals used by this device. (PWM0,1,2,3)
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM1);	
	
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);  	// PWM 0,1 ROLL_MOTOR 
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);     	// PWM 2,3 PITCH_MOTOR
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);  	// FL1 + FL2

	GPIOPinConfigure(GPIO_PB6_M0PWM0);    
	GPIOPinConfigure(GPIO_PB7_M0PWM1);   	
	
	GPIOPinConfigure(GPIO_PA6_M1PWM2);    
	GPIOPinConfigure(GPIO_PA7_M1PWM3);    
	
    //
    // Set GPIO PWM pins.  They are used to output the PWM0-3 signals
    //
    GPIOPinTypePWM(GPIO_PORTA_BASE, GPIO_PIN_6 | GPIO_PIN_7); //pwm1 2-3
    GPIOPinTypePWM(GPIO_PORTB_BASE, GPIO_PIN_6 | GPIO_PIN_7); //pwm0 2-3
    
    // Set the PWM period to 20KHz.
    // N = (1 / F) * SysClk. 
    // Where N is the function parameter
    // F is the desired frequency
    // SysClk is the system clock frequency.
    // In this case you get: (1 / 20KHz) * 80MHz = 4000 cycles.  Note that
    ulPeriod = 4000;// SysCtlClockGet() / PWMRATE;

    //ulPeriod = SysCtlClockGet() / 50000;//PWMRATE;
	//
	// Set the PWM period to chosen rate.
	//
	PWMGenConfigure(PWM_BASE, PWM_GEN_0, PWM_GEN_MODE_UP_DOWN | PWM_GEN_MODE_NO_SYNC);
	PWMGenPeriodSet(PWM_BASE, PWM_GEN_0, ulPeriod);
		
	//PWMGenConfigure(PWM_BASE, PWM_GEN_1, PWM_GEN_MODE_UP_DOWN | PWM_GEN_MODE_NO_SYNC);
	//PWMGenPeriodSet(PWM_BASE, PWM_GEN_1, ulPeriod);

	PWMOutputInvert(PWM_BASE,PWM_OUT_0_BIT | PWM_OUT_1_BIT, true);
	//
	// Enable the PWM0 and PWM1 output signals.
	//
	PWMOutputState(PWM_BASE, PWM_OUT_0_BIT | PWM_OUT_1_BIT, true);
	//
	// Enable the PWM generators.
	//
	PWMGenEnable(PWM_BASE, PWM_GEN_0);
	//PWMGenEnable(PWM_BASE, PWM_GEN_1);

	//
	// Set the PWM period to chosen rate.
	//
	//PWMGenConfigure(PWM1_BASE, PWM_GEN_0, PWM_GEN_MODE_UP_DOWN | PWM_GEN_MODE_NO_SYNC);
	//PWMGenPeriodSet(PWM1_BASE, PWM_GEN_0, ulPeriod);
		
	PWMGenConfigure(PWM1_BASE, PWM_GEN_1, PWM_GEN_MODE_UP_DOWN | PWM_GEN_MODE_NO_SYNC);
	PWMGenPeriodSet(PWM1_BASE, PWM_GEN_1, ulPeriod);

	PWMOutputInvert(PWM1_BASE,PWM_OUT_2_BIT | PWM_OUT_3_BIT, true);
	//
	// Enable the PWM0 and PWM1 output signals.
	//
	PWMOutputState(PWM1_BASE, PWM_OUT_2_BIT | PWM_OUT_3_BIT, true);
	//
	// Enable the PWM generators.
	//
	//PWMGenEnable(PWM1_BASE, PWM_GEN_0);
	PWMGenEnable(PWM1_BASE, PWM_GEN_1);

	
#if 0
    // setup interrupt handlers for overcurrent
    GPIOPinInit(A3906_OC1_INTERRUPT_PORT,A3906_OC1_INTERRUPT_PIN,true,true);
    GPIOPinInit(A3906_OC2_INTERRUPT_PORT,A3906_OC2_INTERRUPT_PIN,true,true);
#endif
    desiredencoder[0] = -1;  // means not active
    desiredencoder[1] = -1;
    dcycle[0] = PITCH_MINIMUM_DUTYCYCLE; // recommended initial values
    dcycle[1] = ROLL_MINIMUM_DUTYCYCLE;
    brakeOnChange = true;
}

// handlers for OC interrupts
int motorinterrupt;
void A3906Action(int motor)
{
  motorinterrupt = motor;
}

/*
  * See A3906 datasheet
  * Pins  00 - disable
  *       01 - Reverse
  *       10 - forward
  *       11 - brake
*/

void SetA3906Logic(enum MOTOR motor, enum A3906Logic direction)
{
  unsigned long bit0,bit1;
  tBoolean b0,b1;
  switch (motor)
  {
    case PITCH_MOTOR:
      bit0 = PWM_OUT_0_BIT;
      bit1 = PWM_OUT_1_BIT;
      break;
    case ROLL_MOTOR:
      bit0 = PWM_OUT_2_BIT;
      bit1 = PWM_OUT_3_BIT;
      break;
  }
  switch (direction)
  {
    case A3906_FORWARD:
      b0 = true;
      b1 = false;
      break;
    case A3906_REVERSE:
      b0 = false;
      b1 = true;
      break;
    case A3906_BRAKE:
      b0 = true;
      b1 = true;
      break;
    case A3906_DISABLE:
      b0 = false;
      b1 = false;
      break;
  }

  if (motor == PITCH_MOTOR)
  {
	  PWMOutputState(PWM_BASE, bit0, b0);
	  PWMOutputState(PWM_BASE, bit1, b1);
  }
  else
  {
	  PWMOutputState(PWM1_BASE, bit0, b0);
	  PWMOutputState(PWM1_BASE, bit1, b1);
  }  
  
  
  
}

//*****************************************************************************
//
//! Turns off all the PWM outputs.
//!
//! This function turns off all of the PWM outputs, preventing them from being
//! propagates to the gate drivers.
//!
//! \return None.
//
//*****************************************************************************
void
PWMOutputOff(void)
{
    //
    // Disable all six PWM outputs.
    //
	PWMOutputState(PWM_BASE, PWM_OUT_0_BIT | PWM_OUT_1_BIT, true);
	PWMOutputState(PWM1_BASE, PWM_OUT_2_BIT | PWM_OUT_3_BIT, true);

    //
    // Set the PWM duty cycles to 50%.
    //
    g_ulPWMDutyCycleRoll = 32768;
    g_ulPWMDutyCyclePitch = 32768;

    //
    // Set the PWM period so that the ADC runs at 1 KHz.
    //
    PWMGenPeriodSet(PWM_BASE, PWM_GEN_0, PWMRATE / 1000);
	PWMGenPeriodSet(PWM_BASE, PWM_GEN_1, PWMRATE / 1000);
    PWMGenPeriodSet(PWM1_BASE, PWM_GEN_0, PWMRATE / 1000);
	PWMGenPeriodSet(PWM1_BASE, PWM_GEN_1, PWMRATE / 1000);

    //
    // Update the PWM duty cycles.
    //
    PWMUpdateDutyCycle();
}

//*****************************************************************************
//
//! Updates the duty cycle in the PWM module.
//!
//! This function programs the duty cycle of the PWM waveforms into the PWM
//! module.  The changes will be written to the hardware and the hardware
//! instructed to start using the new values the next time its counters reach
//! zero.
//!
//! \return None.
//
//*****************************************************************************
static void
PWMUpdateDutyCycle(void)
{
    unsigned long ulWidth;

    //
    // Get the pulse width of the U phase of the motor.  If the width of the
    // positive portion of the pulse is less than the minimum pulse width, then
    // force the signal low continuously.  If the width of the negative portion
    // of the pulse is less than the minimum pulse width, then force the signal
    // high continuously.
    //
    ulWidth = (g_ulPWMDutyCycleRoll * ulPeriod) / 65536;
    //if(ulWidth < g_ulMinPulseWidth)
    //{
      //  ulWidth = g_ulMinPulseWidth;
    //}
    //if((g_ulPWMClock - ulWidth) < g_ulMinPulseWidth)
    //{
      //  ulWidth = g_ulPWMClock - g_ulMinPulseWidth;
    //}

    //
    // Set the pulse width of the ROLL phase of the motor.
    //
    PWMPulseWidthSet(PWM_BASE, PWM_OUT_0, ulWidth);
    PWMPulseWidthSet(PWM_BASE, PWM_OUT_1, ulWidth);

    //
    // Get the pulse width of the Pitch phase of the motor.
    //
    ulWidth = (g_ulPWMDutyCyclePitch * ulPeriod) / 65536;
    //if(ulWidth < g_ulMinPulseWidth)
    //{
      //  ulWidth = g_ulMinPulseWidth;
    //}
    //if((g_ulPWMClock - ulWidth) < g_ulMinPulseWidth)
    //{
      //  ulWidth = g_ulPWMClock - g_ulMinPulseWidth;
    //}

    //
    // Set the pulse width of the Pitch phase of the motor.
    //
    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_0, ulWidth);
    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_1, ulWidth);


    //
    // Perform a synchronous update of all three PWM generators.
    //
    PWMSyncUpdate(PWM_BASE, PWM_GEN_0_BIT | PWM_GEN_1_BIT);
    PWMSyncUpdate(PWM1_BASE, PWM_GEN_0_BIT | PWM_GEN_1_BIT);
	
}

/*
    Motor PITCH or ROLL
    logic FORWARD, REVERSE or BRAKE
    dutycycle percebtage from 0 to 100
*/
void GeneralPitchRoll(enum MOTOR motor, enum A3906Logic logic,unsigned long dutycycle)
{
  unsigned long pins[2];
  // sanity check
  
  dutycycle = min(dutycycle,ulPeriod-((ulPeriod*5)/100));
  	
  // ignore if repeat of previous conditions
  //if (logic == mdirection[motor] && dutycycle == dcycle[motor])
    //return;
  if (motor == PITCH_MOTOR)
  {
    pins[0] = PWM_OUT_0;
    pins[1] = PWM_OUT_1;
  }
  else
  {
    pins[0] = PWM_OUT_2;
    pins[1] = PWM_OUT_3;
  }  
  /*
      Good practice says that change of direction should go via BRAKE for 1 millsec
  */

  //eulWidth = (ulPeriod * dutycycle) / 100;
    
  mdirection[motor] = logic;  // for telemetry
  dcycle[motor] = dutycycle;
  
  SetA3906Logic(motor,logic);
  if (motor == PITCH_MOTOR)
  {
  
	  // set both pins for brake or 1 pin for forward/reverse
	  if (logic == A3906_BRAKE)
	  {
	    //PWMPulseWidthSet(PWM_BASE,pins[0], (ulPeriod * dutycycle) / 100);
	    //PWMPulseWidthSet(PWM_BASE,pins[1], (ulPeriod * dutycycle) / 100);
		
	    PWMPulseWidthSet(PWM_BASE,pins[0], ulPeriod);
	    PWMPulseWidthSet(PWM_BASE,pins[1], ulPeriod);		
	  }
	  else
	    PWMPulseWidthSet(PWM_BASE,
	                   logic == A3906_FORWARD ? pins[0] : pins[1],dutycycle);
	    //PWMPulseWidthSet(PWM_BASE,
	    //               logic == A3906_FORWARD ? pins[0] : pins[1],
	    //               (ulPeriod * dutycycle) / 100);
	  	  
	  	//PWMSyncUpdate(PWM_BASE, PWM_GEN_0_BIT | PWM_GEN_1_BIT);
	  
  }
  else
  {
	  // set both pins for brake or 1 pin for forward/reverse
	  if (logic == A3906_BRAKE)
	  {
	    //PWMPulseWidthSet(PWM1_BASE,pins[0], (ulPeriod * dutycycle) / 100);
	    //PWMPulseWidthSet(PWM1_BASE,pins[1], (ulPeriod * dutycycle) / 100);
	    
	    PWMPulseWidthSet(PWM1_BASE,pins[0], ulPeriod);
	    PWMPulseWidthSet(PWM1_BASE,pins[1], ulPeriod);
	  }
	  else
	    PWMPulseWidthSet(PWM1_BASE,
	                   logic == A3906_FORWARD ? pins[0] : pins[1],dutycycle);
	    //PWMPulseWidthSet(PWM1_BASE,
		//			 logic == A3906_FORWARD ? pins[0] : pins[1],
		//			 (ulPeriod * dutycycle) / 100);

	  //PWMSyncUpdate(PWM1_BASE, PWM_OUT_0_BIT | PWM_OUT_1_BIT);
	  

  }
  	
}

/*
    Arbitary check for when encoders wrap arount e.g. 10 -> 0 -> 1000
    1000 -> 1020 -> 10
    If wrap around detected, overwrite new value with previous value
*/
unsigned long PrevEnc[2]; 
tBoolean GimbalsBoundsCheck(enum MOTOR motor)
{
  tBoolean rc = false;
  	
#if 0
  if (!(Motor[motor].position > lowerlimits[motor] && Motor[motor].position < upperlimits[motor]))
  {
    GeneralPitchRoll(motor,A3906_DISABLE,90);
    rc = true;
  }
#else
  //if (Motor[motor].position == 0xFFFFFFFF )
	  //Motor[motor].position = PrevEnc[motor];	  	
  if (Motor[motor].position > 1000 && PrevEnc[motor] < 10)
  {
    rc = true;
    Motor[motor].position = PrevEnc[motor];
  }
  else if (Motor[motor].position < 10 && PrevEnc[motor] > 1000)
  {
    rc = true;
    Motor[motor].position = PrevEnc[motor];
  }  
  //else if( abs(Motor[motor].position - PrevEnc[motor]) > 500)
	  //rc = true;
  	
  PrevEnc[motor] = Motor[motor].position;
  
#endif
  return rc;
}

void SetValueEncoder(enum MOTOR motor,int newencoder,unsigned dutycycle)
{
  desiredencoder[motor] = newencoder;
  if (dutycycle)  // ignore zero
    dcycle[motor] = dutycycle;
}

/*
    Call this regularily to see if we have got to the desired place
*/
void MoveToEncoder(enum MOTOR motor)
{
  unsigned long currentencoder,newencoder;
  enum A3906Logic dir = A3906_BRAKE;
  tBoolean withinbounds;
  if (desiredencoder[motor] != -1)
  {
    newencoder = desiredencoder[motor];
    currentencoder = Motor[motor].position;
#ifdef FORWARD_DECREASES_ENCODER
    dir = (newencoder > currentencoder) ? A3906_FORWARD : A3906_REVERSE;
#elif defined(FORWARD_INCREASES_ENCODER)
    dir = (newencoder < currentencoder) ? A3906_FORWARD : A3906_REVERSE;
#else
#error Must define behaviour
#endif
      withinbounds = ( currentencoder >= ( newencoder - 3)) && ( currentencoder <= ( newencoder + 3));
    if (!withinbounds)
    {
      if (dir != A3906_BRAKE /*&& !GimbalsBoundsCheck(motor)*/)
        GeneralPitchRoll(motor, dir ,dcycle[motor]);
    }
    else
    {
      SetValueEncoder(motor,-1,0);
      GeneralPitchRoll(motor,A3906_BRAKE,90);
    }
  }
}
