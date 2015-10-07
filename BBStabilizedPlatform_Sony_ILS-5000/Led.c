/*
  Copyright Bluebird Aero Systems Engineering 2012

  $Id: Led.c,v 1.1 2011/03/15 12:23:45 EranS Exp $
  $Log: Led.c,v $
  Revision 1.1  2011/03/15 12:23:45  EranS
  First save

  Revision 1.3  2011/01/26 08:29:19  EranS

  Control LEDs on Bluebird board
*/

#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/debug.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/sysctl.h"
#include "inc/lm4f230h5qr.h"
#include "bluebird.h"
#include "led.h"
#include "reg.h"

void BBLedInit(void)
{
    //
    // Enable the GPIO pin for the LED (PC5).  Set the direction as output, and
    // enable the GPIO pin for digital function.  Care is taken to not disrupt
    // the operation of the JTAG pins on PC0-PC3.
    //
    //
    // Enable the GPIO port that is used for the on-board LED.
    // SYSCTL_PERIPH_GPIOD 
    // GPIO_PIN_0 GPIO_PIN_1     
    //
    SYSCTL_RCGC2_R = SYSCTL_RCGC2_GPIOD;

    //
    // Enable the GPIO pin for the LED (PG2).  Set the direction as output, and
    // enable the GPIO pin for digital function.
    //
    GPIO_PORTD_DIR_R = 0x03;
    GPIO_PORTD_DEN_R = 0x03;
    
}

static char lstate;
static char lstate1;

void BBLedToggle(void)
{
  if (lstate)
	  DEBUG_LED1(0);	 
  else
	  DEBUG_LED1(1);	 
  lstate = !lstate;
}

void BBLedToggle1(void)
{
  if (lstate1)
	  DEBUG_LED2(0);	 
  else
	  DEBUG_LED2(1);	 
  lstate1 = !lstate1;
  
}

void UIFaultLEDBlink(void)
{
	DEBUG_LED1(1);    
	DEBUG_LED1(0);	
}

