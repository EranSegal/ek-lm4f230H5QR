/*
  Copyright Bluebird Aero Systems Engineering 2011

  $Id: reg.c,v 1.3 2011/04/10 10:37:29 EranS Exp $
  $Log: ITG3200.h,v $
  Revision 1.3  2011/04/10 10:37:29  EranS
  Add filer3.c and AHRS from Lumus project

  Revision 1.2  2011/04/06 06:44:20  EranS
  cleaning up, add some constants, move gyro_at_rest code

  Revision 1.1  2011/03/15 12:23:45  EranS
  First save


*/

#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/debug.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "utils/uartstdio.h"
#include "driverlib/i2c.h"
#include "driverlib/adc.h"
#include "driverlib/timer.h"
#include "utils/ustdlib.h"
#include "inc/lm4f230h5qr.h"
#include "driverlib/interrupt.h"
#include "Interpret.h"
#include "comproc.h"
#include "globals.h"
#include "modeid.h"
#include "mtype.h"
#include "reg.h"

static int g_iTp5Status;

// Configuration Registers
unsigned long Register[] =  { 
					  0x0000,			// LED_CONTROL_REG
                      0x0000, 			// MODEM_REGISTER
                      0x0000, 			// MODEM_REGISTER
                      0x0000, 			// CONFIG1_REGISTER
                      0x0000, 			// FPGA_CONTROL_WR_REG
                      0x0000, 			// 
                      0x0000, 			// 
                      0x0000  			// RESET_REGISTER
                    } ;

void Set(WORD reg, WORD mask)
{
  Register[reg] |= mask; 
}

WORD Get(WORD reg, WORD mask)
{
  return Register[reg] & mask; 
}

void Clear(WORD reg, WORD mask)
{
  Register[reg] &= ~mask; 
}

void SetGPIOReg(WORD reg, WORD mask, WORD val)
{
	
   //status_reg         *p_status  = (status_reg*)   GPIO_PORTD_DATA_R;
   Register[reg] = (Register[reg] & ~mask) | val;
	
   switch(reg){
         
      case LED_CONTROL_REG:
         HW_LED_CONTROL = Register[reg];
        break;
	case TP_REGISTER:
	   HW_TP_CONTROL = Register[reg];
	  break;

			
	}                  
}

void TP6Toggle(void)
{
  if (g_iTp5Status)
	  DEBUG_TP6(0);	 
  else
	  DEBUG_TP6(1);	 
  g_iTp5Status = !g_iTp5Status;
  
}
