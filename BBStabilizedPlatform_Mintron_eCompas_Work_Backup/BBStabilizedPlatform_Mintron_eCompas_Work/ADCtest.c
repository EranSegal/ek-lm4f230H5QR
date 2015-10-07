/*

  Copyright Bluebird Aero Systems Engineering 2011

  $Id: ADCtest.c,v 1.4 2011/03/24 12:31:38 EranS Exp $
  $Log: ADCtest.c,v $
  Revision 1.4  2011/03/24 12:31:38  EranS
  Added more commands for telemetry
  added power parameter to motor commands
  calibrated parameters for motor commands

  Revision 1.3  2011/03/23 12:15:32  EranS
  PWM test works, added Stow and Pilot functions

  Revision 1.2  2011/03/22 15:42:49  EranS
  added telemetry
  fixed adc code

  Revision 1.1  2011/03/21 13:42:03  EranS
  fix adc code, add telemtry

  Bluebird Stabilized Platform
*/

#include <stdio.h>
#include <string.h>
#include "bluebird.h"
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/debug.h"
#include "driverlib/sysctl.h"
#include "driverlib/interrupt.h"
#include "driverlib/uart.h"
#include "driverlib/timer.h"
#include "driverlib/pwm.h"
#include "MasterI2C.h"
#include "Console.h"
#include "SysTick.h"
#include "utilities.h"
#include "rmb20d01.h"
#include "A3906.h"

/* ---- Private Variables ------------------------------------------------ */

long g_ulFlags;
static unsigned char pbuff[20];

//*****************************************************************************
//
// The error routine that is called if the driver library encounters an error.
//
//*****************************************************************************
#ifdef DEBUG
void
__error__(char *pcFilename, unsigned long ulLine)
{
  sprintf("Driver %s %u \r\n",pcFilename,ulLine);
  UART0Send(pbuff, strlen(pbuff));
}
#endif
void HardwareInit()
{
  IntMasterDisable();
  //
  // Datasheet says use PLL if reading from ADC
  //
  SysCtlClockSet(SYSCTL_SYSDIV_10 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_6MHZ);
  ConsoleInit(115200);
  InitADC();
  IntMasterEnable();
}

void SoftwareInit()
{
}

int main(void)
{
  HardwareInit();
  SoftwareInit();
  while(1)
  {
    sprintf(pbuff,"%lu,%lu,%lu\r\n",g_ulTickCount,PitchRollData[0],PitchRollData[1]);
    UART0Send(pbuff,strlen(pbuff));
  }
}

// dummy functions to complete linkage
int ContropMessageLoop(unsigned long a,unsigned char b)
{
  return 0;
}

int ITG3200Action()
{
  return 0;
}

void A3906Action(int i)
{
}
