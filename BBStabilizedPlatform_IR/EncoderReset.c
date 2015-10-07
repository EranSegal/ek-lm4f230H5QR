/*
  Copyright Bluebird Aero Systems Engineering 2012

  $Id: EncoderReset.c,v 1.1 2011/05/03 06:46:07 EranS Exp $
  $Log: EncoderReset.c,v $
  Revision 1.1  2011/05/03 06:46:07  EranS
  After Eran rewire - pay attention to encoder pins

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
#include "driverlib/debug.h"
#include "driverlib/sysctl.h"
#include "driverlib/interrupt.h"
#include "driverlib/uart.h"
#include "driverlib/gpio.h"
//#include "driverlib/timer.h"
//#include "driverlib/pwm.h"
#include "Console.h"
#include "utilities.h"
#include "A3906.h"
#include "rmb20d01.h"
#include "SysTick.h"

/* ---- Private Variables ------------------------------------------------ */

char inc = 0;
//*****************************************************************************
//
// The error routine that is called if the driver library encounters an error.
//
//*****************************************************************************
void __error__(char *pcFilename, unsigned long ulLine)
{
}

void HardwareInit()
{
  IntMasterDisable();
  //
  // Datasheet says use PLL if reading from ADC
  //
  SysCtlClockSet(SYSCTL_SYSDIV_10 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_6MHZ);
//  SysCtlClockSet(SYSCTL_SYSDIV_10 | SYSCTL_USE_OSC | SYSCTL_OSC_MAIN | SYSCTL_XTAL_6MHZ);
  ConsoleInit(115200);
  A3906Init();
  SysTickInit(1000);
  RMBD01Init();
  IntMasterEnable();
}

void SoftwareInit()
{
  EncoderInitReset(PITCH_MOTOR);
  EncodeResetPitch();
  EncoderInitReset(ROLL_MOTOR);
  EncodeResetRoll();
}

char pbuff[80];
void PrintTelemetry()
{
  sprintf(pbuff,"%lu,%lu\r\n",PitchRollPosition[0],PitchRollPosition[1]);
  UART0Send(pbuff,strlen(pbuff));
}

int main(void)
{
  HardwareInit();
  SoftwareInit();
  UART0Send("Main\r\n",6);
  while (true)
  {
    PrintTelemetry();
  }
}
// dummy for UART0 interrupt handler
/*
    a - reset pitch
    A - reset roll
*/
int ContropMessageLoop(unsigned long a,unsigned char b)
{
  inc = UARTCharGet(UART0_BASE);
  switch (inc)
  {
    case 'a':
    EncodeResetPitch();
    break;
    case 'A':
    EncodeResetRoll();
    break;
  }
  inc = 0;
  return 0;
}

//
// dummy for linkage
void ITG3200Action(void)
{
}

void SysTickAction()
{
}