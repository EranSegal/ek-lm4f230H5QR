/*
  Copyright Bluebird Aero Systems Engineering 2012

  $Id: Console.c,v 1.4 2011/03/30 06:58:34 EranS Exp $
  $Log: Console.c,v $
  Revision 1.4  2011/03/30 06:58:34  EranS
  added more telemetry
  simplified and improved motor driver

  Revision 1.3  2011/03/22 15:42:49  EranS
  added telemetry
  fixed adc code

  Revision 1.2  2011/03/21 13:42:03  EranS
  fix adc code, add telemtry

  Revision 1.1  2011/03/15 12:23:45  EranS
  First save


*/

#include "bluebird.h"
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
//#include "driverlib/debug.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
//#include "driverlib/sysctl.h"
#include "driverlib/uart.h"

//*****************************************************************************
//
// The UART interrupt handler.
//
//*****************************************************************************
void UART0IntHandler(void)
{
    unsigned long ulStatus;

    //
    // Get the interrrupt status.
    //
    ulStatus = UARTIntStatus(UART0_BASE, true);

    //
    // Clear the asserted interrupts.
    //
    UARTIntClear(UART0_BASE, ulStatus);

    //
    // Loop while there are characters in the receive FIFO.
    //
    
    while(UARTCharsAvail(UART0_BASE))
    {
        //
        // Read the next character from the UART and write it back to the UART.
        //
        ContropMessageLoop(UART0_BASE,0);
    }
}

void ConsoleInit(unsigned long baud)
{
// Enable the peripherals used by this example.
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    //
    // Set GPIO A0 and A1 as UART pins.
    //
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    //
    // Configure the UART for 115,200, 8-N-1 operation.
    //
    UARTConfigSetExpClk(UART0_BASE, SysCtlClockGet(), baud,
                        (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                         UART_CONFIG_PAR_NONE));

    //
    // Enable the UART interrupt.
    //
    IntEnable(INT_UART0);
    UARTIntRegister(UART0_BASE,UART0IntHandler);
    UARTIntEnable(UART0_BASE, UART_INT_RX | UART_INT_RT);
}

//*****************************************************************************
//
// Send a string to the UART.
//
//*****************************************************************************
void
UART0Send(char *pucBuffer, unsigned ulCount)
{
    //
    // Loop while there are more characters to send.
    //
    while(ulCount--)
    {
        //
        // Write the next character to the UART.
        //
        UARTCharPut(UART0_BASE,*pucBuffer++);
    }
}
