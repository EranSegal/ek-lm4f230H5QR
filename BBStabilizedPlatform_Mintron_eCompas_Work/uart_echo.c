
/*
  Copyright Bluebird Aero Systems Engineering 2012


  Revision 1.3  2011/06/01 13:42:03  EranS
  fix Interpret.c code, add  P.T.C return coordinate to BBGCS
  fix Uart_echo.c code, changed UART_CONFIG_STOP_TWO
  Must to clarification why ( nextVision use two stop ) in spite of Controp

  Revision 1.2  2011/06/19  EranS
  added military Joystick
  fixed Interpret.c code for salon Paris 
  
  Revision 1.1  2011/03/15 12:23:45  EranS
  First save


  Bluebird Stabilized Platform
*/

#include <string.h>
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/lm4f230h5qr.h"
#include "driverlib/debug.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "driverlib/rom.h"
#include "utils/uartstdio.h"
#include "driverlib/i2c.h"
#include "driverlib/adc.h"
#include "driverlib/timer.h"
#include "driverlib/pin_map.h"
#include "utils/ustdlib.h"
#include "driverlib/systick.h" 
#include "Bluebird.h"
#include "SysTick.h"
#include "Interpret.h"
#include "uart_echo.h"
#include "Comproc.h"
#include "modeid.h"
#include "globals.h"
#include "mtype.h"
#include "reg.h"

#define TXBUFFSIZE	255
#define TXBUFFSIZE1	255

char m_tTxBuff[TXBUFFSIZE];
char m_nTxBuffIn=0;
char m_nTxNextNdx=0;

char m_tTxBuff1[TXBUFFSIZE1];
char m_nTxBuffIn1=0;
char m_nTxNextNdx1=0;

//*****************************************************************************
//
//! \addtogroup example_list
//! <h1>UART (uart_echo)</h1>
//!
//! This example application utilizes the UART to echo text.  The first UART
//! (connected to the FTDI virtual serial port on the Stellaris LM3S811
//! Evaluation Board) will be configured in 115,200 baud, 8-n-1 mode.  All
//! characters received on the UART are transmitted back to the UART.
//
//*****************************************************************************

//*****************************************************************************
//
// The error routine that is called if the driver library encounters an error.
//
//*****************************************************************************
#ifdef DEBUG
void __error__(char *pcFilename, unsigned long ulLine)
{

}
#endif

int InitUART(unsigned long ulBase, unsigned long ulUARTClk,unsigned long ulBaud, unsigned long ulConfig)
{
		
	if(ulBase == UART0_BASE)
	{
		//
		// Enable the peripherals used by this example.
		//
		SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);		
		SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);		

		//Enable GPIO for UART0
		//PA0-> U0RX 
		//PA1-> U0Tx 
		GPIOPinConfigure(GPIO_PA0_U0RX);		
		GPIOPinConfigure(GPIO_PA1_U0TX);	
		
		//
		// Set GPIO A0 and A1 as UART pins.
		//
		GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
		
		//
		// Configure COM0 Controp the UART for 19,200, 8-N-1 operation.
		//
		//UARTConfigSetExpClk(ulBase, ulUARTClk, ulBaud,ulConfig);
		#if defined(stabilizition)
		//
		// Configure the UART for 19,200, 8-N-1 operation. 
		// Mux Must to get EVEN PAR bin from the Engine
		//
		UARTConfigSetExpClk(UART0_BASE, SysCtlClockGet(), 115200,
							(UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
							 UART_CONFIG_PAR_EVEN));
		#else
		UARTConfigSetExpClk(UART0_BASE, SysCtlClockGet(), 115200,
							(UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
							 UART_CONFIG_PAR_NONE));
		#endif
		//
		// Enable the UART interrupt.
		//
		IntEnable(INT_UART0);
		UARTFIFOLevelSet(UART0_BASE,UART_FIFO_TX1_8,UART_FIFO_RX1_8);
		//UARTIntRegister(UART0_BASE,UART0IntHandler);
		UARTIntEnable(UART0_BASE, UART_INT_RX | UART_INT_TX | UART_INT_RT);

		return 0;

	}
	else
	if(ulBase == UART1_BASE)
	{
	    //
	    // Enable the peripherals used by this example.
	    //
	    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);	    
	    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART1);		

		// Set B 0 and 1 to alternative use
		//GPIODirModeSet(GPIO_PORTB_BASE, GPIO_PIN_0, GPIO_DIR_MODE_HW); 		
		//GPIODirModeSet(GPIO_PORTB_BASE, GPIO_PIN_1, GPIO_DIR_MODE_HW);//GPIO_DIR_MODE_HW	

		//Enable GPIO for UART1 
		//PB0-> U1RX 
		//PB1-> U1Tx 
		GPIOPinConfigure(GPIO_PB0_U1RX);		
		GPIOPinConfigure(GPIO_PB1_U1TX);	

	    //
	    // Set GPIO A0 and A1 as UART pins.
	    //
	    GPIOPinTypeUART(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1);

	    //
	    // Configure COM1 OMAP the UART for 115,200, 8-N-1 operation.
	    //

		UARTConfigSetExpClk(UART1_BASE, SysCtlClockGet(), 9600,
							(UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
							 UART_CONFIG_PAR_NONE));

	    //
	    // Enable the UART interrupt.
	    //
		IntEnable(INT_UART1);
		UARTFIFOLevelSet(UART1_BASE,UART_FIFO_TX1_8,UART_FIFO_RX1_8);
		//UARTIntRegister(UART1_BASE,UART1IntHandler);
		UARTIntEnable(UART1_BASE, UART_INT_RX | UART_INT_TX | UART_INT_RT);


		return 0;
	}
	else
		return -1;

	
}


//*****************************************************************************
//
// The UART interrupt handler.
//
//*****************************************************************************
void UART0IntHandler(void)
{
    unsigned long ulStatus;
	tBoolean bRc;

    //
    // Get the interrrupt status.
    //
    ulStatus = UARTIntStatus(UART0_BASE, true);
    //
    // Clear the asserted interrupts.
    //
    UARTIntClear(UART0_BASE, ulStatus);

	//
	// Check what is the source of the interrupt
	//

	//if(ulStatus & UART_INT_OE)
	//{
	//}
	//else if(ulStatus & UART_INT_BE)
	//{
	//}
	//else if(ulStatus & UART_INT_PE)
	//{
	//}
	if(ulStatus & UART_INT_TX)
	{
		// TX int
		// Push next char to transmitter
		bRc = true;
		while(m_nTxBuffIn > 0 && bRc == true)
		{
			bRc = UARTCharPutNonBlocking(UART0_BASE,m_tTxBuff[m_nTxNextNdx]);
 			if(bRc == true)
			{
				m_nTxNextNdx++;
				m_nTxBuffIn--;
			}
		}
	}
	else 
	if(ulStatus & UART_INT_RX || ulStatus & UART_INT_RT)
	{
		// RX int
		// Read all RX fifo data
		while(UARTCharsAvail(UART0_BASE))
		{
			// Read the next character from the UART
			// (and write it back to the UART) ?
			#if defined(stabilizition)
			ContropMessageLoop(UART0_BASE,0);
			#else
			MessageLoop(UART0_BASE,0);
			#endif
			
		}
	}
}

//*****************************************************************************
//
// The UART interrupt handler.
//
//*****************************************************************************
void UART1IntHandler(void)
{
    unsigned long ulStatus;
	tBoolean bRc;

    //
    // Get the interrrupt status.
    //
    ulStatus = UARTIntStatus(UART1_BASE, true);
    //
    // Clear the asserted interrupts.
    //
    UARTIntClear(UART1_BASE, ulStatus);

	//
	// Check what is the source of the interrupt
	//

	//if(ulStatus & UART_INT_OE)
	//{
	//}
	//else if(ulStatus & UART_INT_BE)
	//{
	//}
	//else if(ulStatus & UART_INT_PE)
	//{
	//}
	if(ulStatus & UART_INT_TX)
	{
		// TX int
		// Push next char to transmitter
		bRc = true;
		while(m_nTxBuffIn1 > 0 && bRc == true)
		{
			bRc = UARTCharPutNonBlocking(UART1_BASE,m_tTxBuff1[m_nTxNextNdx1]);
 			if(bRc == true)
			{
				m_nTxNextNdx1++;
				m_nTxBuffIn1--;
			}
		}
	}
	else if(ulStatus & UART_INT_RX || ulStatus & UART_INT_RT)
	{
		// RX int
		// Read all RX fifo data
		while(UARTCharsAvail(UART1_BASE))
		{
			// Read the next character from the UART
			// (and write it back to the UART) ?
			//MessageLoop(UART0_BASE,0);
			MintronMessageLoop(UART1_BASE,1);						
		}
	}
}


void UART0Send(unsigned char *pucBuffer, unsigned ulCount)
{
	tBoolean bRc;
	static int FCount = 0;
	static int SCount = 0;

    //
    // Loop while there are more characters to send.
    //
	if(m_nTxBuffIn + ulCount >= TXBUFFSIZE)
	{
		// No place to put new data
		// To be sure we purge all tx data
		IntDisable(INT_UART0);
		#if 0
		// TX int
		// Push next char to transmitter
		while(m_nTxBuffIn > 0)
		{
			bRc = UARTCharPutNonBlocking(UART0_BASE,m_tTxBuff[m_nTxNextNdx]);
 			if(bRc == true)
			{
				m_nTxNextNdx++;
				m_nTxBuffIn--;
			}
		}

		memcpy(m_tTxBuff ,pucBuffer,ulCount);
		m_nTxNextNdx = 0;
		m_nTxBuffIn = ulCount;
		#else
		
		//memcpy(m_tTxBuff ,pucBuffer,ulCount);
		//m_nTxNextNdx = 0;
		//m_nTxBuffIn = ulCount;
		FCount++;
		// Fill hardware FIFO
		bRc = true;
		while(m_nTxBuffIn > 0 && bRc == true)
		{
			bRc = UARTCharPutNonBlocking(UART0_BASE,m_tTxBuff[m_nTxNextNdx]);
 			if(bRc == true)
			{
				m_nTxNextNdx++;
				m_nTxBuffIn--;
			}
				
		}
		
		#endif
		
		IntEnable(INT_UART0);
	}
	else if(ulCount > 0)
	{
		IntDisable(INT_UART0);
		SCount++;
		if(m_nTxBuffIn > 0)
		{
			// Move TX data to start
			if(m_nTxNextNdx > 0)
			{
				memmove(m_tTxBuff,&m_tTxBuff[m_nTxNextNdx],m_nTxBuffIn);
			}
			m_nTxNextNdx = 0;
			// Add data to TXBuff
			memcpy(&m_tTxBuff[m_nTxBuffIn],pucBuffer,ulCount);
			m_nTxBuffIn += ulCount;
		}
		else
		{
			memcpy(m_tTxBuff ,pucBuffer,ulCount);
			m_nTxNextNdx = 0;
			m_nTxBuffIn = ulCount;
		}
		// Fill hardware FIFO
		bRc = true;
		while(m_nTxBuffIn > 0 && bRc == true)
		{
			bRc = UARTCharPutNonBlocking(UART0_BASE,m_tTxBuff[m_nTxNextNdx]);
 			if(bRc == true)
			{
				m_nTxNextNdx++;
				m_nTxBuffIn--;
			}
				
		}
 		IntEnable(INT_UART0);
	}
}

void UART1Send(unsigned char *pucBuffer, unsigned ulCount)
{
	tBoolean bRc;
	static int FCount = 0;
	static int SCount = 0;
    //
    // Loop while there are more characters to send.
    //
	if(m_nTxBuffIn1 + ulCount >= TXBUFFSIZE1)
	{
		// No place to put new data
		// To be sure we purge all tx data
		IntDisable(INT_UART1);
		#if 0 // Sent all data buffer until empty and put new data in to the buffer for sent
		// Fill hardware FIFO
		while(m_nTxBuffIn1 > 0)
		{
			bRc = UARTCharPutNonBlocking(UART1_BASE,m_tTxBuff1[m_nTxNextNdx1]);
 			if(bRc == true)
			{
				m_nTxNextNdx1++;
				m_nTxBuffIn1--;
			}
				
		}
		memcpy(m_tTxBuff1 ,pucBuffer,ulCount);
		m_nTxNextNdx1 = 0;
		m_nTxBuffIn1 = ulCount;		
		#else // Throw all data buffer and put new data in to buffer for sent
		//memcpy(m_tTxBuff1 ,pucBuffer,ulCount);
		//m_nTxNextNdx1 = 0;
		//m_nTxBuffIn1 = ulCount;
		FCount++;
		// Fill hardware FIFO
		bRc = true;
		while(m_nTxBuffIn1 > 0 && bRc == true)
		{
			bRc = UARTCharPutNonBlocking(UART1_BASE,m_tTxBuff1[m_nTxNextNdx1]);
 			if(bRc == true)
			{
				m_nTxNextNdx1++;
				m_nTxBuffIn1--;
			}
				
		}
		#endif
		IntEnable(INT_UART1);
	}
	else if(ulCount > 0)
	{
		IntDisable(INT_UART1);
		if(m_nTxBuffIn1 > 0)
		{
			// Move TX data to start
			if(m_nTxNextNdx1 > 0)
			{
				memmove(m_tTxBuff1,&m_tTxBuff1[m_nTxNextNdx1],m_nTxBuffIn1);
			}
			m_nTxNextNdx1 = 0;
			// Add data to TXBuff
			memcpy(&m_tTxBuff1[m_nTxBuffIn1],pucBuffer,ulCount);
			m_nTxBuffIn1 += ulCount;
		}
		else
		{
			memcpy(m_tTxBuff1 ,pucBuffer,ulCount);
			m_nTxNextNdx1 = 0;
			m_nTxBuffIn1 = ulCount;
		}
		// Fill hardware FIFO
		bRc = true;
		while(m_nTxBuffIn1 > 0 && bRc == true)
		{
			bRc = UARTCharPutNonBlocking(UART1_BASE,m_tTxBuff1[m_nTxNextNdx1]);
 			if(bRc == true)
			{
				m_nTxNextNdx1++;
				m_nTxBuffIn1--;
			}
				
		}
 		IntEnable(INT_UART1);
	}
}
