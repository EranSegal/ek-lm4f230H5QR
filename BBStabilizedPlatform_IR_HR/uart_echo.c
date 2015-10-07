
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
#include "inc/hw_uart.h"
#include "inc/lm4f230h5qr.h"
#include "driverlib/debug.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "driverlib/rom.h"
#include "driverlib/udma.h"
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


#define mainFIFO_SET				( 0x10 )

#define TXBUFFSIZE	50
#define TXBUFFSIZE1	255

char m_tTxBuff[TXBUFFSIZE];
char m_nTxBuffIn=0;
char m_nTxNextNdx=0;

char m_tTxBuff1[TXBUFFSIZE1];
char m_nTxBuffIn1=0;
char m_nTxNextNdx1=0;

char pData[200];                    // = "$GPRMC,220516,A,5133.82,N,00042.24,W,45.3,231.8,130694,004.2,W*70";

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
// The number of SysTick ticks per second used for the SysTick interrupt.
//
//*****************************************************************************
#define SYSTICKS_PER_SECOND     100

//*****************************************************************************
//
// The size of the memory transfer source and destination buffers (in words).
//
//*****************************************************************************
#define MEM_BUFFER_SIZE         1024

//*****************************************************************************
//
// The size of the UART transmit and receive buffers.  They do not need to be
// the same size.
//
//*****************************************************************************
#define UART_TXBUF_SIZE         256
#define UART_RXBUF_SIZE         256

//*****************************************************************************
//
// The source and destination buffers used for memory transfers.
//
//*****************************************************************************
static unsigned long g_ulSrcBuf[MEM_BUFFER_SIZE];
static unsigned long g_ulDstBuf[MEM_BUFFER_SIZE];

//*****************************************************************************
//
// The transmit and receive buffers used for the UART transfers.  There is one
// transmit buffer and a pair of recieve ping-pong buffers.
//
//*****************************************************************************
static unsigned char g_ucTxBuf[UART_TXBUF_SIZE];
static unsigned char g_ucRxBufA[UART_RXBUF_SIZE];
static unsigned char g_ucRxBufB[UART_RXBUF_SIZE];

//*****************************************************************************
//
// The count of uDMA errors.  This value is incremented by the uDMA error
// handler.
//
//*****************************************************************************
static unsigned long g_uluDMAErrCount = 0;

//*****************************************************************************
//
// The count of times the uDMA interrupt occurred but the uDMA transfer was not
// complete.  This should remain 0.
//
//*****************************************************************************
static unsigned long g_ulBadISR = 0;

//*****************************************************************************
//
// The count of UART buffers filled, one for each ping-pong buffer.
//
//*****************************************************************************
static unsigned long g_ulRxBufACount = 0;
static unsigned long g_ulRxBufBCount = 0;

//*****************************************************************************
//
// The count of memory uDMA transfer blocks.  This value is incremented by the
// uDMA interrupt handler whenever a memory block transfer is completed.
//
//*****************************************************************************
static unsigned long g_ulMemXferCount = 0;


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


#if 0
//*****************************************************************************
//
// The interrupt handler for uDMA errors.  This interrupt will occur if the
// uDMA encounters a bus error while trying to perform a transfer.  This
// handler just increments a counter if an error occurs.
//
//*****************************************************************************
void
uDMAErrorHandler(void)
{
    unsigned long ulStatus;

    //
    // Check for uDMA error bit
    //
    ulStatus = ROM_uDMAErrorStatusGet();

    //
    // If there is a uDMA error, then clear the error and increment
    // the error counter.
    //
    if(ulStatus)
    {
        ROM_uDMAErrorStatusClear();
        g_uluDMAErrCount++;
    }
}

//*****************************************************************************
//
// The interrupt handler for uDMA interrupts from the memory channel.  This
// interrupt will increment a counter, and then restart another memory
// transfer.
//
//*****************************************************************************
void
uDMAIntHandler(void)
{
    unsigned long ulMode;

    //
    // Check for the primary control structure to indicate complete.
    //
    ulMode = ROM_uDMAChannelModeGet(UDMA_CHANNEL_SW);
    if(ulMode == UDMA_MODE_STOP)
    {
        //
        // Increment the count of completed transfers.
        //
        g_ulMemXferCount++;

        //
        // Configure it for another transfer.
        //
        ROM_uDMAChannelTransferSet(UDMA_CHANNEL_SW, UDMA_MODE_AUTO,
                                   g_ulSrcBuf, g_ulDstBuf, MEM_BUFFER_SIZE);

        //
        // Initiate another transfer.
        //
        ROM_uDMAChannelEnable(UDMA_CHANNEL_SW);
        ROM_uDMAChannelRequest(UDMA_CHANNEL_SW);
    }

    //
    // If the channel is not stopped, then something is wrong.
    //
    else
    {
        g_ulBadISR++;
    }
}


//*****************************************************************************
//
// Initializes the uDMA software channel to perform a memory to memory uDMA
// transfer.
//
//*****************************************************************************
void
InitSWTransfer(void)
{
    unsigned int uIdx;

    //
    // Fill the source memory buffer with a simple incrementing pattern.
    //
    for(uIdx = 0; uIdx < MEM_BUFFER_SIZE; uIdx++)
    {
        g_ulSrcBuf[uIdx] = uIdx;
    }

    //
    // Enable interrupts from the uDMA software channel.
    //
    ROM_IntEnable(INT_UDMA);

    //
    // Put the attributes in a known state for the uDMA software channel.
    // These should already be disabled by default.
    //
    ROM_uDMAChannelAttributeDisable(UDMA_CHANNEL_SW,
                                    UDMA_ATTR_USEBURST | UDMA_ATTR_ALTSELECT |
                                    (UDMA_ATTR_HIGH_PRIORITY |
                                    UDMA_ATTR_REQMASK));

    //
    // Configure the control parameters for the SW channel.  The SW channel
    // will be used to transfer between two memory buffers, 32 bits at a time.
    // Therefore the data size is 32 bits, and the address increment is 32 bits
    // for both source and destination.  The arbitration size will be set to 8,
    // which causes the uDMA controller to rearbitrate after 8 items are
    // transferred.  This keeps this channel from hogging the uDMA controller
    // once the transfer is started, and allows other channels cycles if they
    // are higher priority.
    //
    ROM_uDMAChannelControlSet(UDMA_CHANNEL_SW | UDMA_PRI_SELECT,
                              UDMA_SIZE_32 | UDMA_SRC_INC_32 | UDMA_DST_INC_32 |
                              UDMA_ARB_8);

    //
    // Set up the transfer parameters for the software channel.  This will
    // configure the transfer buffers and the transfer size.  Auto mode must be
    // used for software transfers.
    //
    ROM_uDMAChannelTransferSet(UDMA_CHANNEL_SW | UDMA_PRI_SELECT,
                               UDMA_MODE_AUTO, g_ulSrcBuf, g_ulDstBuf,
                               MEM_BUFFER_SIZE);

    //
    // Now the software channel is primed to start a transfer.  The channel
    // must be enabled.  For software based transfers, a request must be
    // issued.  After this, the uDMA memory transfer begins.
    //
    ROM_uDMAChannelEnable(UDMA_CHANNEL_SW);
    ROM_uDMAChannelRequest(UDMA_CHANNEL_SW);
}


#endif

int InitUART(unsigned long ulBase, unsigned long ulUARTClk,unsigned long ulBaud, unsigned long ulConfig)
{
	unsigned int uIdx;
	#if 0
	if(ulBase == UART0_BASE)
	{
		
		
		//
		// Fill the TX buffer with a simple data pattern.
		//
		for(uIdx = 0; uIdx < UART_TXBUF_SIZE; uIdx++)
		{
			g_ucTxBuf[uIdx] = uIdx;
		}
		
		//
		// Enable the UART peripheral, and configure it to operate even if the CPU
		// is in sleep.
		//
		ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
		ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
		ROM_SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_UART0);

		/* Set GPIO A0 and A1 as peripheral function.  They are used to output the UART signals. */
		GPIODirModeSet( GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1, GPIO_DIR_MODE_HW );

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
		// Configure the UART communication parameters.
		//
		//ROM_UARTConfigSetExpClk(UART0_BASE, ROM_SysCtlClockGet(), 115200,
		//						UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
		//						UART_CONFIG_PAR_NONE);
		//
		// Configure the UART for 19,200, 8-N-1 operation. 
		// Mux Must to get EVEN PAR bin from the Engine
		//
		UARTConfigSetExpClk(UART0_BASE, SysCtlClockGet(), 19200,
							(UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
							 UART_CONFIG_PAR_EVEN));		
		//
		// Set both the TX and RX trigger thresholds to 4.	This will be used by
		// the uDMA controller to signal when more data should be transferred.	The
		// uDMA TX and RX channels will be configured so that it can transfer 4
		// bytes in a burst when the UART is ready to transfer more data.
		//
		ROM_UARTFIFOLevelSet(UART0_BASE, UART_FIFO_TX4_8, UART_FIFO_RX4_8);
		
		//
		// Enable the UART for operation, and enable the uDMA interface for both TX
		// and RX channels.
		//
		ROM_UARTEnable(UART0_BASE);
		ROM_UARTDMAEnable(UART0_BASE, UART_DMA_RX | UART_DMA_TX);
		
		//
		// This register write will set the UART to operate in loopback mode.  Any
		// data sent on the TX output will be received on the RX input.
		//
		HWREG(UART0_BASE + UART_O_CTL) |= UART_CTL_LBE;
		
		//
		// Enable the UART peripheral interrupts.  Note that no UART interrupts
		// were enabled, but the uDMA controller will cause an interrupt on the
		// UART interrupt signal when a uDMA transfer is complete.
		//
		ROM_IntEnable(INT_UART0);
		
		//
		// Put the attributes in a known state for the uDMA UART0RX channel.  These
		// should already be disabled by default.
		//
		ROM_uDMAChannelAttributeDisable(UDMA_CHANNEL_UART0RX,
										UDMA_ATTR_ALTSELECT | UDMA_ATTR_USEBURST |
										UDMA_ATTR_HIGH_PRIORITY |
										UDMA_ATTR_REQMASK);
		
		//
		// Configure the control parameters for the primary control structure for
		// the UART RX channel.  The primary contol structure is used for the "A"
		// part of the ping-pong receive.  The transfer data size is 8 bits, the
		// source address does not increment since it will be reading from a
		// register.  The destination address increment is byte 8-bit bytes.  The
		// arbitration size is set to 4 to match the RX FIFO trigger threshold.
		// The uDMA controller will use a 4 byte burst transfer if possible.  This
		// will be somewhat more effecient that single byte transfers.
		//
		ROM_uDMAChannelControlSet(UDMA_CHANNEL_UART0RX | UDMA_PRI_SELECT,
								  UDMA_SIZE_8 | UDMA_SRC_INC_NONE | UDMA_DST_INC_8 |
								  UDMA_ARB_4);
		
		//
		// Configure the control parameters for the alternate control structure for
		// the UART RX channel.  The alternate contol structure is used for the "B"
		// part of the ping-pong receive.  The configuration is identical to the
		// primary/A control structure.
		//
		ROM_uDMAChannelControlSet(UDMA_CHANNEL_UART0RX | UDMA_ALT_SELECT,
								  UDMA_SIZE_8 | UDMA_SRC_INC_NONE | UDMA_DST_INC_8 |
								  UDMA_ARB_4);
		
		//
		// Set up the transfer parameters for the UART RX primary control
		// structure.  The mode is set to ping-pong, the transfer source is the
		// UART data register, and the destination is the receive "A" buffer.  The
		// transfer size is set to match the size of the buffer.
		//
		ROM_uDMAChannelTransferSet(UDMA_CHANNEL_UART0RX | UDMA_PRI_SELECT,
								   UDMA_MODE_PINGPONG,
								   (void *)(UART0_BASE + UART_O_DR),
								   g_ucRxBufA, sizeof(g_ucRxBufA));
		
		//
		// Set up the transfer parameters for the UART RX alternate control
		// structure.  The mode is set to ping-pong, the transfer source is the
		// UART data register, and the destination is the receive "B" buffer.  The
		// transfer size is set to match the size of the buffer.
		//
		ROM_uDMAChannelTransferSet(UDMA_CHANNEL_UART0RX | UDMA_ALT_SELECT,
								   UDMA_MODE_PINGPONG,
								   (void *)(UART0_BASE + UART_O_DR),
								   g_ucRxBufB, sizeof(g_ucRxBufB));
		
		//
		// Put the attributes in a known state for the uDMA UART0TX channel.  These
		// should already be disabled by default.
		//
		ROM_uDMAChannelAttributeDisable(UDMA_CHANNEL_UART0TX,
										UDMA_ATTR_ALTSELECT |
										UDMA_ATTR_HIGH_PRIORITY |
										UDMA_ATTR_REQMASK);
		
		//
		// Set the USEBURST attribute for the uDMA UART TX channel.  This will
		// force the controller to always use a burst when transferring data from
		// the TX buffer to the UART.  This is somewhat more effecient bus usage
		// than the default which allows single or burst transfers.
		//
		ROM_uDMAChannelAttributeEnable(UDMA_CHANNEL_UART0TX, UDMA_ATTR_USEBURST);
		
		//
		// Configure the control parameters for the UART TX.  The uDMA UART TX
		// channel is used to transfer a block of data from a buffer to the UART.
		// The data size is 8 bits.  The source address increment is 8-bit bytes
		// since the data is coming from a buffer.	The destination increment is
		// none since the data is to be written to the UART data register.	The
		// arbitration size is set to 4, which matches the UART TX FIFO trigger
		// threshold.
		//
		ROM_uDMAChannelControlSet(UDMA_CHANNEL_UART0TX | UDMA_PRI_SELECT,
								  UDMA_SIZE_8 | UDMA_SRC_INC_8 | UDMA_DST_INC_NONE |
								  UDMA_ARB_4);
		
		//
		// Set up the transfer parameters for the uDMA UART TX channel.  This will
		// configure the transfer source and destination and the transfer size.
		// Basic mode is used because the peripheral is making the uDMA transfer
		// request.  The source is the TX buffer and the destination is the UART
		// data register.
		//
		ROM_uDMAChannelTransferSet(UDMA_CHANNEL_UART0TX | UDMA_PRI_SELECT,
								   UDMA_MODE_BASIC, g_ucTxBuf,
								   (void *)(UART0_BASE + UART_O_DR),
								   sizeof(g_ucTxBuf));
		
		//
		// Now both the uDMA UART TX and RX channels are primed to start a
		// transfer.  As soon as the channels are enabled, the peripheral will
		// issue a transfer request and the data transfers will begin.
		//
		ROM_uDMAChannelEnable(UDMA_CHANNEL_UART0RX);
		ROM_uDMAChannelEnable(UDMA_CHANNEL_UART0TX);

	}
	#else
	if(ulBase == UART0_BASE)
	{
		//
		// Enable the peripherals used by this example.
		//
		SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);		
		SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);		

		/* Set GPIO A0 and A1 as peripheral function.  They are used to output the UART signals. */
		GPIODirModeSet( GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1, GPIO_DIR_MODE_HW );

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
		UARTConfigSetExpClk(UART0_BASE, SysCtlClockGet(), 19200,
							(UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
							 UART_CONFIG_PAR_EVEN));
		//UARTConfigSetExpClk(UART0_BASE, SysCtlClockGet(), 19200,
		//					(UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
		//					 UART_CONFIG_PAR_NONE));		
		#else
		UARTConfigSetExpClk(UART0_BASE, SysCtlClockGet(), 115200,
							(UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
							 UART_CONFIG_PAR_NONE));
		#endif
		/* We don't want to use the fifo.  This is for test purposes to generate as many interrupts as possible. */
		HWREG( UART0_BASE + UART_O_LCR_H ) &= ~mainFIFO_SET;

		/* Enable Tx interrupts. */
		HWREG( UART0_BASE + UART_O_IM ) |= UART_INT_TX;
		HWREG( UART0_BASE + UART_O_IM ) |= UART_INT_RX;
		//IntPrioritySet( INT_UART0, configKERNEL_INTERRUPT_PRIORITY );
		IntEnable( INT_UART0 );
		
		//
		// Enable the UART interrupt.
		//
		//IntEnable(INT_UART0);
		//UARTFIFOLevelSet(UART0_BASE,UART_FIFO_TX1_8,UART_FIFO_RX1_8);
		//UARTIntRegister(UART0_BASE,UART0IntHandler);
		//UARTIntEnable(UART0_BASE, UART_INT_RX | UART_INT_TX | UART_INT_RT);

		return 0;

	}
	#endif
	else
	if(ulBase == UART1_BASE)
	{
	    //
	    // Enable the peripherals used by this example.
	    //
	    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);	    
	    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART1);		

		/* Set GPIO A0 and A1 as peripheral function.  They are used to output the UART signals. */
		GPIODirModeSet( GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1, GPIO_DIR_MODE_HW );

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

		UARTConfigSetExpClk(UART1_BASE, SysCtlClockGet(), 115200,
							(UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
							 UART_CONFIG_PAR_NONE));

		/* We don't want to use the fifo.  This is for test purposes to generate as many interrupts as possible. */
		HWREG( UART1_BASE + UART_O_LCR_H ) &= ~mainFIFO_SET;

		/* Enable Tx interrupts. */
		HWREG( UART1_BASE + UART_O_IM ) |= UART_INT_TX;
		HWREG( UART1_BASE + UART_O_IM ) |= UART_INT_RX;
		//IntPrioritySet( INT_UART0, configKERNEL_INTERRUPT_PRIORITY );
		IntEnable( INT_UART1 );

	    //
	    // Enable the UART interrupt.
	    //
		//IntEnable(INT_UART1);
		//UARTFIFOLevelSet(UART1_BASE,UART_FIFO_TX1_8,UART_FIFO_RX1_8);
		//UARTIntRegister(UART1_BASE,UART1IntHandler);
		//UARTIntEnable(UART1_BASE, UART_INT_RX | UART_INT_TX | UART_INT_RT);


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
			OmapMessageLoop(UART1_BASE,1);						
		}
	}
}


void UART0Send(unsigned char *pucBuffer, unsigned ulCount)
{
	tBoolean bRc;

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

    //
    // Loop while there are more characters to send.
    //
	if(m_nTxBuffIn1 + ulCount >= TXBUFFSIZE1)
	{
		// No place to put new data
		// To be sure we purge all tx data
		//IntDisable(INT_UART1);
		/* Start the Tx of the message on the UART. */
		UARTIntDisable( UART1_BASE, UART_INT_TX );		
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
		//IntEnable(INT_UART1);
		UARTIntEnable(UART1_BASE, UART_INT_TX);
	}

	else if(ulCount > 0)
	{
		//IntDisable(INT_UART1);
		/* Start the Tx of the message on the UART. */
		UARTIntDisable( UART1_BASE, UART_INT_TX );		
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
 		//IntEnable(INT_UART1);
		UARTIntEnable(UART1_BASE, UART_INT_TX);
	}
}
