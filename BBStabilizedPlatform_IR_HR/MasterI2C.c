/*

  Copyright Bluebird Aero Systems Engineering 2012

  $Id: MasterI2C.c,v 1.1 2011/03/15 12:23:45 EranS Exp $
  $Log: MasterI2C.c,v $
  Revision 1.1  2011/03/15 12:23:45  EranS
  First save

  Revision 1.2  2011/01/26 08:29:19  EranS

    Generic I2C interface
*/

#include "bluebird.h"
#include "inc/hw_ints.h"
#include "inc/lm4f230h5qr.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/debug.h"
#include "driverlib/gpio.h"
#include "driverlib/rom.h"
#include "driverlib/interrupt.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "driverlib/i2c.h"
#include "pins.h"
#include "reg.h"


#define MAXRETRIES              5           // number of receive attempts before giving up
 

//*****************************************************************************
//
// The states in the interrupt handler state machine.
//
//*****************************************************************************
enum I2Cstate { STATE_IDLE,		// 0
				 STATE_WRITE_NEXT,	// 1
				 STATE_WRITE_FINAL, // 2
				 STATE_WAIT_ACK,	// 3
  				 STATE_SEND_ACK,	// 4
  				 STATE_READ_ONE,	// 5
  				 STATE_READ_FIRST,	// 6
  				 STATE_READ_NEXT,	// 7
  				 STATE_READ_FINAL,	// 8
  				 STATE_READ_WAIT};	// 9

static volatile enum I2Cstate g_ulState = STATE_IDLE;
static unsigned char currentaddress;

//*****************************************************************************
//
// The variables that track the data to be transmitted or received.
//
//*****************************************************************************
static unsigned char *g_pucData = 0;
static unsigned long g_ulCount = 0;

static unsigned long tempc;


#if 0

static tBoolean WaitI2CFinished(void)
{
	// Wait until the current byte has been transferred.
	while(ROM_I2CMasterIntStatus(I2C_MASTER_BASE, false) == 0){}

	if(ROM_I2CMasterErr(I2C_MASTER_BASE) != I2C_MASTER_ERR_NONE)
	{
		ROM_I2CMasterIntClear(I2C_MASTER_BASE);
		return(false);
	}

	// Clear any interrupts set.
	while(ROM_I2CMasterIntStatus(I2C_MASTER_BASE, false))
	{
		ROM_I2CMasterIntClear(I2C_MASTER_BASE);
	}

	// Fixes I2C transactions.. ?
	ROM_SysCtlDelay(10000);

	return(true);
}

int I2CRead(unsigned char slave, unsigned char address, unsigned int len, unsigned char * readdata, unsigned int readcnt)
{
unsigned long ulToRead;

	// Start with a dummy write to get the address set in the EEPROM.
	ROM_I2CMasterSlaveAddrSet(I2C_MASTER_BASE, slave, false);

	// Place the address to be written in the data register.
	ROM_I2CMasterDataPut(I2C_MASTER_BASE, address);

	// Perform a single send, writing the address as the only byte.
	ROM_I2CMasterControl(I2C_MASTER_BASE, I2C_MASTER_CMD_BURST_SEND_START);

	// Wait until the current byte has been transferred.
	if(!WaitI2CFinished()) 
	{ 
		return(false); 
	}

	// Put the I2C master into receive mode.
	ROM_I2CMasterSlaveAddrSet(I2C_MASTER_BASE, slave, true);

	// Start the receive.
	ROM_I2CMasterControl(I2C_MASTER_BASE, ((readcnt > 1) ? I2C_MASTER_CMD_BURST_RECEIVE_START :
	I2C_MASTER_CMD_SINGLE_RECEIVE));

	// Receive the required number of bytes.
	ulToRead = readcnt;

	while(ulToRead)
	{
		// Wait until the current byte has been read.
		while(ROM_I2CMasterIntStatus(I2C_MASTER_BASE, false) == 0){}
		// Fixes I2C transactions.. ?
		ROM_SysCtlDelay(1000);

		// Clear pending interrupt notification.
		ROM_I2CMasterIntClear(I2C_MASTER_BASE);

		// Read the received character.
		*readdata++ = ROM_I2CMasterDataGet(I2C_MASTER_BASE);
		ulToRead--;

		// Set up for the next byte if any more remain.
		if(ulToRead)
		{
			ROM_I2CMasterControl(I2C_MASTER_BASE, ((ulToRead == 1) ? I2C_MASTER_CMD_BURST_RECEIVE_FINISH :
			I2C_MASTER_CMD_BURST_RECEIVE_CONT));
		}
	}

	return(readcnt - ulToRead);
}

#endif

void I2C_Init(void){
  volatile unsigned long delay;
  SYSCTL_RCGC1_R |= SYSCTL_RCGC1_I2C0;  // 1) activate I2C0
  SYSCTL_RCGC2_R |= SYSCTL_RCGC2_GPIOB; // 2) activate port B
  delay = SYSCTL_RCGC2_R;               // allow time to finish activating
  GPIO_PORTB_AFSEL_R |= 0x0C;           // 3) enable alt funct on PB2,3
  GPIO_PORTB_ODR_R |= 0x0C;             // 4) enable open drain on PB2,3
  GPIO_PORTB_DEN_R |= 0x0C;             // 5) enable digital I/O on PB2,3
  I2C0_MASTER_MTPR_R = 2;               // 6) configure for 100 kbps clock
  // 20*(TPR+1)*167ns = 10us, with TPR=2
  I2C0_MASTER_MCR_R = I2C_MCR_MFE;      // 7) master function enable
}

// receives one byte from specified slave
// Note for HMC6352 compass only:
// Used with 'r' and 'g' commands
// Note for TMP102 thermometer only:
// Used to read the top byte of the contents of the pointer register
//  This will work but is probably not what you want to do.
unsigned char I2C_Recv(char slave){
  int retryCounter = 1;
  do{
    while(I2C0_MASTER_MCS_R&I2C_MCS_BUSY){};// wait for I2C ready
    I2C0_MASTER_MSA_R = (slave<<1)&0xFE;    // MSA[7:1] is slave address
    I2C0_MASTER_MSA_R |= 0x01;              // MSA[0] is 1 for receive
    I2C0_MASTER_MCS_R = (0
                         & ~I2C_MCS_ACK     // negative data ack (last byte)
                         | I2C_MCS_STOP     // generate stop
                         | I2C_MCS_START    // generate start/restart
                         | I2C_MCS_RUN);    // master enable
    while(I2C0_MASTER_MCS_R&I2C_MCS_BUSY){};// wait for transmission done
    retryCounter = retryCounter + 1;        // increment retry counter
  }                                         // repeat if error
  while(((I2C0_MASTER_MCS_R&(I2C_MCS_ADRACK|I2C_MCS_ERROR)) != 0) && (retryCounter <= MAXRETRIES));
  return (I2C0_MASTER_MDR_R&0xFF);          // usually returns 0xFF on error
}

// receives two bytes from specified slave
// Note for HMC6352 compass only:
// Used with 'A' commands
// Note for TMP102 thermometer only:
// Used to read the contents of the pointer register
unsigned short I2C_Recv2(char slave){
  unsigned char data1,data2;
  int retryCounter = 1;
  do{
    while(I2C0_MASTER_MCS_R&I2C_MCS_BUSY){};// wait for I2C ready
    I2C0_MASTER_MSA_R = (slave<<1)&0xFE;    // MSA[7:1] is slave address
    I2C0_MASTER_MSA_R |= 0x01;              // MSA[0] is 1 for receive
    I2C0_MASTER_MCS_R = (0
                         | I2C_MCS_ACK      // positive data ack
                         & ~I2C_MCS_STOP    // no stop
                         | I2C_MCS_START    // generate start/restart
                         | I2C_MCS_RUN);    // master enable
    while(I2C0_MASTER_MCS_R&I2C_MCS_BUSY){};// wait for transmission done
    data1 = (I2C0_MASTER_MDR_R&0xFF);       // MSB data sent first
    I2C0_MASTER_MCS_R = (0
                         & ~I2C_MCS_ACK     // negative data ack (last byte)
                         | I2C_MCS_STOP     // generate stop
                         & ~I2C_MCS_START   // no start/restart
                         | I2C_MCS_RUN);    // master enable
    while(I2C0_MASTER_MCS_R&I2C_MCS_BUSY){};// wait for transmission done
    data2 = (I2C0_MASTER_MDR_R&0xFF);       // LSB data sent last
    retryCounter = retryCounter + 1;        // increment retry counter
  }                                         // repeat if error
  while(((I2C0_MASTER_MCS_R&(I2C_MCS_ADRACK|I2C_MCS_ERROR)) != 0) && (retryCounter <= MAXRETRIES));
  return (data1<<8)+data2;                  // usually returns 0xFFFF on error
}


// sends one byte to specified slave
// Note for HMC6352 compass only:
// Used with 'S', 'W', 'O', 'C', 'E', 'L', and 'A' commands
//  For 'A' commands, I2C_Recv2() should also be called
// Note for TMP102 thermometer only:
// Used to change the pointer register
// Returns 0 if successful, nonzero if error
unsigned long I2C_Send1(char slave, unsigned char data1){
  while(I2C0_MASTER_MCS_R&I2C_MCS_BUSY){};// wait for I2C ready
  I2C0_MASTER_MSA_R = (slave<<1)&0xFE;    // MSA[7:1] is slave address
  I2C0_MASTER_MSA_R &= ~0x01;             // MSA[0] is 0 for send
  I2C0_MASTER_MDR_R = data1&0xFF;         // prepare first byte
  I2C0_MASTER_MCS_R = (0
                       & ~I2C_MCS_ACK     // no data ack (no data on send)
                       | I2C_MCS_STOP     // generate stop
                       | I2C_MCS_START    // generate start/restart
                       | I2C_MCS_RUN);    // master enable
  while(I2C0_MASTER_MCS_R&I2C_MCS_BUSY){};// wait for transmission done
                                          // return error bits
  return (I2C0_MASTER_MCS_R&(I2C_MCS_DATACK|I2C_MCS_ADRACK|I2C_MCS_ERROR));
}

//*****************************************************************************
//
// The I2C interrupt handler.
//
//*****************************************************************************
// keep track of states
enum I2Cstate statearray[20];
unsigned stateindex = 0;
void I2C0IntHandler(void)
{
  statearray[stateindex++] = g_ulState;
  if (stateindex == 20)
    stateindex = 0;
    //
    // Clear the I2C interrupt.
    //
    ROM_I2CMasterIntClear(I2C0_MASTER_BASE);

    //
    // Determine what to do based on the current state.
    //
    switch(g_ulState)
    {
        //
        // The idle state.
        //
        case STATE_IDLE:
        {
            //
            // There is nothing to be done.
            //
            break;
        }

        //
        // The state for the middle of a burst write.
        //
        case STATE_WRITE_NEXT:
        {
            //
            // Write the next byte to the data register.
            //
            ROM_I2CMasterDataPut(I2C0_MASTER_BASE, *g_pucData++);
            g_ulCount--;

            //
            // Continue the burst write.
            //
            ROM_I2CMasterControl(I2C0_MASTER_BASE, I2C_MASTER_CMD_BURST_SEND_CONT);

            //
            // If there is one byte left, set the next state to the final write
            // state.
            //
            if(g_ulCount == 1)
            {
                g_ulState = STATE_WRITE_FINAL;
            }

            //
            // This state is done.
            //
            break;
        }

        //
        // The state for the final write of a burst sequence.
        //
        case STATE_WRITE_FINAL:
        {
            //
            // Write the final byte to the data register.
            //
            ROM_I2CMasterDataPut(I2C0_MASTER_BASE, *g_pucData++);
            g_ulCount--;

            //
            // Finish the burst write.
            //
            ROM_I2CMasterControl(I2C0_MASTER_BASE,
                             I2C_MASTER_CMD_BURST_SEND_FINISH);

            //
            // The next state is to wait for the burst write to complete.
            //
            g_ulState = STATE_SEND_ACK;

            //
            // This state is done.
            //
            break;
        }

        //
        // Wait for an ACK on the read after a write.
        //
        case STATE_WAIT_ACK:
        {
            //
            // See if there was an error on the previously issued read.
            //
            if(ROM_I2CMasterErr(I2C0_MASTER_BASE) == I2C_MASTER_ERR_NONE)
            {
                //
                // Read the byte received.
                //
                ROM_I2CMasterDataGet(I2C0_MASTER_BASE);

                //
                // There was no error, so the state machine is now idle.
                //
                g_ulState = STATE_IDLE;

                //
                // This state is done.
                //
                break;
            }
			else
			{
            	//
            	// Fall through to STATE_SEND_ACK.
            	//
            	//BBLedToggle();
			}
            	
        }

        //
        // Send a read request, looking for the ACK to indicate that the write
        // is done.
        //
        case STATE_SEND_ACK:
        {
            //
            // Put the I2C master into receive mode.
            //
            ROM_I2CMasterSlaveAddrSet(I2C0_MASTER_BASE, currentaddress, true);

            //
            // Perform a single byte read.
            //
            ROM_I2CMasterControl(I2C0_MASTER_BASE, I2C_MASTER_CMD_SINGLE_RECEIVE);

            //
            // The next state is the wait for the ack.
            //
            g_ulState = STATE_WAIT_ACK;

            //
            // This state is done.
            //
            break;
        }

        //
        // The state for a single byte read.
        //
        case STATE_READ_ONE:
        {
            //
            // Put the I2C master into receive mode.
            //
            ROM_I2CMasterSlaveAddrSet(I2C0_MASTER_BASE, currentaddress, true);

            //
            // Perform a single byte read.
            //
            ROM_I2CMasterControl(I2C0_MASTER_BASE, I2C_MASTER_CMD_SINGLE_RECEIVE);

            //
            // The next state is the wait for final read state.
            //
            g_ulState = STATE_READ_WAIT;

            //
            // This state is done.
            //
            break;
        }

        //
        // The state for the start of a burst read.
        //
        case STATE_READ_FIRST: // 6
        {
            //
            // Put the I2C master into receive mode.
            //
            ROM_I2CMasterSlaveAddrSet(I2C0_MASTER_BASE, currentaddress, true);

            //
            // Start the burst receive.
            //
            ROM_I2CMasterControl(I2C0_MASTER_BASE,
                             I2C_MASTER_CMD_BURST_RECEIVE_START);

            //
            // The next state is the middle of the burst read.
            //
            g_ulState = STATE_READ_NEXT; // 7

            //
            // This state is done.
            //
            break;
        }

        //
        // The state for the middle of a burst read.
        //
        case STATE_READ_NEXT: // 7
        {
            //
            // Read the received character.
            //
            *g_pucData++ = ROM_I2CMasterDataGet(I2C0_MASTER_BASE);
            g_ulCount--;

            //
            // Continue the burst read.
            //
            ROM_I2CMasterControl(I2C0_MASTER_BASE,
                             I2C_MASTER_CMD_BURST_RECEIVE_CONT);

            //
            // If there are two characters left to be read, make the next
            // state be the end of burst read state.
            //
            if(g_ulCount == 2)
            {
                g_ulState = STATE_READ_FINAL; // 8
            }

            //
            // This state is done.
            //
            break;
        }

        //
        // The state for the end of a burst read.
        //
        case STATE_READ_FINAL: // 8
        {
            //
            // Read the received character.
            //
            *g_pucData++ = ROM_I2CMasterDataGet(I2C0_MASTER_BASE);
            g_ulCount--;

            //
            // Finish the burst read.
            //
            ROM_I2CMasterControl(I2C0_MASTER_BASE,
                             I2C_MASTER_CMD_BURST_RECEIVE_FINISH);

            //
            // The next state is the wait for final read state.
            //
            g_ulState = STATE_READ_WAIT; // 9

            //
            // This state is done.
            //
            break;
        }

        //
        // This state is for the final read of a single or burst read.
        //
        case STATE_READ_WAIT: // 9
        {
            //
            // Read the received character.
            //
            tempc = ROM_I2CMasterDataGet(I2C0_MASTER_BASE);
            *g_pucData++ = (unsigned char)tempc;
            g_ulCount--;

            //
            // The state machine is now idle.
            //
            g_ulState = STATE_IDLE;

            //
            // This state is done.
            //
            break;
        }
    }
}

/*
  Read from any device
*/
void MasterI2C0Read(unsigned char address, unsigned char *pucData, unsigned long ulOffset,
          unsigned long ulCount)
{
    currentaddress = address;
    //
    // Save the data buffer to be read.
    //
    g_pucData = pucData;
    g_ulCount = ulCount;

    //
    // Set the next state of the interrupt state machine based on the number of
    // bytes to read.
    //
    if(ulCount == 1)
    {
        g_ulState = STATE_READ_ONE;
    }
    else
    {
        g_ulState = STATE_READ_FIRST;
    }
    //SysCtlDelay(100);   // copied from Osram16x16

    
    ROM_I2CMasterSlaveAddrSet(I2C0_MASTER_BASE, currentaddress, false);

    //
    // Place the address to be written in the data register.
    //
    ROM_I2CMasterDataPut(I2C0_MASTER_BASE, ulOffset);

    //
    // Perform a single send, writing the address as the only byte.
    //
    ROM_I2CMasterControl(I2C0_MASTER_BASE, I2C_MASTER_CMD_SINGLE_SEND);
	
	//DEBUG_TP6(1);

    //
    // Wait until the I2C interrupt state machine is idle.
    //
    while(g_ulState != STATE_IDLE)
    {
    }
	//DEBUG_TP6(0);
}

//*****************************************************************************
//
// Write to any device.
//
//*****************************************************************************
void MasterI2C0Write(unsigned char address,unsigned char *pucData, unsigned long ulOffset,
           unsigned long ulCount)
{
  currentaddress = address;
    //
    // Save the data buffer to be written.
    //
    g_pucData = pucData;
    g_ulCount = ulCount;

    //
    // Set the next state of the interrupt state machine based on the number of
    // bytes to write.
    //
    if(ulCount != 1)
    {
        g_ulState = STATE_WRITE_NEXT;
    }
    else
    {
        g_ulState = STATE_WRITE_FINAL;
    }

    //SysCtlDelay(204);   // copied from Osram16x16
   
    
    ROM_I2CMasterSlaveAddrSet(I2C0_MASTER_BASE, currentaddress, false);

    //
    // Place the address to be written in the data register.
    //
    ROM_I2CMasterDataPut(I2C0_MASTER_BASE, ulOffset);

    //
    // Start the burst cycle, writing the address as the first byte.
    //
    ROM_I2CMasterControl(I2C0_MASTER_BASE, I2C_MASTER_CMD_BURST_SEND_START);

    //
    // Wait until the I2C interrupt state machine is idle.
    //
    while(g_ulState != STATE_IDLE)
    {
    }
}


// Only master mode supported
void i2c_config(unsigned long i2c_hz)
{
    I2C0_MASTER_MCR_R = (1<<4);  // Master Mode Enable (datasheet says 0x20)

    // i2c_hz = cpu_hz / (20*(1+timer_prd))
    // extra term in numerator gives us correct rounding (round towards larger timer_prd)
    unsigned long timer_prd = (SysCtlClockGet() + 20*i2c_hz - 1) / (20*i2c_hz) - 1;
    I2C0_MASTER_MTPR_R |= 0x00000003;//timer_prd;
}

/*
    Set up I2C pins and clock slow/fast
    rate400 true = 400KHz, false 100KHz
*/
void MasterI2C0Init(int rate400)
{
    //I2CMasterEnable(I2C0_MASTER_BASE);  // causes fault
    //
    // Enable the I2C and GPIO port B blocks as they are needed by this driver.
    //
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C0);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);

    //
    // Configure the I2C SCL and SDA pins for I2C operation.
    //
	GPIOPinTypeI2CSCL(GPIO_PORTB_BASE, GPIO_PIN_2);
	GPIOPinTypeI2C(GPIO_PORTB_BASE, GPIO_PIN_3);	

	GPIOPinConfigure(GPIO_PB2_I2C0SCL);    
	GPIOPinConfigure(GPIO_PB3_I2C0SDA);
	
    //
    // Initialize the I2C master.
    //
    ROM_I2CMasterInitExpClk(I2C0_MASTER_BASE, SysCtlClockGet(), rate400);
    // Register interrupt handler
    // or we could just edit the startup.c file
    //I2CIntRegister(I2C0_MASTER_BASE,I2C0IntHandler);
    //
    // Enable the I2C interrupt.
    //
    ROM_IntEnable(INT_I2C0);   // already done via I2CIntRegister
    //
    // Enable the I2C master interrupt.
    //
    ROM_I2CMasterIntEnable(I2C0_MASTER_BASE);
}


/*
    Set up I2C pins and clock slow/fast
    rate400 true = 400KHz, false 100KHz
*/
void MasterI2C0Disable(int rate400)
{

    //
    // Enable the I2C interrupt.
    //
    ROM_IntDisable(INT_I2C0);   // already done via I2CIntRegister
    //
    // Enable the I2C master interrupt.
    //
    ROM_I2CMasterIntDisable(I2C0_MASTER_BASE);
}

