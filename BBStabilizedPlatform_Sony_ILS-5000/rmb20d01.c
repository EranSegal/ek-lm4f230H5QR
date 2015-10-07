/*
  $Id: rmb20d01.c,v 1.11 2011/08/03 07:45:41 EranS Exp $
  $Log: rmb20d01.c,v $
  Revision 1.11  2011/08/03 07:45:41  EranS
  Do reset of decoders at higher duty cycle

  Revision 1.10  2011/07/20 06:49:47  EranS
  changed ITG rate to 100
  encoder init ar higher PWM
  telemetry changed

  Revision 1.9  2011/07/14 07:19:48  EranS
  added script engine

  Revision 1.8  2011/05/11 07:39:16  EranS
  Integrate with BB library, stow/pilot buttons now work

  Revision 1.7  2011/05/03 06:46:07  EranS
  After Eran rewire - pay attention to encoder pins

  Revision 1.6  2011/04/17 08:44:11  EranS
  add proper reset of encoders via GPIO

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

  Revision 1.2  2011/01/26 08:29:19  EranS
*/
#include <string.h>
#include <stdio.h>
#include "bluebird.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_adc.h"
#include "driverlib/debug.h"
#include "driverlib/gpio.h"
#include "driverlib/qei.h"
#include "driverlib/pwm.h"
#include "driverlib/sysctl.h"
#include "driverlib/systick.h"
#include "driverlib/timer.h"
#include "driverlib/adc.h"
#include "driverlib/interrupt.h"
#include "driverlib/rom.h"
#include "inc/hw_ints.h" 
#include "inc/lm4f230h5qr.h"
#include "A3906.h"
#include "GPIOhandler.h"
#include "utilities.h"
#include "SysTick.h"
#define RMB20_MAIN
#include "rmb20d01.h"
#include "Pins.h"
#include "uart_echo.h"
#include "encoder.h"


//*****************************************************************************
//
// This function converts a current value into the corresponding ADC reading.
//
//*****************************************************************************
#define CURRENT_TO_ADC(ulC)     ((unsigned long)((((ulC) * 3361) - 1856) /    \
                                                 65536))

//*****************************************************************************
//
// This function converts an ADC reading value into current in Amperes, as an
// 8.8 fixed-point value.
//
//*****************************************************************************
#define ADC_TO_CURRENT(ulA)     ((unsigned long)((((ulA) * 4972) + 42018) /   \
                                                 256))

//*****************************************************************************
//
// Converts a bus voltage to an ADC reading.
//
//*****************************************************************************
#define VBUS_TO_ADC(v)          (((v) * 1024) / (18 * 256))

//*****************************************************************************
//
// Converts an ADC reading to a bus voltage.
//
//*****************************************************************************
#define ADC_TO_VBUS(a)          (((a) * 18 * 256) / 1024)

//*****************************************************************************
//
// The following defines which ADC channel control should be used for each
// kind of data item.  Basically it maps how the ADC channels are connected
// on the board.  This is a hardware configuration.
//
//*****************************************************************************
#define CHAN_USER0      ADC_CTL_CH0
#define CHAN_USER1      ADC_CTL_CH1
#define CHAN_USER2      ADC_CTL_CH2
#define CHAN_USER3      ADC_CTL_CH3
//*****************************************************************************
//
// The following maps the order that items are acquired and stored by the
// ADC sequencers.  Note that 16 samples are specified, using 2 of the
// 8 sample sequencers.  The current is sampled multiple times deliberately
// because that value tends to bounce around.  It is sampled multiple
// times and will be averaged.
//
//*****************************************************************************
unsigned long g_ulADCSeq[] =
{
    CHAN_USER0, CHAN_USER1, CHAN_USER2, CHAN_USER3,
};
#define NUM_ADC_CHANNELS (sizeof(g_ulADCSeq) / sizeof(unsigned long))

//*****************************************************************************
//
// A buffer to hold one set of ADC data that is acquired per sample time.
//
//*****************************************************************************
static unsigned long g_ulADCData[NUM_ADC_CHANNELS];

//*****************************************************************************
//
// The averaged winding current.
//
//*****************************************************************************
static unsigned short g_usCurrent = 0;

//*****************************************************************************
//
// The averaged ADC current.
//
//*****************************************************************************
unsigned long g_ulAverage = 0;
unsigned long g_ucOversampleCnt = 0;
unsigned long g_ulSum = 0;
unsigned long g_ucOversampleIdx = 0;
unsigned long g_ulSampleBuffer[32];

//*****************************************************************************
//
// The averaged winding current when the motor is not being driven.  This is
// used to cancel out any zero-current error that may be present (due to
// component inaccuracies).
//
//*****************************************************************************
static unsigned short g_usCurrentZero;


#if 1

#if 1
void
ADCIntHandler(void)
{
	//
	// Clear the interrupt
	//
	ADCIntClear(ADC0_BASE, 3);
	//
	// Check g_ucOversampleIdx to make sure that it is within range
	//
	if(g_ucOversampleIdx == 16)
	{
		g_ucOversampleIdx = 0;
	}
	//
	// Subtract the oldest value from the global sum
	//
	g_ulSum -= g_ulSampleBuffer[g_ucOversampleIdx];
	//
	// Replace the oldest value with the new sample value
	//
	g_ulSampleBuffer[g_ucOversampleIdx] = HWREG(ADC0_BASE + ADC_O_SSFIFO3);
	//
	// Add the new sample to the overall sum
	//
	g_ulSum += g_ulSampleBuffer[g_ucOversampleIdx];
	//
	// Increment g_ucOversampleIdx
	//
	g_ucOversampleIdx++;
	//
	// Get the averaged value from the sample buffer data
	//
	g_ulAverage = g_ulSum >> 4;
	
	new_adc = true;
	

	//
	// Placeholder for ADC processing code
	//
}
#else
void
ADCIntHandler(void)
{
	//
	// Clear the interrupt
	//
	ADCIntClear(ADC0_BASE, 3);

	//
	// Replace the oldest value with the new sample value
	//
	g_ulAverage = HWREG(ADC0_BASE + ADC_O_SSFIFO3);
	
	new_adc = true;
	

	//
	// Placeholder for ADC processing code
	//
}

#endif

/****************************************************************************
*
*   Initialize the ADC to collect sample from our 4 line sensors.
*   ADC0 channel 0 
*   ADC0 channel 1 
*
****************************************************************************/
unsigned long ulTimeSysClock;

void RMBD01Init(void)
{


  //
  // Configure the ADC for Pitch and Roll
  // sequence 3 means a single sample
  // sequence 1, 2 means up to 4 samples
  // sequence 0 means up to 8 samples
  // we use sequence 1
  //
  if (SysCtlPeripheralPresent(SYSCTL_PERIPH_ADC0))
  {

    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);	
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);


    //
    // Configure the pins to be used as analog inputs.
    //
    GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_2 | GPIO_PIN_3);

    //
    // Select the external reference for greatest accuracy.
    //
    //ADCReferenceSet(ADC0_BASE, ADC_REF_EXT_3V);

    //
    // Only for LM4F232 Apply workaround for erratum 6.1, in order to use the
    // external reference.
    //
    //SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    //HWREG(GPIO_PORTB_BASE + GPIO_O_AMSEL) |= GPIO_PIN_6;
	
    ADCSequenceConfigure(ADC0_BASE, 3, ADC_TRIGGER_TIMER, 0); // 1 captures up to 4 samples
    //ADCSequenceConfigure(ADC0_BASE, 0, ADC_TRIGGER_PROCESSOR, 0); // 1 captures up to 4 samples


    //ADCSequenceStepConfigure(ADC0_BASE, 0, 0, ENCODER_CHANNEL_PITCH);     // sample pitch
    //ADCSequenceStepConfigure(ADC0_BASE, 0, 1, ENCODER_CHANNEL_ROLL | ADC_CTL_IE | ADC_CTL_END);     // sample roll

    ADCHardwareOversampleConfigure(ADC0_BASE, 64);
    //ADCSoftwareOversampleConfigure(ADC0_BASE, 0, 8);	

	//ADCSoftwareOversampleStepConfigure(ADC0_BASE, 0, 0, (ADC_CTL_CH1 | ADC_CTL_IE | ADC_CTL_END));
	ADCSequenceStepConfigure(ADC0_BASE, 3, 0, ADC_CTL_IE | ADC_CTL_END | ADC_CTL_CH0);


	//ADCSoftwareOversampleStepConfigure(ADC0_BASE, 0, 0, (ADC_CTL_CH1 | ADC_CTL_IE | ADC_CTL_END));
    //ADCSequenceStepConfigure(ADC0_BASE, 0, 0, ADC_CTL_CH0);     // sample roll
    //ADCSequenceStepConfigure(ADC0_BASE, 0, 0, ADC_CTL_CH1 | ADC_CTL_IE | ADC_CTL_END);     // sample roll
	    
    //ADCIntRegister(ADC0_BASE,0,ADCIntHandler);

	//
	// Enable the sequencer and interrupt
	//
	ADCSequenceEnable(ADC0_BASE, 3);
	ADCIntEnable(ADC0_BASE, 3);
	IntEnable(INT_ADC3);
	//
	// Zero the oversample counter and the sum
	//
	g_ucOversampleCnt = 0;
	g_ulSum = 0;
	
    //ADCSequenceEnable(ADC0_BASE, 2);
    //ADCIntEnable(ADC0_BASE, 2);

	#if 0
    //
    // Enable the ADC sequencers
    //
    ADCSequenceEnable(ADC0_BASE, 0);

    //
    // Flush the ADC sequencers to be sure there is no lingering data.
    //
    ADCSequenceDataGet(ADC0_BASE, 0, PitchRollAmp);

    //
    // Enable ADC interrupts
    //
    ADCIntClear(ADC0_BASE, 0);
    ADCIntEnable(ADC0_BASE, 0);
    IntEnable(INT_ADC0SS0);

	#else
    //
    // Setup timer for ADC_TRIGGER_TIMER
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1);
	
    //
    // Configure the second timer to generate triggers to the ADC 
    //
    TimerConfigure(TIMER1_BASE, TIMER_CFG_32_BIT_PER);
    TimerLoadSet(TIMER1_BASE, TIMER_A, ROM_SysCtlClockGet() / 20000); // 50 us == 20K
    ulTimeSysClock = ROM_SysCtlClockGet()/20000;

  
    TimerControlStall(TIMER1_BASE, TIMER_A, true);
    TimerControlTrigger(TIMER1_BASE, TIMER_A, true);

    //
    // Enable the timers.
    //	
    TimerEnable(TIMER1_BASE, TIMER_A);
	#endif
	
    new_adc = false;
    //
  }
  //
  // Clear outstanding ADC interrupt and enable
  //
  //ADCIntClear(ADC0_BASE, 2);
  //ADCIntEnable(ADC0_BASE, 2);  
  //IntEnable(INT_ADC2);

  //
  // Set the zero current to indicate that the initial sampling has just
  // begun.
  //
  g_usCurrentZero = 0xffff;
} // InitADC

#else


//*****************************************************************************
//
// The interrupt handler for the ADC interrupt.
//
//*****************************************************************************
void ADCIntHandler(void)
{
    int rc;  
    //
    // Clear the ADC interrupt.
    //
    ADCIntClear(ADC0_BASE, 1);
	
    //
    // Read the data from the ADC.
    //
	//rc = ADCSequenceDataGet(ADC0_BASE, 1, &g_ulADCData[0]);    
    rc = ADCSequenceDataGet(ADC0_BASE, 1, PitchRollAmp);
    if (rc == 2)  // should have read 2 samples
      new_adc = true;
    else
      memset(PitchRollAmp,0xff,sizeof(PitchRollAmp)); // so we know of error            	
    //else
    //  memset(&g_ulADCData[0],0xff,sizeof(&g_ulADCData[0])); // so we know of error            
	
}
/****************************************************************************
*
*   Initialize the ADC to collect sample from our 4 line sensors.
*   ADC0 channel 0 
*   ADC0 channel 1 
*
****************************************************************************/

void RMBD01Init( void )
{

  //
  // Configure the ADC for Pitch and Roll
  // sequence 3 means a single sample
  // sequence 1, 2 means up to 4 samples
  // sequence 0 means up to 8 samples
  // we use sequence 1
  //
  if (SysCtlPeripheralPresent(SYSCTL_PERIPH_ADC0))
  {

    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
    ADCSequenceConfigure(ADC0_BASE, 1, ADC_TRIGGER_TIMER, 0); // 1 captures up to 4 samples

    //
    // Select the external reference for greatest accuracy.
    //
    ADCReferenceSet(ADC0_BASE, ADC_REF_EXT_3V);

    ADCSequenceStepConfigure(ADC0_BASE, 1, 0, ENCODER_CHANNEL_PITCH);     // sample pitch
    ADCSequenceStepConfigure(ADC0_BASE, 1, 1, ENCODER_CHANNEL_ROLL | ADC_CTL_IE | ADC_CTL_END);     // sample roll
    ADCIntRegister(ADC0_BASE,1,ADCIntHandler);
    ADCSequenceEnable(ADC0_BASE, 1);
    ADCIntEnable(ADC0_BASE, 1);


    //
    // Setup timer for ADC_TRIGGER_TIMER
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1);
	
    //
    // Configure the second timer to generate triggers to the ADC 
    //
    TimerConfigure(TIMER1_BASE, TIMER_CFG_32_BIT_PER);
    TimerLoadSet(TIMER1_BASE, TIMER_A, SysCtlClockGet() / 1000); // 2 ms
  
    TimerControlStall(TIMER1_BASE, TIMER_A, true);
    TimerControlTrigger(TIMER1_BASE, TIMER_A, true);

    //
    // Enable the timers.
    //	
    TimerEnable(TIMER1_BASE, TIMER_A);
    new_adc = false;
    //
  }
  //
  // Clear outstanding ADC interrupt and enable
  //
  ADCIntClear(ADC0_BASE, 1);
  IntEnable(INT_ADC0);
} // InitADC
#endif
