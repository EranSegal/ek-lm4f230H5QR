/*
  Copyright BlueBird Aero Systems Engineering Ltd 2012

  $Id: Encoder.c,v 1.1 2012/07/14 07:19:48 EranS Exp $
  $Log: Encoder.c,v $

  Revision 1.1  2011/03/15 12:23:45  EranS
  First save

  Bluebird Stabilized Platform
*/
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/lm4f230h5qr.h"
#include "driverlib/gpio.h"
#include "driverlib/qei.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/pin_map.h"
#include "driverlib/systick.h"
#include "driverlib/interrupt.h"
#include "constants.h"
#include "encoder.h"
#include "pins.h"
#include "A3906.h"
#include "rmb20d01.h"


#define INIT_POS_123 (0)
#define MAX_POSITION 9999

extern unsigned long ulPeriod;

unsigned long g_ulminrollpos,g_ulmaxrollpos;
unsigned long g_ulminpitchpos,g_ulmaxpitchpos;

unsigned long g_ulqei0pos_left,g_ulqei0pos_right;
unsigned long g_ulqei1pos_left,g_ulqei1pos_right;

//*****************************************************************************
//
// The number of lines in the attached quadrature encoder.
//
//*****************************************************************************
static unsigned long g_ulEncoderLines = 0;
static unsigned long g_ulEncoder1Lines = 0;

//*****************************************************************************
//
// The time at which the previous encoder edge occurred.
//
//*****************************************************************************
static unsigned long g_ulEncoderPrevious;
static unsigned long g_ulEncoder1Previous;

//*****************************************************************************
//
// The number of system clocks between edges from the encoder.
//
//*****************************************************************************
static unsigned long g_ulEncoderClocks;
static unsigned long g_ulEncoder1Clocks;

//*****************************************************************************
//
// The number of ticks until the encoder (and therefore motor) is assumed to
// have stopped.
//
//*****************************************************************************
static unsigned short g_usEncoderCount;
static unsigned short g_usEncoder1Count;

static unsigned long EncoderCount = 0;
//static unsigned long Encoder1Count = 0;

extern volatile unsigned long g_ulTickCount;

static unsigned short g_usEncoderFlags;
static unsigned short g_usEncoder1Flags;

//***************************************************************************** 
//----------VARIABLES 
//***************************************************************************** 
unsigned long status2, status1; 
unsigned long status4, status3; 
 
//***************************************************************************** 
//----------VARIABLES 
//***************************************************************************** 
static unsigned long ulstatus;
static unsigned long ulstatus1;

unsigned long pultemp[10]; 
unsigned char uctemp[4]; 
unsigned long ulCurrent_Position; 

unsigned long pultemp1[10]; 
unsigned char uctemp1[4]; 
unsigned long ulCurrent_Position1; 
float g_ulqei0Currentdeg = 0;
static float g_ulqei1Currentdeg = 0;

void MotorBrakePositionSet(enum MOTOR motor,unsigned long point);
void MotorBrakePositionClean(enum MOTOR motor);
int MotorPositionError(enum MOTOR motor);

//*****************************************************************************
//
// Sets the H-bridge brake/coast configuration.
//
//*****************************************************************************
void
MotorBrakePositionSet(enum MOTOR motor,unsigned long point)
{
    //
    // Set the stow position point.  This will be applied on the
    //	
    if(motor == PITCH_MOTOR)
    {
		Motor[motor].stowpoint = point;
		HWREGBITH(&g_usEncoderFlags, ENCODER_FLAG_PITCH) = 1;
    }
	else
	{
		Motor[motor].stowpoint = point;
		HWREGBITH(&g_usEncoderFlags, ENCODER_FLAG_ROLL) = 1;
	}
}

//*****************************************************************************
//
// Gets the H-bridge brake/coast configuration.
//
//*****************************************************************************
void
MotorBrakePositionClean(enum MOTOR motor)
{
    //
    // Clean the stow/pilot status.
    //	
    if(motor == PITCH_MOTOR)
		HWREGBITH(&g_usEncoderFlags, ENCODER_FLAG_PITCH) = 0;
	else
		HWREGBITH(&g_usEncoderFlags, ENCODER_FLAG_ROLL) = 0;

}

//*****************************************************************************
//
// Gets the H-bridge brake/coast configuration.
//
//*****************************************************************************
int
MotorPositionError(enum MOTOR motor)
{
    //
    // Clean the stow/pilot status.
    //	
    if(motor == PITCH_MOTOR)
    {
		if(HWREGBITH(&g_usEncoderFlags, ENCODER_FLAG_EDGE_FUALSE) == 1)		
			return -1;			
    }
	else
	{
		if(HWREGBITH(&g_usEncoder1Flags, ENCODER_FLAG_EDGE_FUALSE) == 1)		
			return -1;
	}

	return 0;
}

/*
  Move motor until no more change in encoder value, which
  we assume to mean that the gimbal is at its extreme point.
  Reset the appropriate encoder
*/
unsigned long prevenc;
void EncoderInitReset(unsigned long ulBase,enum MOTOR motor,enum A3906Logic logic)
{
	  unsigned long start;
	  tBoolean encChanged;
	  start = g_ulTickCount;
	  // move until encoders show no change
	  encChanged = true;
	  prevenc = QEIPositionGet(ulBase);
	  GeneralPitchRoll(motor,logic,motor == PITCH_MOTOR ? ((ulPeriod*35)/100): ((ulPeriod*35)/100));	  
	  while(encChanged)
	  {
		while ((g_ulTickCount - start) < 1000);   // encoder samples at 500Hz, we check at 10Hz
		encChanged = (prevenc != QEIPositionGet(ulBase));
		prevenc = QEIPositionGet(ulBase);
		start = g_ulTickCount;
	  }
	  
	  GeneralPitchRoll(motor,A3906_BRAKE,ulPeriod);
	  
	  if(ulBase == QEI0_BASE) // PITCH_MOTOR
	  {
		  if(logic == A3906_FORWARD)
		  {
			  //
			  // Indicate that an full edge has been seen.
			  //

			  //Motor[motor].max_position = QEIPositionGet(QEI0_BASE);
			  //Motor[motor].avg_position = QEIPositionGet(QEI0_BASE)/2;
			  
		  	  if( QEIPositionGet(QEI0_BASE) < FULL_SCALE_PITCH_MOTOR)
			    HWREGBITH(&g_usEncoderFlags, ENCODER_FLAG_EDGE_FUALSE) = 1;
			  else
				  HWREGBITH(&g_usEncoderFlags, ENCODER_FLAG_EDGE_FUALSE) = 0;
			  				  
		  }
		  else
		  {
			  QEIPositionSet(ulBase, 0);
			  Motor[motor].min_position = 1000;			  
			  Motor[motor].max_position = 120000; 
			  Motor[motor].relative = 42100;
			  HWREGBITH(&g_usEncoderFlags, ENCODER_FLAG_EDGE_FUALSE) = 0;	  
		  }
	  }
	  else					//	ROLL_MOTOR
	  {
		  if(logic == A3906_FORWARD)
		  {
			  QEIPositionSet(ulBase, 0);
			  Motor[motor].min_position = 315500;			  
			  Motor[motor].max_position = 156000;
			  Motor[motor].relative = 236700;
			  
			  HWREGBITH(&g_usEncoder1Flags, ENCODER_FLAG_EDGE_FUALSE) = 0;
			  
		  }
		  else
		  {
			  //Motor[motor].max_position = QEIPositionGet(QEI1_BASE);							  
			  //Motor[motor].avg_position = (QEIPositionGet(QEI1_BASE)/2)+5500;	// Fix symmetry 							  

			  if( QEIPositionGet(QEI1_BASE) < FULL_SCALE_ROLL_MOTOR)
				HWREGBITH(&g_usEncoder1Flags, ENCODER_FLAG_EDGE_FUALSE) = 1;
			  else
				  HWREGBITH(&g_usEncoder1Flags, ENCODER_FLAG_EDGE_FUALSE) = 0;
			  
		  }
	
	  }

}



void EncoderInit2Point(enum MOTOR motor,enum A3906Logic driction,unsigned long point)
{

  unsigned long ulpos;  
  //tContext sContext;
  unsigned long ulBase = (motor == PITCH_MOTOR ? QEI0_BASE : QEI1_BASE);

  //
  // Save the time.
  //
  ulpos = QEIPositionGet(ulBase);
  
  if(ulpos > point)
	  GeneralPitchRoll(motor,A3906_REVERSE,((ulPeriod*25)/100));
  else		
  if(ulpos < point)	
	  GeneralPitchRoll(motor,A3906_FORWARD,((ulPeriod*25)/100));

  MotorBrakePositionSet(motor,point);
  
  //while(ulpos != point)
	  //ulpos = QEIPositionGet(ulBase);	  
    
  //GeneralPitchRoll(motor,A3906_BRAKE,99);
	
}


void EncoderWhile2Point(enum MOTOR motor,enum A3906Logic driction,unsigned long point)
{

  unsigned long ulpos;  
  //tContext sContext;
  unsigned long ulBase = (motor == PITCH_MOTOR ? QEI0_BASE : QEI1_BASE);

  //
  // Save the time.
  //
  ulpos = QEIPositionGet(ulBase);
  
  if(ulpos > point)
	  GeneralPitchRoll(motor,A3906_REVERSE,((ulPeriod*30)/100));
  else		
  if(ulpos < point)	
	  GeneralPitchRoll(motor,A3906_FORWARD,((ulPeriod*30)/100));

  //MotorBrakePositionSet(motor,point);
  
  while(ulpos != point)
  {
	ulpos = QEIPositionGet(ulBase);	  

	if((ulpos > point))
		GeneralPitchRoll(motor,A3906_REVERSE,((ulPeriod*30)/100));				
	else	  
	if((ulpos < point))			  
		GeneralPitchRoll(motor,A3906_FORWARD,((ulPeriod*30)/100));
  }
  
  GeneralPitchRoll(motor,A3906_BRAKE,ulPeriod);
  //QEIPositionSet(ulBase, 0);
  
}


//*****************************************************************************
//
// This function is called to handle the GPIO edge interrupt from the
// quadrature encoder.
//
//*****************************************************************************
void
QEI0IntHandler(void)
{
    unsigned long ulNow;

    //
    // Save the time.
    //
    ulNow = SysTickValueGet();

    //
    // Clear the encoder interrupt.
    //
    GPIOPinIntClear(QEI_PITCH_PHA_PORT, QEI_PITCH_PHA_PIN);

	ulstatus = QEIIntStatus(QEI0_BASE,false); 
	if (  (ulstatus & QEI_INTDIR) == QEI_INTDIR) 
	{ 
	  // 
	  // clear	Interrupt Bit	 
	   QEIIntClear(QEI0_BASE, QEI_INTDIR); 
	  // 
	  //code for Direction change 
	  //.............. 
	} 
	if (  (ulstatus & QEI_INTINDEX) == QEI_INTINDEX) 
	{ 
	  // 
	  // clear	Interrupt Bit	 
	   QEIIntClear(QEI0_BASE, QEI_INTINDEX); 
	  // 
	  //code for Index change 
	  //.............. 
	}		
	if (  (ulstatus & QEI_INTERROR) == QEI_INTERROR) 
	{ 
	  //	 
	  // clear	Interrupt Bit	 
		QEIIntClear(QEI0_BASE, QEI_INTERROR); 
	  // 
	  //code for Phase ERROR 
	  //.............. 
	} 

    //
    // Determine the number of system clocks between the previous edge and this
    // edge.
    //
    if(g_ulEncoderPrevious > ulNow)
    {
        g_ulEncoderClocks = g_ulEncoderPrevious - ulNow;
    }
    else
    {
        g_ulEncoderClocks = (SYSCLK_50MHZ - ulNow) + g_ulEncoderPrevious;		
		
    }

    //
    // Save the time of the current edge as the time of the previous edge.
    //
    g_ulEncoderPrevious = ulNow;

    //
    // Indicate that an edge has been seen.
    //
    HWREGBITH(&g_usEncoderFlags, ENCODER_FLAG_EDGE) = 1;

    //
    // If the previous edge time was valid, then indicate that the time between
    // edges is also now valid.
    //
    if(HWREGBITH(&g_usEncoderFlags, ENCODER_FLAG_PREVIOUS) == 1)
    {
        HWREGBITH(&g_usEncoderFlags, ENCODER_FLAG_VALID) = 1;
    }

    //
    // Indicate that the previous edge time is valid.
    //
    HWREGBITH(&g_usEncoderFlags, ENCODER_FLAG_PREVIOUS) = 1;
}

//*****************************************************************************
//
// This function is called to handle the GPIO edge interrupt from the
// quadrature encoder.
//
//*****************************************************************************
void
QEI1IntHandler(void)
{
    unsigned long ulNow;

    //
    // Save the time.
    //
    ulNow = SysTickValueGet();

    //
    // Clear the encoder interrupt.
    //
    GPIOPinIntClear(QEI_ROLL_PHA_PORT, QEI_ROLL_PHA_PIN);

	ulstatus1 = QEIIntStatus(QEI1_BASE,false); 
	if (  (ulstatus1 & QEI_INTDIR) == QEI_INTDIR) 
	{ 
	  // 
	  // clear	Interrupt Bit	 
	   QEIIntClear(QEI1_BASE, QEI_INTDIR); 
	  // 
	  //code for Direction change 
	  //.............. 
	} 
	if (  (ulstatus1 & QEI_INTINDEX) == QEI_INTINDEX) 
	{ 
	  // 
	  // clear	Interrupt Bit	 
	   QEIIntClear(QEI1_BASE, QEI_INTINDEX); 
	  // 
	  //code for Index change 
	  //.............. 
	}		
	if (  (ulstatus1 & QEI_INTERROR) == QEI_INTERROR) 
	{ 
	  //	 
	  // clear	Interrupt Bit	 
		QEIIntClear(QEI1_BASE, QEI_INTERROR); 
	  // 
	  //code for Phase ERROR 
	  //.............. 
	} 

    //
    // Determine the number of system clocks between the previous edge and this
    // edge.
    //
    if(g_ulEncoder1Previous > ulNow)
    {
        g_ulEncoder1Clocks = g_ulEncoder1Previous - ulNow;
    }
    else
    {
        g_ulEncoder1Clocks = (SYSCLK_50MHZ - ulNow) + g_ulEncoder1Previous;		
		
    }

    //
    // Save the time of the current edge as the time of the previous edge.
    //
    g_ulEncoder1Previous = ulNow;

    //
    // Indicate that an edge has been seen.
    //
    HWREGBITH(&g_usEncoder1Flags, ENCODER_FLAG_EDGE) = 1;

    //
    // If the previous edge time was valid, then indicate that the time between
    // edges is also now valid.
    //
    if(HWREGBITH(&g_usEncoder1Flags, ENCODER_FLAG_PREVIOUS) == 1)
    {
        HWREGBITH(&g_usEncoder1Flags, ENCODER_FLAG_VALID) = 1;
    }

    //
    // Indicate that the previous edge time is valid.
    //
    HWREGBITH(&g_usEncoder1Flags, ENCODER_FLAG_PREVIOUS) = 1;
}

//*****************************************************************************
//
// This function prepares the quadrature encoder module for capturing the
// position and speed of the motor.
//
//*****************************************************************************
void
Encoder_Init(unsigned long ulBase)
{

	if(ulBase == QEI0_BASE) // J6 PITCH Encoder 
	{
		//
		// Enable the peripherals QEI example.
		//	
		SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
		SysCtlPeripheralEnable(SYSCTL_PERIPH_QEI0); 
	
	    // unlock GPIO PD7
	    GPIO_PORTD_LOCK_R = GPIO_LOCK_KEY;
		GPIO_PORTD_CR_R = GPIO_PIN_7;

		//Enable GPIO for QEI0
		//PD3-> IDX0 				
		//PD6-> PHA0
		//PD7-> PHB0 
		GPIOPinConfigure(GPIO_PD3_IDX0);		
		GPIOPinConfigure(GPIO_PD6_PHA0);	
		GPIOPinConfigure(GPIO_PD7_PHB0);

		GPIOPinTypeQEI(QEI_PITCH_PHA_PORT,QEI_PITCH_PHA_PIN); 
		GPIOPinTypeQEI(QEI_PITCH_PHB_PORT,QEI_PITCH_PHB_PIN);
		GPIOPinTypeQEI(QEI_PITCH_INDEX_PORT,QEI_PITCH_INDEX_PIN);
		
		
		// Set D 0 and 1 to alternative use 
		GPIODirModeSet(GPIO_PORTD_BASE, GPIO_PIN_3, GPIO_DIR_MODE_HW);//GPIO_DIR_MODE_HW	
		GPIODirModeSet(GPIO_PORTD_BASE, GPIO_PIN_6, GPIO_DIR_MODE_HW); 
		GPIODirModeSet(GPIO_PORTD_BASE, GPIO_PIN_7, GPIO_DIR_MODE_HW); 
		

		//
		// Configure the QEI module.
		//
		QEIConfigure(QEI0_BASE,
						 (QEI_CONFIG_NO_RESET | QEI_CONFIG_CAPTURE_A_B |
						  QEI_CONFIG_QUADRATURE | QEI_CONFIG_NO_SWAP), QEI0_POSCNT_MAX-1);
		
		
		//Set to 0 
		QEIPositionSet(QEI0_BASE, 0);

		QEIVelocityConfigure(QEI0_BASE, QEI_VELDIV_1, 500000); //Configure the Velocity capture Module //500000 is 10ms at 50MHz 

		QEIVelocityEnable(QEI0_BASE);	//Enable the Velocity capture Module	
		
		QEIIntEnable(QEI0_BASE, QEI_INTDIR | QEI_INTINDEX);   //Enable Interrupt when the Timer is reach 0 on Valocity capture mode 
		
		QEIEnable(QEI0_BASE);
				
		//
		// Configure the encoder input to generate an interrupt on every rising
		// edge.
		//
		//GPIOIntTypeSet(QEI_PITCH_PHA_PORT, QEI_PITCH_PHA_PIN, GPIO_RISING_EDGE);
		//GPIOPinIntEnable(QEI_PITCH_PHA_PORT, QEI_PITCH_PHA_PIN);
		//IntEnable(QEI_PITCH_PHA_INT);			

		//Interrupt Enable 
		//IntEnable(INT_QEI1); 

			
	}
	else
	if(ulBase == QEI1_BASE)		// J8 ROLL Encoder 
	{

		//
		// Enable the peripherals QEI example.
		//	
		SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
		SysCtlPeripheralEnable(SYSCTL_PERIPH_QEI1); 

		//Enable GPIO for QEI1 
		//PC4-> IDX0 				
		//PC5-> PHA0
		//PC6-> PHB0 
		GPIOPinConfigure(GPIO_PC4_IDX1);		
		GPIOPinConfigure(GPIO_PC5_PHA1);	
		GPIOPinConfigure(GPIO_PC6_PHB1);		
			
		GPIOPinTypeQEI(QEI_ROLL_PHA_PORT,QEI_ROLL_PHA_PIN); 
		GPIOPinTypeQEI(QEI_ROLL_PHB_PORT,QEI_ROLL_PHB_PIN);
		GPIOPinTypeQEI(QEI_ROLL_INDEX_PORT,QEI_ROLL_INDEX_PIN);
		
		
		// Set F 0 and 1 to alternative use 
		GPIODirModeSet(GPIO_PORTC_BASE, GPIO_PIN_4, GPIO_DIR_MODE_HW);//GPIO_DIR_MODE_HW	
		GPIODirModeSet(GPIO_PORTC_BASE, GPIO_PIN_5, GPIO_DIR_MODE_HW); 
		GPIODirModeSet(GPIO_PORTC_BASE, GPIO_PIN_6, GPIO_DIR_MODE_HW); 
		

		//
		// Configure the QEI module.
		//
		QEIConfigure(QEI1_BASE,
						 (QEI_CONFIG_NO_RESET | QEI_CONFIG_CAPTURE_A_B |
						  QEI_CONFIG_QUADRATURE | QEI_CONFIG_SWAP), QEI1_POSCNT_MAX-1);
		
		
		//Set to 0 
		QEIPositionSet(QEI1_BASE, 0);

		QEIVelocityConfigure(QEI1_BASE, QEI_VELDIV_1, 500000); //Configure the Velocity capture Module //500000 is 10ms at 50MHz 

		QEIVelocityEnable(QEI1_BASE);	//Enable the Velocity capture Module	
		
		QEIIntEnable(QEI1_BASE, QEI_INTDIR | QEI_INTINDEX);   //Enable Interrupt when the Timer is reach 0 on Valocity capture mode 
		
		QEIEnable(QEI1_BASE);
				
		//
		// Configure the encoder input to generate an interrupt on every rising
		// edge.
		//
		//GPIOIntTypeSet(QEI_ROLL_PHA_PORT, QEI_ROLL_PHA_PIN, GPIO_RISING_EDGE);
		//GPIOPinIntEnable(QEI_ROLL_PHA_PORT, QEI_ROLL_PHA_PIN);
		//IntEnable(QEI_ROLL_PHA_INT);			

		//Interrupt Enable 
		//IntEnable(INT_QEI1); 
		



	}
}

//*****************************************************************************
//
// This function is called periodically to determine when the encoder has
// stopped rotating (based on too much time passing between edges).
//
//*****************************************************************************
void
EncoderTick(unsigned long ulBase)
{
	if(ulBase == QEI0_BASE)
	{
		#if 0
	    //
	    // See if an edge has been seen since the last call.
	    //
	    if(HWREGBITH(&g_usEncoderFlags, ENCODER_FLAG_EDGE) == 1)
	    {
	        //
	        // Clear the edge flag.
	        //
	        HWREGBITH(&g_usEncoderFlags, ENCODER_FLAG_EDGE) = 0;

	        //
	        // Reset the delay counter.
	        //
	        g_usEncoderCount = ENCODER_WAIT_TIME;
			
	    }
	    //
	    // Otherwise, see if the delay counter is still active.
	    //
	    else if(g_usEncoderCount != 0)
	    {
	        //
	        // Decrement the delay counter.
	        //
	        g_usEncoderCount--;

	        //
	        // If the delay counter has reached zero, then indicate that there are
	        // no valid speed values from the encoder.
	        //
	        if(g_usEncoderCount == 0)
	        {
	            HWREGBITH(&g_usEncoderFlags, ENCODER_FLAG_PREVIOUS) = 0;
	            HWREGBITH(&g_usEncoderFlags, ENCODER_FLAG_VALID) = 0;
	        }
	    }
		#endif
	
		Motor[PITCH_MOTOR].position = QEIPositionGet(QEI0_BASE);
		Motor[PITCH_MOTOR].direction= QEIDirectionGet(QEI0_BASE);
		//Motor[PITCH_MOTOR].aangle = Motor[PITCH_MOTOR].position / 924.444f/2.0f; // 360
		Motor[PITCH_MOTOR].aangle= ((Motor[PITCH_MOTOR].position - Motor[PITCH_MOTOR].zero_pos) / 924.444f);	// 180
		//Motor[PITCH_MOTOR].aangle= Motor[PITCH_MOTOR].position / 1079.466f/2.0f;

		//Motor[PITCH_MOTOR].deg2sec = (Motor[PITCH_MOTOR].aangle - Motor[PITCH_MOTOR].cur_angle)*Motor[PITCH_MOTOR].direction;
		//Motor[PITCH_MOTOR].cur_angle = Motor[PITCH_MOTOR].aangle;

		//
		// If the previous edge time was valid, then indicate that the time between
		// edges is also now valid.
		//
		if(HWREGBITH(&g_usEncoderFlags, ENCODER_FLAG_PITCH) == 1)
		{
			if(QEIPositionGet(QEI0_BASE) == Motor[PITCH_MOTOR].stowpoint)   
			{
				GeneralPitchRoll(PITCH_MOTOR,A3906_BRAKE,ulPeriod);	 
				HWREGBITH(&g_usEncoderFlags, ENCODER_FLAG_PITCH) = 0;
			}
			else
			if((QEIPositionGet(QEI0_BASE) > Motor[PITCH_MOTOR].stowpoint))
				GeneralPitchRoll(PITCH_MOTOR,A3906_REVERSE,((ulPeriod*25)/100));				
			else	  
			if((QEIPositionGet(QEI0_BASE) < Motor[PITCH_MOTOR].stowpoint))			  
				GeneralPitchRoll(PITCH_MOTOR,A3906_FORWARD,((ulPeriod*25)/100));
							
			
						
		}
		
		
	}
	else
	if(ulBase == QEI1_BASE)
	{
		#if 0
		//
		// See if an edge has been seen since the last call.
		//
		if(HWREGBITH(&g_usEncoder1Flags, ENCODER_FLAG_EDGE) == 1)
		{
			//
			// Clear the edge flag.
			//
			HWREGBITH(&g_usEncoder1Flags, ENCODER_FLAG_EDGE) = 0;
		
			//
			// Reset the delay counter.
			//
			g_usEncoder1Count = ENCODER_WAIT_TIME;

		}
		
		//
		// Otherwise, see if the delay counter is still active.
		//
		else if(g_usEncoder1Count != 0)
		{
			//
			// Decrement the delay counter.
			//
			g_usEncoder1Count--;
		
			//
			// If the delay counter has reached zero, then indicate that there are
			// no valid speed values from the encoder.
			//
			if(g_usEncoder1Count == 0)
			{
				HWREGBITH(&g_usEncoder1Flags, ENCODER_FLAG_PREVIOUS) = 0;
				HWREGBITH(&g_usEncoder1Flags, ENCODER_FLAG_VALID) = 0;
			}
		}
		#endif
		
		Motor[ROLL_MOTOR].position = QEIPositionGet(QEI1_BASE);
		Motor[ROLL_MOTOR].direction= QEIDirectionGet(QEI1_BASE);
		//Motor[ROLL_MOTOR].aangle= Motor[ROLL_MOTOR].position / 924.444f/2.0f;	// 360
		Motor[ROLL_MOTOR].aangle= ((Motor[ROLL_MOTOR].position - Motor[ROLL_MOTOR].zero_pos) / 924.444f);	// 180
		//Motor[ROLL_MOTOR].aangle= Motor[ROLL_MOTOR].position / 1079.466f/2.0f;

		//Motor[ROLL_MOTOR].deg2sec = (Motor[ROLL_MOTOR].aangle - Motor[ROLL_MOTOR].cur_angle)*Motor[ROLL_MOTOR].direction;
		//Motor[ROLL_MOTOR].cur_angle = Motor[ROLL_MOTOR].aangle;
		
		//
		// If the previous edge time was valid, then indicate that the time between
		// edges is also now valid.
		//
		if(HWREGBITH(&g_usEncoderFlags, ENCODER_FLAG_ROLL) == 1)
		{
			if(QEIPositionGet(QEI1_BASE) == Motor[ROLL_MOTOR].stowpoint)	  
			{
				GeneralPitchRoll(ROLL_MOTOR,A3906_BRAKE,ulPeriod);	
				HWREGBITH(&g_usEncoderFlags, ENCODER_FLAG_ROLL) = 0;
			}
			else
			if((QEIPositionGet(QEI1_BASE) < Motor[ROLL_MOTOR].stowpoint))
				GeneralPitchRoll(ROLL_MOTOR,A3906_FORWARD,((ulPeriod*25)/100));				
			else	  
			if((QEIPositionGet(QEI1_BASE) > Motor[ROLL_MOTOR].stowpoint))			  
				GeneralPitchRoll(ROLL_MOTOR,A3906_REVERSE,((ulPeriod*25)/100));				
			
		}			

	}
}

//*****************************************************************************
//
// This function sets the number of lines in the attached encoder.
//
//*****************************************************************************
void
EncoderLinesSet(unsigned long ulBase,unsigned long ulLines)
{
    //
    // Save the number of lines in the encoder.
    //
    if(ulBase == QEI0_BASE)
	    g_ulEncoderLines = ulLines;
	else
	if(ulBase == QEI1_BASE)
		g_ulEncoder1Lines = ulLines;
		
}

//*****************************************************************************
//
// This function gets the number of lines in the attached encoder.
//
//*****************************************************************************
unsigned long
EncoderLinesGet(void)
{
    //
    // Return the number of lines in the encoder.
    //
    return(g_ulEncoderLines);
}

//*****************************************************************************
//
// This function ``sets'' the position of the encoder.  This is the position
// against which all further movements of the encoder are measured in a
// relative sense.
//
//*****************************************************************************
void
EncoderPositionSet(long lPosition)
{
    //
    // Convert the position into the number of encoder lines.
    //
    //lPosition = MathMul16x16(lPosition, g_ulEncoderLines * 4);
	lPosition = lPosition*( g_ulEncoderLines * 4);

    //
    // Set the encoder position in the quadrature encoder module.
    //
    QEIPositionSet(QEI0_BASE, lPosition);
}

//*****************************************************************************
//
// This function ``sets'' the position of the encoder.  This is the position
// against which all further movements of the encoder are measured in a
// relative sense.
//
//*****************************************************************************
void
Encoder1PositionSet(long lPosition)
{
    //
    // Convert the position into the number of encoder lines.
    //
    //lPosition = MathMul16x16(lPosition, g_ulEncoderLines * 4);
	lPosition = lPosition*( g_ulEncoder1Lines * 4);

    //
    // Set the encoder position in the quadrature encoder module.
    //
    QEIPositionSet(QEI1_BASE, lPosition);
}

//*****************************************************************************
//
// Gets the current position of the encoder, specified as a signed 16.16 fixed-
// point value that represents a number of full revolutions.
//
//*****************************************************************************
float
EncoderPositionGet(unsigned long ulBase)
{
    //
    // Convert the encoder position to a number of revolutions and return it.
    //
    //return(MathDiv16x16(QEIPositionGet(QEI0_BASE), g_ulEncoderLines * 4));
    return(QEIPositionGet(ulBase)/ (g_ulEncoderLines * 4));
	
}

//*****************************************************************************
//
// Gets the current position of the encoder, specified as a signed 16.16 fixed-
// point value that represents a number of full revolutions.
//
//*****************************************************************************
float
Encoder1PositionGet(void)
{
    //
    // Convert the encoder position to a number of revolutions and return it.
    //
    //return(MathDiv16x16(QEIPositionGet(QEI0_BASE), g_ulEncoderLines * 4));
    return(QEIPositionGet(QEI1_BASE)/ (g_ulEncoder1Lines * 4));
	
}

//*****************************************************************************
//
// Gets the current position of the encoder, return
// angle value that represents a number of full revolutions.
//
//*****************************************************************************
float
EncoderDegPositionGet(unsigned long ulBase)
{
    //
    // Convert the encoder position to a degrees of revolutions and return it.
    //
    return( QEIPositionGet(ulBase)/ 462.222f);	
}

#if 1
//*****************************************************************************
//
// Gets the current speed of the encoder, specified as an unsigned 16.16 fixed-
// point value that represents the speed in revolutions per minute (RPM).
//
//*****************************************************************************
long
EncoderVelocityGet(long lSigned)
{
    //
    // If the time between edges is not valid, then the speed is zero.
    //
    if(HWREGBITH(&g_usEncoderFlags, ENCODER_FLAG_VALID) == 0)
    {
        return(0);
    }

    //
    // Convert the time between edges into a speed in RPM and return it.
    //
    //return(MathDiv16x16(SYSCLK * 60, g_ulEncoderClocks * g_ulEncoderLines));
	return( (SYSCLK * 60) / (g_ulEncoderClocks * g_ulEncoderLines));
}

//*****************************************************************************
//
// Gets the current speed of the encoder, specified as an unsigned 16.16 fixed-
// point value that represents the speed in revolutions per minute (RPM).
//
//*****************************************************************************
long
Encoder1VelocityGet(long lSigned)
{
    //
    // If the time between edges is not valid, then the speed is zero.
    //
    if(HWREGBITH(&g_usEncoder1Flags, ENCODER_FLAG_VALID) == 0)
    {
        return(0);
    }

    //
    // Convert the time between edges into a speed in RPM and return it.
    //
    //return(MathDiv16x16(SYSCLK * 60, g_ulEncoderClocks * g_ulEncoderLines));
	return( (SYSCLK * 60) / (g_ulEncoder1Clocks * g_ulEncoder1Lines));
}

#else
//*****************************************************************************
//
// Gets the current speed of the encoder, specified as an unsigned 16.16 fixed-
// point value that represents the speed in revolutions per minute (RPM).
//
//*****************************************************************************
long
EncoderVelocityGet(long lSigned)
{
    long lDir;

    //
    // If the time between edges is not valid, then the speed is zero.
    //
    if(HWREGBITH(&g_usEncoderFlags, ENCODER_FLAG_VALID) == 0)
    {
        return(0);
    }

    //
    // See if a signed velocity should be returned.
    //
    if(lSigned)
    {
        //
        // Get the current direction from the QEI module.
        //
        lDir = QEIDirectionGet(QEI0_BASE);
    }
    else
    {
        //
        // An unsigned velocity is requested, so assume the direction is
        // positive.
        //
        lDir = 1;
    }

    //
    // Convert the time between edges into a speed in RPM and return it.
    //
	return( (lDir * SYSCLK * 60) / (g_ulEncoderClocks * g_ulEncoderLines));
}

#endif

//*****************************************************************************
//
// Gets the current position of the encoder, specified as a signed 16.16 fixed-
// point value that represents a number of full revolutions.
//
//*****************************************************************************
unsigned long
EncoderCountSet(unsigned long count)
{
    //
    // Convert the encoder position to a number of revolutions and return it.
    //
    return EncoderCount = count;
}

//*****************************************************************************
//
// Gets the current position of the encoder, specified as a signed 16.16 fixed-
// point value that represents a number of full revolutions.
//
//*****************************************************************************
unsigned long
EncoderCountGet(void)
{
    //
    // Convert the encoder position to a number of revolutions and return it.
    //
    return EncoderCount/64;
}


