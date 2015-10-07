/*
  Copyright Bluebird Aero Systems Engineering 2012

  //////////////////////////////////////////
  Suitable for IAR ver 6.40.1
  //////////////////////////////////////////
  
  $Id: StabilizedPlatform.c,v 1.14 2011/07/14 07:19:48 EranS Exp $
  $Log: StabilizedPlatform.c,v $

  Revision 1.14.1  2012/02/12 06:51:48  EranS
  Add pitch&roll limitation
  
  Revision 1.14  2011/07/14 07:19:48  EranS
  added script engine

  Revision 1.13  2011/06/18 09:13:50  EranS
  before france changes

  Revision 1.12  2011/05/11 07:39:16  EranS
  Integrate with BB library, stow/pilot buttons now work

  Revision 1.11  2011/05/03 07:33:52  EranS
  Move omega initialization code, rename ADC init function

  Revision 1.10  2011/04/12 15:03:35  EranS
  added joystick interface

  Revision 1.9  2011/04/12 05:50:35  EranS
  calibrated gimbal / gyro / accelerometer

  Revision 1.8  2011/04/10 10:37:29  EranS
  Add filer3.c and AHRS from Lumus project

  Revision 1.7  2011/04/06 13:09:03  EranS
  modified BB application to work with stabilizer

  Revision 1.6  2011/04/06 11:59:48  EranS
  unified stabilize code for pitch/roll

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


  Bluebird Stabilized Platform
*/


#include <stdarg.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "bluebird.h"
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "utils/ustdlib.h"
#include "driverlib/gpio.h"
#include "driverlib/adc.h"
#include "driverlib/debug.h"
#include "driverlib/fpu.h"
#include "driverlib/sysctl.h"
#include "driverlib/rom.h"
#include "driverlib/interrupt.h"
#include "driverlib/timer.h"
#include "driverlib/pwm.h"
#include "driverlib/qei.h"
#include "driverlib/pin_map.h"
#include "driverlib/uart.h"
#include "sensorlib/hw_mpu9150.h"
#include "sensorlib/hw_ak8975.h"
#include "sensorlib/i2cm_drv.h"
#include "sensorlib/ak8975.h"
#include "sensorlib/mpu9150.h"
#include "sensorlib/comp_dcm.h"
#include "utils/uartstdio.h"
#include "MasterI2C.h"
#include "ITG3200.h"
#include "bma180.h"
//#include "uart_echo.h"
#include "SysTick.h"
#include "utilities.h"
#include "Led.h"
#include "A3906.h"
#include "rmb20d01.h"
#include "Stabilize.h"
#include "Interpret.h"
#include "comproc.h"
#include "pins.h"
#include "MAG3110.h"
#include "encoder.h"
#include "filter3.h"
#include "reg.h"
#include "comproc.h"

#define COMP_DCM
#define until(B) {volatile unsigned long long dd=0xffffffff; do{if(!B) break;}while(--dd);}
#define M_PI                    3.14159265358979323846

/* ---- Private Variables ------------------------------------------------ */
//#define TELEMETRY
// 2.4.8 for MINTRON Engine version

tBoolean StowBitSet = false;
tBoolean PilotBitSet = false;
tBoolean onetimepilot = false;
tBoolean onetimestow = false;
tBoolean stabilizepitch = true;
// output from joystick, from -128 to +128 each axis
unsigned long lastitg;

#ifdef COMP_DCM
uint_fast32_t ui32Idx, ui32CompDCMStarted;
float pfData[16];
float *pfAccel, *pfGyro, *pfMag, *pfEulers, *pfQuaternion;
int_fast32_t i32IPart[16], i32FPart[16];

//*****************************************************************************
//
// Global flags to alert main that MPU9150 I2C transaction is complete
//
//*****************************************************************************
volatile uint_fast8_t g_vui8I2CDoneFlag;

//*****************************************************************************
//
// Global flags to alert main that MPU9150 I2C transaction error has occurred.
//
//*****************************************************************************
volatile uint_fast8_t g_vui8ErrorFlag;

//*****************************************************************************
//
// Global instance structure for the ISL29023 sensor driver.
//
//*****************************************************************************
tMPU9150 g_sMPU9150Inst;

//*****************************************************************************
//
// Global Instance structure to manage the DCM state.
//
//*****************************************************************************
tCompDCM g_sCompDCMInst;

//*****************************************************************************
//
// Global counter to control and slow down the rate of data to the terminal.
//
//*****************************************************************************
#define PRINT_SKIP_COUNT        1

uint32_t g_ui32PrintSkipCounter;

#endif

struct list_head {
	struct list_head *next, *prev;
};

enum apm_suspend_state {
	SUSPEND_NONE = 0,
	SUSPEND_PENDING,
	SUSPEND_READ,
	SUSPEND_ACKED,
	SUSPEND_ACKTO,
	SUSPEND_WAIT,
	SUSPEND_DONE,
};

enum apm_standby_state {
	STANDBY_NONE = 0,
	STANDBY_PENDING,
	STANDBY_READ,
	STANDBY_ACKED,
	STANDBY_ACKTO,
	STANDBY_WAIT,
	STANDBY_DONE,
};
/*
 * The per-file APM data
 */
struct apm_user {
	struct list_head	list;

	unsigned int		suser: 1;
	unsigned int		writer: 1;
	unsigned int		reader: 1;

	int			suspend_result;
	int			standby_result;
	
	enum apm_suspend_state	suspend_state;		
	int b;
	enum apm_standby_state  standby_state;
	
	//struct apm_queue	queue;
};


// JOYSTICK ////////////////////////////////////////////////////////////////
// output from joystick, from -128 to +128 each axis
//int   joy_ver = 0, joy_hor=0;
////////////////////////////////////////////////////////////////////////////
void SetBitMode(void);
void AHRS_Init();
void AHRS();

tBoolean g_bInit = true;

//*****************************************************************************
//
// System clock rate in Hz.
//
//*****************************************************************************
extern unsigned long g_ui32SysClock;

extern float omega_vertical,omega_horizontal;  
extern float Phic,Thetac;
extern float u[2];
extern unsigned long g_ulqei0pos_left,g_ulqei0pos_right;
extern tBoolean g_tbstabilizer;
extern ctrl_type ctrl;
extern MINTRON_TXRX_S LensZoomWrite;
extern unsigned long g_ulAverage;
extern float y_rate_roll;
extern signed short ITG3200values[3];

extern float fPhi6DOF, fLPPhi6DOF;					// raw and low pass roll angle phi (deg)
extern float fThe6DOF, fLPThe6DOF;					// raw and low pass pitch angle the (deg)
extern float fPsi6DOF, fLPPsi6DOF;					// raw and low pass yaw angle psi (deg)
extern float fRho6DOF, fLPRho6DOF;					// raw and low pass compass angle rho (deg)
extern float fdelta6DOF, fLPdelta6DOF;				// raw and low pass filtered geomagnetic inclination angle delta (deg) 
extern short iGpx, iGpy, iGpz;						// raw accelerometer sensor output in bit counts 
extern short iBpx, iBpy, iBpz;						// raw magnetometer sensor output in bit counts 
extern float fFitErrorpc;							    // current computed fit error %
//*****************************************************************************
//
// The error routine that is called if the driver library encounters an error.
//
//*****************************************************************************
#ifdef DEBUG
void
__error__(char *pcFilename, unsigned long ulLine)
{
}
#endif


//unsigned long ulSysClock;
void HardwareInit()
{
  IntMasterDisable();
  
  // Set the system clock to run at 50MHz from the PLL. // PLL=400MHz
  // sysc2000000lk = 400MHz/2/4 = 50MHz
  //SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);

  ROM_SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);

  // Set the system clock to run at 20MHz from the PLL. // PLL=100MHz
  // sysc2000000lk = 100MHz/2/4 = 20MHz
  //SysCtlClockSet(SYSCTL_SYSDIV_10| SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);
  g_ui32SysClock = ROM_SysCtlClockGet();
  
   
  if (InitUART(UART0_BASE,SysCtlClockGet(),19200,
      	(UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE)) == -1)
      	  while (1);    // hang

  //if (InitUART(UART1_BASE,SysCtlClockGet(),9600,
  //    	(UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE)) == -1)
  //    	  while (1);    // hang
      
  SysTickInit(1000); // 1msec counter
  TimerInit(1);    	 // 100ms counter
  
  BBLedInit();  
  DEBUG_LED1(1);
  DEBUG_LED1(0);  
  DEBUG_LED2(1);	 
  DEBUG_LED2(0);	 

  MasterI2C0Init(SysCtlClockGet() >= 40000000L);  // Need 40MHz for 400K
  A3906Init(); 
  RMBD01Init();
  Encoder_Init(QEI0_BASE);   // J6 PITCH Encoder 
  Encoder_Init(QEI1_BASE);   // J8 ROLL Encoder
  EncoderLinesSet(QEI0_BASE,64);
  EncoderLinesSet(QEI1_BASE,64);
  //HBridgeInit();

  //GPIOPinInit('E',4,false,false); /*TP5 */
  //GPIOPinInit('E',5,false,false); /*TP6 */

  IntMasterEnable();
}

void InitParams(void)
{

	SetDMCflag(0xFFFF, 0);	
	SetDMCflag(DMC_DISABLE_ZOOM,DMC_DISABLE_ZOOM); 	
	SetDMCflag(DMC_STABILIZE_OFF,DMC_STABILIZE_OFF);
	SetDMCflag(DMC_RATE,DMC_RATE); 
	
	cpReset(0);
	cpReset(1);
	init_ctrl();
		
}

void SoftwareInit()
{

  //
  // initialization BlueBird parameters for communication protocol.
  //
  InitParams();
  ITG3200Init();
  
  #ifdef USE_BMA180_INTERRUPT
  BMA180Init();
  #endif
  MAG3110Init();

  //
  // Set the priorities of the interrupts.	
  // The following interrupts are used
  // (listed in priority from highest to lowest):
  //
  //	 QEI-ROLL	 - The input from the quadrature encoder.
  //
  //	 QEI-PITCH	 - The input from the quadrature encoder.
  //  
  //	 PWM0		 - The periodic control loop for the application.
  //
  //	 PWM1		 - The periodic control loop for the application.
  //  
  //	 UART0		 - The UART <-> BBGCS.  This must be the same
  //				   priority as the PAYLOAD interrupt in order to provide the
  //				   required mutual exclusion between the two.
  //
  //	 UART1		 - The UART <-> PAYLOAD.  This must be the same
  //				   priority as the BBGCS interrupt in order to provide the
  //				   required mutual exclusion between the two.
  //
  //
  // Set the priorities of the interrupts used by the application.
  //

  //
  // Enable the peripherals used by this example.
  //
  
  IntPrioritySet(INT_I2C0, 0x00);
  IntPrioritySet(INT_GPIOA, 0x02);
  IntPrioritySet(INT_GPIOE, 0x40);  
  IntPrioritySet(QEI_PITCH_PHA_INT, 0x60);
  IntPrioritySet(QEI_ROLL_PHA_INT, 0x60);
  IntPrioritySet(INT_PWM0, 0x80);
  IntPrioritySet(INT_PWM1, 0x80);
  IntPrioritySet(INT_UART0, 0xA0);  
  IntPrioritySet(INT_UART1, 0xA0);
 

  //
  // PITCH_MOTOR is PORTB & J6- PITCH Encoder 
  //
  // moves gimbal to extreme right and down
  MotorBrakePositionClean(PITCH_MOTOR);
  //GeneralPitchRoll(PITCH_MOTOR,A3906_FORWARD,500);	 
  //EncoderInitReset(QEI0_BASE,PITCH_MOTOR,A3906_REVERSE);  // J6 PITCH Encoder  
  //EncoderInitReset(QEI0_BASE,PITCH_MOTOR,A3906_FORWARD);  // J6 PITCH Encoder  
  //EncoderInitReset(QEI0_BASE,PITCH_MOTOR,A3906_REVERSE);  // J6 PITCH Encoder  

  //
  // ROLL_MOTOR is PORTA & J8- ROLL Encoder
  //
  // moves gimbal to extreme right and down    
  MotorBrakePositionClean(ROLL_MOTOR);
  //GeneralPitchRoll(ROLL_MOTOR,A3906_FORWARD,800);
  //EncoderInitReset(QEI1_BASE,ROLL_MOTOR,A3906_FORWARD); // J8 ROLL Encoder  
  //EncoderInitReset(QEI1_BASE,ROLL_MOTOR,A3906_REVERSE); // J8 ROLL Encoder
  //EncoderInitReset(QEI1_BASE,ROLL_MOTOR,A3906_FORWARD); // J8 ROLL Encoder  
  
  #ifdef USE_BMA180_INTERRUPT
  AHRS_Init();
  #endif
  
  lastitg = g_ulTickCount;  
  StabilizeInit();
  //SetBitMode();

  
}

unsigned long dmcflag;
unsigned int loopcount;
float PitchAmp = 0 ;

 int main(void)
{
  unsigned long last_telemtry;
  char pbuff[300]; 
  long lTemp;
  struct apm_user *as_usr_list;

  #ifdef COMP_DCM

  //
  // Initialize convenience pointers that clean up and clarify the code
  // meaning. We want all the data in a single contiguous array so that
  // we can make our pretty printing easier later.
  //
  pfAccel = pfData;
  pfGyro = pfData + 3;
  pfMag = pfData + 6;
  pfEulers = pfData + 9;
  pfQuaternion = pfData + 12;
  #endif
	
  //
  // The FPU should be enabled because some compilers will use floating-
  // point registers, even for non-floating-point code.  If the FPU is not
  // enabled this will cause a fault.  This also ensures that floating-
  // point operations could be added to this application and would work
  // correctly and use the hardware floating-point unit.  Finally, lazy
  // stacking is enabled for interrupt handlers.  This allows floating-
  // point instructions to be used within interrupt handlers, but at the
  // expense of extra stack usage.
  //
  if (as_usr_list->standby_state == STANDBY_ACKED)
  {
  	FPUEnable();
  	FPULazyStackingEnable();
  }
  as_usr_list->standby_state = STANDBY_DONE;
  if (as_usr_list->standby_state == STANDBY_DONE)
  {
  	FPUEnable();
  	FPULazyStackingEnable();
  }
  as_usr_list->suspend_state = SUSPEND_ACKED;

  HardwareInit();
  SoftwareInit();


   //
   // Keep only some parts of the systems running while in sleep mode.
   // GPIOB is for the MPU9150 interrupt pin.
   // UART0 is the virtual serial port
   // TIMER0, TIMER1 and WTIMER5 are used by the RGB driver
   // I2C3 is the I2C interface to the ISL29023
   //
   //ROM_SysCtlPeripheralClockGating(true);
   //ROM_SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_GPIOA);
   //ROM_SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_UART0);
   //ROM_SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_TIMER0);
   //ROM_SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_TIMER1);
   //ROM_SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_I2C0);
   //ROM_SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_WTIMER5);

   
  //last_telemtry = g_ulTickCount;
  //
  // Initialize the DCM system. 50 hz sample rate.
  // accel weight = .2, gyro weight = .8, mag weight = .2
  //
  CompDCMInit(&g_sCompDCMInst, 1.0f / 50.0f, 0.2f, 0.6f, 0.2f);
  omega_vertical = 0.0f;
  omega_horizontal = 0.0f;

  ui32CompDCMStarted = 0;
  while(1)
  {

		
    	// Execute main loop to interpretation protocol command	    
    	//MainModeLoop();	
		EncoderTick(QEI0_BASE); // PITCH Encoder	  
		EncoderTick(QEI1_BASE); // ROLL Encoder

		#if 0
		// restrict telemtry to about 10Hz, 
		if ((g_ulTickCount - last_telemtry) > 10)
		//if(magready)
		{	 
			last_telemtry = g_ulTickCount;
			eCompassCalculate();

			
			sprintf(pbuff,"%.2f,%.2f, ACC: %hd,%hd,%hd MAG: %hd,%hd,%hd\r\n",fRho6DOF,fFitErrorpc,iGpx,iGpy,iGpz,iBpx,iBpy,iBpz);
			UART0Send(pbuff, strlen(pbuff));

			//sprintf(pbuff,"%.2f,%.2f,%.2f,%.2f, ACC: %hd,%hd,%hd MAG: %hd,%hd,%hd\r\n",fRho6DOF,fPhi6DOF,fThe6DOF,fPsi6DOF,iGpx,iGpy,iGpz,iBpx,iBpy,iBpz);
			//UART0Send(pbuff, strlen(pbuff));

			//sprintf(pbuff,"%.2f,%.2f,%.2f,%.2f,ACC: %hd,%hd,%hd MAG: %hd,%hd,%hd\r\n",fRho6DOF,fPhi6DOF,fThe6DOF,fPsi6DOF,iGpx,iGpy,iGpz,iBpx,iBpy,iBpz);
			//UART0Send(pbuff, strlen(pbuff));

			//sprintf(pbuff,"ROLL %.2f,PITCH %.2f,YAW %.2f\r\n",fPhi6DOF,fThe6DOF,fPsi6DOF);
			//UART0Send(pbuff, strlen(pbuff));
			
			//sprintf(pbuff,"Phi %.2f The %.2f Psi %.2f Rho %.2f delta %.2f\n", fPhi6DOF, fThe6DOF, fPsi6DOF, fRho6DOF, fdelta6DOF);
			//sprintf(pbuff,"%lu,%.2f\r\n",g_ulTickCount,fRho6DOF);	
			//UART0Send(pbuff, strlen(pbuff)); 


		}
		#endif	

		if((itgready == true) && (magready == true) && (xbmaready == true))
		//if((itgready == true))	
		{
		    //DEBUG_TP6(0);
						
			/*------------------------------------------------------------*/
			/* ITG3200getXYZ() time is 30~ microsecond						   */
			/*------------------------------------------------------------*/	
			ITG3200getXYZ(pfGyro, pfGyro + 1,pfGyro + 2);
		
			#ifdef USE_BMA180_INTERRUPT
			/*------------------------------------------------------------*/
			/* BMA180GetXYZ() time is 30~ microsecond						   */
			/*------------------------------------------------------------*/				
			BMA180GetXYZ( pfAccel, pfAccel + 1,pfAccel + 2); 		

			/*------------------------------------------------------------*/
			/* ITG3200getXYZ() time is 30~ microsecond						   */
			/*------------------------------------------------------------*/	
			MAG3110getXYZ(pfMag, pfMag + 1,pfMag + 2);
			//eCompassCalculate();

			#ifdef COMP_DCM		
			//
			// Check if this is our first data ever.
			//
			if(ui32CompDCMStarted == 0)
			{
				//
				// Set flag indicating that DCM is started.
				// Perform the seeding of the DCM with the first data set.
				//
				ui32CompDCMStarted = 1;
				CompDCMMagnetoUpdate(&g_sCompDCMInst, pfMag[0], pfMag[1],
									 pfMag[2]);
				CompDCMAccelUpdate(&g_sCompDCMInst, pfAccel[0], pfAccel[1],
								   pfAccel[2]);
				CompDCMGyroUpdate(&g_sCompDCMInst, pfGyro[0], pfGyro[1],
								  pfGyro[2]);
				CompDCMStart(&g_sCompDCMInst);
			}
			else
			{
				//
				// DCM Is already started.	Perform the incremental update.
				//
				CompDCMMagnetoUpdate(&g_sCompDCMInst, pfMag[0], pfMag[1],
									 pfMag[2]);
				CompDCMAccelUpdate(&g_sCompDCMInst, pfAccel[0], pfAccel[1],
								   pfAccel[2]);
				CompDCMGyroUpdate(&g_sCompDCMInst, -pfGyro[0], -pfGyro[1],
								  -pfGyro[2]);
				CompDCMUpdate(&g_sCompDCMInst);
			}

			//
			// Increment the skip counter.	Skip counter is used so we do not
			// overflow the UART with data.
			//
			g_ui32PrintSkipCounter++;
			if(g_ui32PrintSkipCounter >= PRINT_SKIP_COUNT)
			{
				//
				// Reset skip counter.
				//
				g_ui32PrintSkipCounter = 0;
			
				//
				// Get Euler data. (Roll Pitch Yaw)
				//
				CompDCMComputeEulers(&g_sCompDCMInst, pfEulers, pfEulers + 1,
									 pfEulers + 2);


	            //
	            // convert mag data to micro-tesla for better human interpretation.
	            //
	            //pfMag[0] *= 1e6;
	            //pfMag[1] *= 1e6;
	            //pfMag[2] *= 1e6;

	            //
	            // Convert Eulers to degrees. 180/PI = 57.29...
	            // Convert Yaw to 0 to 360 to approximate compass headings.
	            //
	            //pfEulers[0] *= 57.295779513082320876798154814105f;
	            //pfEulers[1] *= 57.295779513082320876798154814105f;
	            //pfEulers[2] *= 57.295779513082320876798154814105f;
	            //if(pfEulers[2] < 0)
	            //{
	                //pfEulers[2] += 360.0f;
	            //}

				//sprintf(pbuff, "%3.3f; %3.3f; %3.3f \r\n",
				//		pfData[9],pfData[10],Phic);
				//UART0Send(pbuff, strlen(pbuff));


				#if 0
	            //
	            // Now drop back to using the data as a single array for the
	            // purpose of decomposing the float into a integer part and a
	            // fraction (decimal) part.
	            //
	            for(ui32Idx = 0; ui32Idx < 16; ui32Idx++)
	            {
	                //
	                // Conver float value to a integer truncating the decimal part.
	                //
	                i32IPart[ui32Idx] = (int32_t) pfData[ui32Idx];

	                //
	                // Multiply by 1000 to preserve first three decimal values.
	                // Truncates at the 3rd decimal place.
	                //
	                i32FPart[ui32Idx] = (int32_t) (pfData[ui32Idx] * 1000.0f);

	                //
	                // Subtract off the integer part from this newly formed decimal
	                // part.
	                //
	                i32FPart[ui32Idx] = i32FPart[ui32Idx] -
	                                    (i32IPart[ui32Idx] * 1000);

	                //
	                // make the decimal part a positive number for display.
	                //
	                if(i32FPart[ui32Idx] < 0)
	                {
	                    i32FPart[ui32Idx] *= -1;
	                }
	            }

				//
				// Print the angular velocities in the table.
				//
				sprintf(pbuff,"\033[7;17H%3d.%03d", i32IPart[3], i32FPart[3]);
				UART0Send(pbuff, strlen(pbuff)); 
				sprintf(pbuff,"\033[7;40H%3d.%03d", i32IPart[4], i32FPart[4]);
				UART0Send(pbuff, strlen(pbuff)); 
				sprintf(pbuff,"\033[7;63H%3d.%03d", i32IPart[5], i32FPart[5]);
				UART0Send(pbuff, strlen(pbuff));

	            //
	            // Print the Eulers in a table.
	            //
				sprintf(pbuff,"\033[5;17H%3d.%03d", i32IPart[9], i32FPart[9]);
				UART0Send(pbuff, strlen(pbuff)); 
				sprintf(pbuff,"\033[5;40H%3d.%03d", i32IPart[10], i32FPart[10]);
				UART0Send(pbuff, strlen(pbuff)); 
				sprintf(pbuff,"\033[5;63H%3d.%03d", i32IPart[11], i32FPart[11]);
				UART0Send(pbuff, strlen(pbuff)); 

				//sprintf(pbuff,"%.2f\r\n",(Phic));
				//UART0Send(pbuff, strlen(pbuff));
				#endif 
	
	
			
			}				
			#endif

			/*------------------------------------------------------------*/
			/* AHRS() time is 25~ microsecond		   						    */
			/*------------------------------------------------------------*/					
			//AHRS();
			
			#endif
			
			if(g_tbstabilizer == true)
			{
				/*------------------------------------------------------------*/
				/* Stabilize() time is 4~ microsecond								  */
				/*------------------------------------------------------------*/						
				Stabilize(ROLL_MOTOR);
				/*------------------------------------------------------------*/
				/* Stabilize() time is 4~ microsecond								  */
					
				/*------------------------------------------------------------*/									
				//Stabilize(PITCH_MOTOR);
				/*------------------------------------------------------------*/
				/* Total time is 115~ microsecond test by EranS 						*/
				/* Gyro and Encoder dose not test yet. Need testing with new board			*/
				/*------------------------------------------------------------*/		
			}		


	}	
	//DEBUG_TP6(0);
	//DEBUG_TP5(0);

  }
}

// chance to turn off motors
void  SysTickAction()
{
  GimbalsBoundsCheck(PITCH_MOTOR);
  GimbalsBoundsCheck(ROLL_MOTOR);
}
