/*
	This file are use for joystick control for SCD Engine

	Copyright BlueBird Aero Systems Ltd 2011
	http://www.bluebird-uav.com

	Revision 1.14  2012/02/21 09:13:50	EranS
	separate MINTRON/SCD project changes
	
*/


#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/debug.h"
#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "driverlib/timer.h"
#include "utils/uartstdio.h"
#include "driverlib/i2c.h"
#include "driverlib/interrupt.h"
#include "Interpret.h"
#include "comproc.h"
#include "Interpret.h"
#include "modeid.h"
#include "globals.h"
#include "reg.h"
#include "uart_echo.h"
#include "SysTick.h"
#include "A3906.h"
#include "rmb20d01.h"
#include "encoder.h"
#include "bluebird.h"
#include "filter3.h"


#define SENSORS_NUM 				 2u
#define GAIN_LEVEL_NUM				 4u
#define INPUTS_NUM 					 SENSORS_NUM
#define GAIN_SENS_NUM				 GAIN_LEVEL_NUM
#define SENSORS_MASK             	 0x000F

#define until(B) {volatile unsigned long long dd=0xffffffff; do{if(!B) break;}while(--dd);}

#define  FOVmin  4.5
#define  FOVmax  48
#define  VERTICAL_PROPORTION_TO_COMMAND 1
#define  HORIZONTAL_PROPORTION_TO_COMMAND 1
#define TXSIZE  20
#define MP_OK 1
#define MP_ERROR -1
#define BEYOND2PIX 4096.0f / 360.0f

float A_HOR  = 0.220f;
float B_HOR  = 0.080f;

float A_SLOW_VER  = 0.00595;
float B_SLOW_VER  = 0.220f;

float A_FAST_VER  = 0.120f;
float B_FAST_VER  = 0.020f;

float g_fmaximum_ver_rate = 7.92f;
float g_fmaximum_hor_rate = 10.42f;

//float LOS_Roll_Pitch=0;

static int opt_zoom = 8;
//*****************************************************************************
//
// The value that is to be modified via bit-banding.
//
//*****************************************************************************
BYTE SensState[INPUTS_NUM] = {0};
BYTE GainLevelState[GAIN_SENS_NUM] = {0};

unsigned char g_sendBuffer[20];

tBoolean ignorejoystick = false;
tBoolean g_tbstabilizer = true;

static char Payload_corr=0; // for Mintron

static int DMCflag = 0;
static char g_curentmode = 10;
float  vertical,horizontal;
char Telemetry[16];
//char Payload_type=14;
//char Payload_type=7; // For IR Golden
char Payload_type=8; // for Mintron

//bTelemBlock m_bTelemBlock;
extern unsigned short DMCmode;
extern ctrl_type ctrl;
extern BYTE	Stabilize_status;
extern LabelCmd LabelCmdMsg[MAX_LABEL_LENGTH];
extern float omega_vertical,omega_horizontal;  
extern float pitchangle;
extern float Phic,Thetac;
extern unsigned short target_pos;
extern unsigned short Agc_level;
extern unsigned short Sens_level;

extern MINTRON_TXRX_S LensZoomWrite;
extern MINTRON_TXRX_S AgcSensLevel;

extern float fPhi6DOF, fLPPhi6DOF;					// raw and low pass roll angle phi (deg)
extern float fThe6DOF, fLPThe6DOF;					// raw and low pass pitch angle the (deg)
extern float fPsi6DOF, fLPPsi6DOF;					// raw and low pass yaw angle psi (deg)
extern float fRho6DOF, fLPRho6DOF;					// raw and low pass compass angle rho (deg)
extern float fFitErrorpc;							    // current computed fit error %

const ZoomPosition FrameArray[13] = {
//Width,Height,BlockNumX,BlockNumY,BlockNum,BlkWidthPwr,BlkBandPwr,Reserve
{ 0x1000,  0x108F, 47.0f }, // Z1
{ 0x1090,  0x110F, 45.0f}, // Z1.2
{ 0x1110,  0x11CF, 40.0f}, // Z1.5
{ 0x11D2,  0x12A6, 35.0f}, // Z2
{ 0x12A8,  0x132F, 30.0f}, // Z3
{ 0x1330,  0x1367, 25.0f}, // Z4
{ 0x1368,  0x13AF, 20.0f}, // Z5
{ 0x13B0,  0x13EF, 15.0f}, // Z6
{ 0x13F0,  0x1427, 10.0f}, // Z7
{ 0x1428,  0x1447, 8.0f}, // Z8
{ 0x1448,  0x1457, 7.0f}, // Z9
{ 0x1458,  0x1456, 5.0f }  // Z10
};

void InitSens(void)
{
  HWORD i;

 for(i=0; i<INPUTS_NUM; i++)
     SensState[i] = 0;
 
}

void InitGainLevel(void)
{
  HWORD i;

 for(i=0; i<GAIN_SENS_NUM; i++)
		GainLevelState[i] = 0;
 
}


/*
 * FUNCTION: SensorDebounce - Perform the debouncing sensor algorithm
 * DESCRIPTION: SensState[i] keep the previouse samples of the sensors state
 *               in a FIFO shift register 
 */
void SensorDebounce(unsigned char* msg)
{
   unsigned short i, s, frs;

   frs = msg[6];

   if( frs & 0x3)
   {	   	      
	   frs &= SENSORS_MASK;   
	   
	   for(i=0; i<INPUTS_NUM; i++, frs >>= 1)
	   {
	     s = (unsigned short)SensState[i] << 1;
	     s |= frs & 1;
	     SensState[i] = (unsigned char)(s & 0x1F);
	   }
   }
   else
	InitSens();
   	
}



/*
 * FUNCTION: SensorDebounce - Perform the debouncing sensor algorithm
 * DESCRIPTION: SensState[i] keep the previouse samples of the sensors state
 *               in a FIFO shift register 
 */
void GainDebounce(unsigned char* msg)
{
   unsigned short i, s, frs;

   frs = msg[5];

   if( frs & 0xF)
   {	   	      
	   frs &= SENSORS_MASK;   
	   
	   for(i=0; i<GAIN_LEVEL_NUM; i++, frs >>= 1)
	   {
	     s = (unsigned short)GainLevelState[i] << 1;
	     s |= frs & 1;
	     GainLevelState[i] = (unsigned char)(s & 0x3);
	   }
   }
   else
	InitGainLevel();
   	
}


/*
 *               New Full Majority Table
 *
 * Bit wise Majority  table: '1' - more '1' than '0'
 *                           '0' - more '0' than '1'
 *                           '-1'- num('0') == num('1') 
 */
const unsigned short MajorityTab[256] = {	
 0, 1,-1,-1,-1, 0, 1,-1,-1, 0, 0, 0, 1, 0, 1,-1,
-1, 0, 0, 0, 0, 0, 0,-1, 1, 0, 0,-1, 1,-1,-1, 1, 
 0, 0, 0, 0, 0, 0, 0,-1, 0, 0, 0,-1, 0,-1,-1, 1, 
-1, 0, 0,-1, 0,-1,-1, 1, 1,-1,-1, 1,-1, 1, 1, 1, 
 0, 0, 0, 0, 0, 0, 0,-1, 0, 0, 0,-1, 0,-1,-1, 1, 
 0, 0, 0,-1, 0,-1,-1, 1, 0,-1,-1, 1,-1, 1, 1, 1, 
 0, 0, 0,-1, 0,-1,-1, 1, 0,-1,-1, 1,-1, 1, 1, 1, 
-1,-1,-1, 1,-1, 1, 1, 1,-1, 1, 1, 1, 1, 1, 1, 1, 
 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,-1, 0,-1,-1,-1, 
 0, 0, 0,-1, 0,-1,-1, 1, 0,-1,-1, 1,-1, 1, 1, 1, 
 0, 0, 0,-1, 0,-1,-1, 1, 0,-1,-1, 1,-1, 1, 1, 1, 
 0,-1,-1, 1,-1, 1, 1, 1,-1, 1, 1, 1, 1, 1, 1,-1, 
 0, 0, 0, 0, 0,-1,-1, 0, 0,-1,-1, 1,-1, 1, 1,-1, 
 0,-1,-1, 1,-1, 1, 1, 1,-1, 1, 1, 1, 1, 1, 1,-1, 
 0, 0,-1, 0,-1, 1, 1, 0,-1, 1, 1, 1, 1, 1, 1,-1,
-1, 0, 1, 0, 0, 1, 1,-1,-1, 0, 1,-1,-1,-1, 0, 1};

/*
 * FUNCTION: GetSensorState - Returns the sensors state after the debounce filter
 *           based on majority table.
 */
unsigned short GetSensorState(void)
{
   static unsigned short state = 0;
   unsigned short i,f;
   signed short db;
   for (i=0,f=1; i< INPUTS_NUM; i++, f<<=1)
   {
      db = (signed short)MajorityTab[SensState[i]];
      if(db > 0) // More 1's than 0's
         state |= f;
      else
      if(db == 0) // More 0' than 1's
         state &= ~ f;
    }

    return state; 
}


unsigned int GetDMCflag(unsigned int mask)
{
   return (DMCflag & mask);
}

void SetDMCflag(unsigned int mask, unsigned int val)
{
   DMCflag &= ((~mask) | val);
   DMCflag |= (mask & val); 
}


//*****************************************************************************
//
// Send a string to the UART.
//
//*****************************************************************************
void
UART0SendDataBlock(unsigned char *pucBuffer, unsigned ulCount)
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


void ReportData()
{


	//static int num = 0;
	int checksum,i;
	float Roll_Report,Pitch_Report;
	float tmpPhic=0,tmpThetac=0;
	int temp = 0;
	//float Los_Roll=0.0f;

	//Los_Roll = (Phic * RAD2DEG);
	//Los_Roll *= BEYOND2PIX;

	//tmpPhic = Motor[ROLL_MOTOR].aangle; // (Phic * RAD2DEG);
	Roll_Report = Motor[ROLL_MOTOR].aangle; // (Phic * RAD2DEG);;
	Roll_Report *= BEYOND2PIX;

	//tmpThetac= Motor[PITCH_MOTOR].aangle; // (Thetac* RAD2DEG);	
	Pitch_Report = Motor[PITCH_MOTOR].aangle; // (Thetac* RAD2DEG);	;
	Pitch_Report*=BEYOND2PIX;
		
	g_sendBuffer[0] = 0xb0; //0xb0;
	g_sendBuffer[1] = 0x3b;
	g_sendBuffer[2] = 0x4f;

	g_sendBuffer[3] = 0;//num++ & 0xFF;				//STATUS W0  - TBD by Bluebird
	g_sendBuffer[4] = 0;				// STATUS W1- TBD by Bluebird			// erez    ????

	temp = 0;	
	temp = ((g_curentmode<<2) & 0xFC);
	g_sendBuffer[5] = temp;				// D0: payload is free to accept commands D1: 0- there are no payload fails
	
	g_sendBuffer[6] = 0;				// R for NUCps	spare	R for AGC ROIps	Iris Reportps

	temp = 0;	
	// D1: 1 == Payload Ready
	if( GetDMCflag(DMC_ENGINE_CONNECT)) // D6: Hot Blk/Wht Rptps
		temp |= 0x02;
	
	g_sendBuffer[7] = ((0x4) | temp); // D1: 1- payload has completed initialization process; D2: 1- camera is using electronic zoom
	
	//g_sendBuffer[8] =  ((int)(38.67f-(FrameArray[opt_zoom].FOV2Deg))/18.4)*129;
	g_sendBuffer[8] =  ((int)(-4.2448f*FrameArray[opt_zoom].FOV2Deg))+51.591f;
	
	g_sendBuffer[9] = ((int)Pitch_Report & 0xFF); // Pitch Report LSB 0-7 Bit
	//Pitch_Report_MSb = ((Pitch_Report >> 8) & 0x0F);
	g_sendBuffer[10] = Payload_type<<4 | (((int)Pitch_Report >> 8) & 0x0F);	
	
 	g_sendBuffer[11] = ((int)Roll_Report & 0xFF); 			// Roll_Report_LSB 0-7 Bit;
	g_sendBuffer[12] = ((int)Roll_Report >> 8) & 0x0F; 			// Roll_Report_LSB 0-3 Bit;

	g_sendBuffer[13] = Payload_corr;

 	//g_sendBuffer[14] = 0;//((int)LOS_Roll_Pitch & 0xFF); 			// Roll_Report_LSB 0-7 Bit;
	//g_sendBuffer[15] = 0;//((int)LOS_Roll_Pitch >> 8) & 0x0F; 			// Roll_Report_LSB 0-3 Bit;

	//g_sendBuffer[16] = ((int)Los_Roll & 0xFF); ;
	//g_sendBuffer[17] = ((int)Los_Roll >> 8) & 0x0F;

	//g_sendBuffer[14] = ((int)LOS_Roll_Pitch >> 24) & 0xFF; 			// LOS Roll_Report;
 	//g_sendBuffer[15] = ((int)LOS_Roll_Pitch >> 16) & 0xFF; 			
	//g_sendBuffer[16] = ((int)LOS_Roll_Pitch >> 8) & 0xFF;			
 	//g_sendBuffer[17] = ((int)LOS_Roll_Pitch) & 0xFF; 	

	for (checksum = 0,i=0;i<(18);i++)
		checksum+=g_sendBuffer[i];
	g_sendBuffer[19] = checksum % 256;

	/* write the users command out the serial port */
 	UART0Send(g_sendBuffer, 20);	
 }

void ObservationMode(unsigned char* msg)
{
int cmd;
//unsigned char len=2;	
//float tmpPhic=0,tmpThetac=0;


	/*Payload Pitch and Roll  Rate Command to calculate the pixel 
	location of the center of the required video image
	Pitch (vertical) 0.5 [rad/sec] = 1000 Pixels/Sec
	Roll (Horizontal) 1[rad/sec] = 2000 pixels/Sec
	For every video frame, Update the center of the FOV by 
	these rates multiplied by the command range as defined in Byte 13 & 14:
	*/
	/*Byte 13
	D0-D7	
	Vertical rate command in camera coordinate system. 
	Together with Byte 8/D0-D1 determines 10 bit unsigned command. 
	The payload response to a command in the allowed range of [0..1023] is  
	given by:  max_rate*(command-512)/511  
	Command of 512 --> 0 rate
	Positive command --> line of sight moves upward
	In Point Coordinate mode this command moves target position 
	data by an offset proportional to command.
	In Hold Coordinate mode this command moves holding coordinates 
	at a rate proportional to command.
	*/
	//printf("Interpret - ObservationMode()@%d \n",__LINE__);   //debug
	//Roll_Rate = ((Data[7] & 0xF)<<2) +  ((Data[8] & 0xC)>>2);   
	//Pitch_Rate = ((Data[8] & 0xF)<<2) +  (Data[11] & 0x3); 

	//printf(" 0x%08X 0x%08X\n",*vertical,*horizontal); 

	cmd = (int)((int)(msg[16]<<2) + (msg[11] & 0x3));
	//vertical = VERTICAL_PROPORTION_TO_COMMAND*(512-cmd)/4;		// Nissim  /511 ; 
	vertical = (float)g_fmaximum_ver_rate*((((float)cmd-512.0f)/511.0f));		//Nissim  /511  ;

	//horizontal = HORIZONTAL_PROPORTION_TO_COMMAND * (512-cmd)/4;		//Nissim  /511  ;
	/*
	Byte 14
	D0-D7	
	Horizontal rate command in camera coordinate system. 
	Together with Byte 8/D2-D3 determines 10 bit unsigned command. 
	The payload response to a command in the allowed range of [0..1023] is  
	given by:  max_rate*(command-512)/511  
	Command of 512 --> 0 rate
	Positive command --> line of sight moves to the left.
	In Point Coordinate mode this command moves target position data by an offset proportional to command.
	In Hold Coordinate mode this command moves holding coordinates at a rate proportional to command.
	*/
	cmd = (int)((int)(msg[17]<<2) + ((msg[11])>>2 & 0x3));
	//vertical = VERTICAL_PROPORTION_TO_COMMAND*(512-cmd)/4;		// Nissim  /511 ; 
	//horizontal = HORIZONTAL_PROPORTION_TO_COMMAND * (512-cmd)/4;		//Nissim  /511  ;
	horizontal = (float)g_fmaximum_hor_rate*(((float)cmd-512.0f)/511.0f);		// Nissim  /511 ; 
}

int PointCoordinateMode(unsigned char* msg)
{
	if(GetDMCmode() == ONLINE_STATE)
	{
	
		if( (!GetDMCflag(DMC_POINT_2_COORD)) || (!GetDMCflag(DMC_EXE_MODE)) )					
			{					
	
				SetDMCflag(DMC_POINT_2_COORD,DMC_POINT_2_COORD);							
				SetDMCflag(DMC_CAM_GUIDE | DMC_STOW | DMC_PILOT_WINDOW | DMC_RATE | DMC_PTC_CG,0);								
				
				SetDMCflag(DMC_EXE_MODE,DMC_EXE_MODE);
	
				if(GetDMCflag(DMC_OSD_ON))
				{								
					Set_OSD_Titel(4);			
					cpPutMessage(UART1_BASE,0x10,sizeof(LensZoomWrite),(unsigned char *)&LensZoomWrite);  
				}		
				
				return POINT_COORDINATE_MODE;
			}				
	}

	return -1;

}

int HoldCoordinateMode(unsigned char* msg)
{
	if(GetDMCmode() == ONLINE_STATE)
	{				
		if( (!GetDMCflag(DMC_RATE)) || (!GetDMCflag(DMC_EXE_MODE)) )	
		{
			SetDMCflag(DMC_RATE,DMC_RATE);						
			SetDMCflag(DMC_STOW | DMC_PILOT_WINDOW | DMC_POINT_2_COORD | DMC_CAM_GUIDE | DMC_PTC_CG ,0);																									
	
			SetDMCflag(DMC_EXE_MODE,DMC_EXE_MODE);

			g_tbstabilizer = true;			
			ignorejoystick = false; 						
			omega_vertical = (Thetac * RAD2DEG);		
			omega_horizontal = (Phic * RAD2DEG);		
			
			MotorBrakePositionClean(PITCH_MOTOR);
			MotorBrakePositionClean(ROLL_MOTOR);
			
			if(GetDMCflag(DMC_OSD_ON))
			{										
				Set_OSD_Titel(1);			
				cpPutMessage(UART1_BASE,0x10,sizeof(LensZoomWrite),(unsigned char *)&LensZoomWrite);  
			}
			
			return HOLD_COORDINATE_MODE;				
		}
		
	}					

	return -1;

}

int RateMode(unsigned char* msg)
{
	g_tbstabilizer = true;			

	if(GetDMCmode() == ONLINE_STATE)
	{				
		if( (!GetDMCflag(DMC_RATE)) || (!GetDMCflag(DMC_EXE_MODE)) )	
		{
			SetDMCflag(DMC_RATE,DMC_RATE);						
			SetDMCflag(DMC_STOW | DMC_PILOT_WINDOW | DMC_POINT_2_COORD | DMC_CAM_GUIDE | DMC_PTC_CG ,0);																									
	
			SetDMCflag(DMC_EXE_MODE,DMC_EXE_MODE);

			g_tbstabilizer = true;			
			ignorejoystick = false; 
			
			omega_vertical = (Thetac * RAD2DEG);		
			omega_horizontal = (Phic * RAD2DEG);		
			
			MotorBrakePositionClean(PITCH_MOTOR);
			MotorBrakePositionClean(ROLL_MOTOR);
						
			if(GetDMCflag(DMC_OSD_ON))
			{										
				Set_OSD_Titel(1);			
				cpPutMessage(UART1_BASE,0x10,sizeof(LensZoomWrite),(unsigned char *)&LensZoomWrite);  
			}
			
			return OBSERVATION_MODE;				
		}
		
	}					

	return -1;
}

int PilotWindowMode(unsigned char* msg)
{

	if(GetDMCmode() == ONLINE_STATE)
	{
		if(!GetDMCflag(DMC_PILOT_WINDOW) || (!GetDMCflag(DMC_EXE_MODE)) )
		{
						
			SetDMCflag(DMC_PILOT_WINDOW,DMC_PILOT_WINDOW);	// signal for control center			
			SetDMCflag(DMC_STOW | DMC_RATE | DMC_POINT_2_COORD | DMC_CAM_GUIDE | DMC_PTC_CG,0); 																

			SetDMCflag(DMC_EXE_MODE,DMC_EXE_MODE);

			g_tbstabilizer = false;	  
			ignorejoystick = true;		

			EncoderInit2Point(PITCH_MOTOR,A3906_FORWARD,Motor[PITCH_MOTOR].max_position);		  
			EncoderInit2Point(ROLL_MOTOR,A3906_FORWARD,Motor[ROLL_MOTOR].zero_pos); // SCD Engine encoder 360

			//EncoderWhile2Point(PITCH_MOTOR,A3906_FORWARD,Motor[PITCH_MOTOR].max_position);
			//EncoderWhile2Point(ROLL_MOTOR,A3906_REVERSE,Motor[ROLL_MOTOR].avg_position);
			
			if(GetDMCflag(DMC_OSD_ON))
			{			
				Set_OSD_Titel(3);			
				cpPutMessage(UART1_BASE,0x10,sizeof(LensZoomWrite),(unsigned char *)&LensZoomWrite);  
			}
			
			return PILOT_WINDOW_MODE;
		}
		
	}		
	
	return -1;
}



int SafeMode(unsigned char* msg)
{

	if(GetDMCmode() == ONLINE_STATE)
	{
		if(!GetDMCflag(DMC_STOW) || (!GetDMCflag(DMC_EXE_MODE)) )
		{
			SetDMCflag(DMC_STOW,DMC_STOW);		
			SetDMCflag(DMC_PILOT_WINDOW | DMC_RATE | DMC_POINT_2_COORD | DMC_CAM_GUIDE | DMC_PTC_CG,0); // signal for control center		
			
			SetDMCflag(DMC_EXE_MODE,DMC_EXE_MODE);

			ignorejoystick = true;	  
			g_tbstabilizer = false;						

			//EncoderInitReset(QEI0_BASE,PITCH_MOTOR,A3906_REVERSE);	// J6 PITCH Encoder  
			//EncoderInitReset(QEI1_BASE,ROLL_MOTOR,A3906_FORWARD); // J8 ROLL Encoder  

			
			EncoderInit2Point(PITCH_MOTOR,A3906_FORWARD,1);		  
			EncoderInit2Point(ROLL_MOTOR,A3906_FORWARD,1); // SCD Engine encoder 360			

			if(GetDMCflag(DMC_OSD_ON))
			{
				Set_OSD_Titel(2);			
				cpPutMessage(UART1_BASE,0x10,sizeof(LensZoomWrite),(unsigned char *)&LensZoomWrite);  
			}
			
			return SAFE_MODE;
		}
	}
	
	return -1;
}


void Osd_Show(unsigned char* msg)
{
 	unsigned short cmd;
 	//unsigned short idx = 1;
 	//unsigned short i = 0;
	//int id = 0;

 		cmd = (msg[3] & 0x40);
	
		switch(cmd)
		{

			case 0x0:

					if(GetDMCmode() == ONLINE_STATE)
					{
						if( GetDMCflag(DMC_OSD_ON) || (!GetDMCflag(DMC_EXE_MODE)) ) // OSD OFF
						{
							SetDMCflag(DMC_OSD_ON,0);																
							ctrl.time[TIME_ZOOM] = GetSysTickCount();														
							putDMCmode(SET_OSD_OFF);	
						}
					}

			break;

			case 0x40:

					if(GetDMCmode() == ONLINE_STATE)
					{
						if( (!GetDMCflag(DMC_OSD_ON)) || (!GetDMCflag(DMC_EXE_MODE)) ) // OSD ON
						{
							SetDMCflag(DMC_OSD_ON,DMC_OSD_ON);	
							ctrl.time[TIME_ZOOM] = GetSysTickCount();	
							putDMCmode(SET_OSD_ON);		
						}
					}

			break;

		
			default:

			break;
			
			}
}

void Mark_Show(unsigned char* msg)
{
 		unsigned short cmd;

 		cmd = (msg[3] & 0x20);
	
		switch(cmd)
		{

			case 0x0:

					if(GetDMCmode() == ONLINE_STATE)
					{
						if( GetDMCflag(DMC_MARK_CROOS) || (!GetDMCflag(DMC_EXE_MODE)) ) // MARK OFF
						{
	 						SetDMCflag(DMC_MARK_CROOS,0);	
							SetDMCflag(DMC_EXE_MODE,DMC_EXE_MODE);

							LensZoomWrite.data[0]=0x0; //																		
							LensZoomWrite.data[1]=0x0; // mark off
							LensZoomWrite.data[12]=0x1; //
							memset(&LensZoomWrite.data[2],0,10);
							cpPutMessage(UART1_BASE,0x56,sizeof(LensZoomWrite),(unsigned char *)&LensZoomWrite);  
						}
					}

			break;

			case 0x20:

					if(GetDMCmode() == ONLINE_STATE)
					{
						if( !GetDMCflag(DMC_MARK_CROOS) || (!GetDMCflag(DMC_EXE_MODE)) ) // MARK ON
						{
							SetDMCflag(DMC_MARK_CROOS,DMC_MARK_CROOS);	
							SetDMCflag(DMC_EXE_MODE,DMC_EXE_MODE);

							LensZoomWrite.data[0]=0x0; // mark off												
							LensZoomWrite.data[1]=0x1; // mark on					
							LensZoomWrite.data[12]=0x1; // mark off						
							memset(&LensZoomWrite.data[2],0,10);												
							cpPutMessage(UART1_BASE,0x56,sizeof(LensZoomWrite),(unsigned char *)&LensZoomWrite);  
						}
					}

			break;

		
			default:

			break;
			
			}
}

// Interpreting the data of the mode
void InterpretCmdMode(unsigned char *bdData1)
{	
	char mode = bdData1[3] & 0x0F;

	if(GetDMCflag(DMC_ENGINE_CONNECT))
	{
		if(g_curentmode != mode)
		{
			if ( mode == OBSERVATION_MODE) // 0
				g_curentmode = RateMode(bdData1);
			else if ( mode == HOLD_COORDINATE_MODE) // 2
				g_curentmode = HoldCoordinateMode(bdData1);
			else if ( mode == POINT_COORDINATE_MODE) //1//
				g_curentmode = PointCoordinateMode(bdData1);			
			else if ( mode == PILOT_WINDOW_MODE) //3// 
				g_curentmode = PilotWindowMode(bdData1);
			else if ( mode == SAFE_MODE) //4// 
				g_curentmode = SafeMode(bdData1);
			//else if ( mode == BIT_MODE) //10
				//g_curentmode = BitMode(bdData1);
			
		}
		
		ObservationMode(bdData1);


	}
}


void SetBitMode(void)
{
    unsigned long start;		


	start = GetSysTickCount();
	while ((GetSysTickCount() - start) < 3000);   // encoder samples at 500Hz, we check at 10Hz
	
	target_pos = E_MAX_ZX10;	
	ZoomFovCalculation();
	g_fmaximum_ver_rate = A_FAST_VER*FrameArray[opt_zoom].FOV2Deg+B_FAST_VER;
	g_fmaximum_hor_rate = A_HOR*FrameArray[opt_zoom].FOV2Deg+B_HOR; 									
	memset(&LensZoomWrite.data,0,13);	
	LensZoomWrite.write = 0x21;
	LensZoomWrite.data[1]=HIBYTE(target_pos);	//	0x00~0xFF(ZOOM target position HIGH BYTE)
	LensZoomWrite.data[2]=LOBYTE(target_pos);	//	0x00~0xFF(ZOOM target position LOW BYTE)	
	cpPutMessage(UART1_BASE,0x45,sizeof(LensZoomWrite),(unsigned char *)&LensZoomWrite);							
			
	start = GetSysTickCount();
	while ((GetSysTickCount() - start) < 3000);   // encoder samples at 500Hz, we check at 10Hz


	target_pos = MINI_ZOOM;	
	ZoomFovCalculation();
	g_fmaximum_ver_rate = A_FAST_VER*FrameArray[opt_zoom].FOV2Deg+B_FAST_VER;
	g_fmaximum_hor_rate = A_HOR*FrameArray[opt_zoom].FOV2Deg+B_HOR; 									
	memset(&LensZoomWrite.data,0,13);	
	LensZoomWrite.write = 0x21;
	LensZoomWrite.data[1]=HIBYTE(target_pos);	//	0x00~0xFF(ZOOM target position HIGH BYTE)
	LensZoomWrite.data[2]=LOBYTE(target_pos);	//	0x00~0xFF(ZOOM target position LOW BYTE)	
	cpPutMessage(UART1_BASE,0x45,sizeof(LensZoomWrite),(unsigned char *)&LensZoomWrite);							
			
	start = GetSysTickCount();
	while ((GetSysTickCount() - start) < 3000);   // encoder samples at 500Hz, we check at 10Hz
	
	Set_OSD_Titel(0xE); 		
			
	LensZoomWrite.write = 0x21;
	LensZoomWrite.data[0]=0x1;	//	
	LensZoomWrite.data[1]=0x0;	// 0x01 (n=11:character 11 ~ 15)		   
			
	LensZoomWrite.data[2] = 0;
	LensZoomWrite.data[4] = 0;	
	LensZoomWrite.data[6] = 0;	
	LensZoomWrite.data[8] = 0;	
	LensZoomWrite.data[10] = 0; 	
	cpPutMessage(UART1_BASE,0x10,sizeof(LensZoomWrite),(unsigned char *)&LensZoomWrite);  

	EncoderWhile2Point(PITCH_MOTOR,A3906_FORWARD,Motor[PITCH_MOTOR].relative);
	EncoderWhile2Point(ROLL_MOTOR,A3906_REVERSE,Motor[ROLL_MOTOR].relative);
					
	Motor[PITCH_MOTOR].zero_pos = QEIPositionGet(QEI0_BASE);
	Motor[ROLL_MOTOR].zero_pos = QEIPositionGet(QEI1_BASE);

	/*------------------------------------------------------------*/
	/* BMA180GetXYZ() time is 30~ microsecond						   */
	/*------------------------------------------------------------*/				
	BMA180GetXYZ(); 				
	/*------------------------------------------------------------*/
	/* AHRS() time is 25~ microsecond										 */
	/*------------------------------------------------------------*/					
	AHRS();

	SetRelativeInertial();

}

void ZoomFovCalculation(void)
{
	int i = 0;

	for(i=0;i<13;i++)
		if((target_pos >= FrameArray[i].StartZoom) && (target_pos <= FrameArray[i].EndZoom) )
		{
			opt_zoom = i;
			break;
		}

}
	
float  FOV_command(int command)
{
/* Linear from widest FOV (0) to narrowest optical FOV (15). Does not include electronic zoom range. 
 FOVmin = 4.5?, FOVmax = 48?
 FOVcmd =FOVmax+ command X  (FOVmin-FOVmax)/15

*/
float FOVcmd , step=(FOVmax-FOVmin)/15;

FOVcmd = FOVmax - step;

//printf("Interpret - FOV_command()@%d \n",__LINE__);

return FOVcmd;
}

void UavLatitude(unsigned char* msg)
{
	unsigned short cmd,mode;
	unsigned short idx = 1;
	unsigned short i = 0;

	mode = (msg[6] & 0x0F);
		
	for(i=1;i<8;i++)
	{
			
		cmd = (msg[6] & idx);
		
		switch(cmd)
		{

			case 0x01: // ZOOM OUT

				if(cmd)
				{
					if(GetDMCmode() == ONLINE_STATE)
					{
						if(target_pos-SensState[0] >= MINI_ZOOM) 								
						{
							target_pos-=SensState[0];	
							ZoomFovCalculation();
							g_fmaximum_ver_rate = A_FAST_VER*FrameArray[opt_zoom].FOV2Deg+B_FAST_VER;
							g_fmaximum_hor_rate = A_HOR*FrameArray[opt_zoom].FOV2Deg+B_HOR;							
							memset(&LensZoomWrite.data,0,13);
							LensZoomWrite.write = 0x21;
							LensZoomWrite.data[1]=HIBYTE(target_pos);	//	0x00~0xFF(ZOOM target position HIGH BYTE)
							LensZoomWrite.data[2]=LOBYTE(target_pos);	//	0x00~0xFF(ZOOM target position LOW BYTE)	
							cpPutMessage(UART1_BASE,0x45,sizeof(LensZoomWrite),(unsigned char *)&LensZoomWrite);
						}
						else
						{
							target_pos = MINI_ZOOM;
							ZoomFovCalculation();
							g_fmaximum_ver_rate = A_FAST_VER*FrameArray[0].FOV2Deg+B_FAST_VER;
							g_fmaximum_hor_rate = A_HOR*FrameArray[0].FOV2Deg+B_HOR;							
							
							memset(&LensZoomWrite.data,0,13);	
							LensZoomWrite.write = 0x21;
							LensZoomWrite.data[1]=HIBYTE(target_pos);	//	0x00~0xFF(ZOOM target position HIGH BYTE)
							LensZoomWrite.data[2]=LOBYTE(target_pos);	//	0x00~0xFF(ZOOM target position LOW BYTE)	
							cpPutMessage(UART1_BASE,0x45,sizeof(LensZoomWrite),(unsigned char *)&LensZoomWrite);							
						}					
					}					
				}
				
			break;

			case 0x02: // ZOOM IN

				if(cmd)
				{
					if(GetDMCmode() == ONLINE_STATE)
					{				

						if(target_pos+SensState[1] <= MAX_ZOOM)					
						{					
							target_pos+=SensState[1];							
							ZoomFovCalculation();
							g_fmaximum_ver_rate = A_FAST_VER*FrameArray[opt_zoom].FOV2Deg+B_FAST_VER;
							g_fmaximum_hor_rate = A_HOR*FrameArray[opt_zoom].FOV2Deg+B_HOR;							
							memset(&LensZoomWrite.data,0,13);							
							LensZoomWrite.write = 0x21;
							LensZoomWrite.data[1]=HIBYTE(target_pos);	//	0x00~0xFF(ZOOM target position HIGH BYTE)
							LensZoomWrite.data[2]=LOBYTE(target_pos);	//	0x00~0xFF(ZOOM target position LOW BYTE)	
							cpPutMessage(UART1_BASE,0x45,sizeof(LensZoomWrite),(unsigned char *)&LensZoomWrite);							
						}
						else
						{
							target_pos = MAX_ZOOM;		
							ZoomFovCalculation();
							g_fmaximum_ver_rate = A_FAST_VER*FrameArray[11].FOV2Deg+B_FAST_VER;
							g_fmaximum_hor_rate = A_HOR*FrameArray[11].FOV2Deg+B_HOR;							
							
							memset(&LensZoomWrite.data,0,13);		
							LensZoomWrite.write = 0x21;
							LensZoomWrite.data[1]=HIBYTE(target_pos);	//	0x00~0xFF(ZOOM target position HIGH BYTE)
							LensZoomWrite.data[2]=LOBYTE(target_pos);	//	0x00~0xFF(ZOOM target position LOW BYTE)								
							cpPutMessage(UART1_BASE,0x45,sizeof(LensZoomWrite),(unsigned char *)&LensZoomWrite);		
							
						}
					
					}									
				}
				
			break;
			
			#if 1
			case 0x04: // Camera Guide Mode
				
				if((mode == 0x0C)) // PTC Mode Point To Koordenet
				{
					if(GetDMCmode() == ONLINE_STATE)
					{
				
							if( !GetDMCflag(DMC_POINT_2_COORD) ||!GetDMCflag(DMC_EXE_MODE) )					
							{					

								SetDMCflag(DMC_POINT_2_COORD,DMC_POINT_2_COORD);							
								SetDMCflag(DMC_CAM_GUIDE | DMC_STOW | DMC_PILOT_WINDOW | DMC_RATE | DMC_PTC_CG,0);								
								
								SetDMCflag(DMC_EXE_MODE,DMC_EXE_MODE);

								if(GetDMCflag(DMC_OSD_ON))
								{								
									Set_OSD_Titel(4);			
									cpPutMessage(UART1_BASE,0x10,sizeof(LensZoomWrite),(unsigned char *)&LensZoomWrite);  
								}								
								
							}
								
					}
				}
				else
				{
 					if(mode == 0x4) 					
					{
						if(cmd)	// CG
						{
								if(GetDMCmode() == ONLINE_STATE)
								{
							
									if( (!GetDMCflag(DMC_CAM_GUIDE)) || (!GetDMCflag(DMC_EXE_MODE)) )				
									{
										SetDMCflag(DMC_CAM_GUIDE,DMC_CAM_GUIDE);									
										SetDMCflag(DMC_POINT_2_COORD | DMC_STOW | DMC_PILOT_WINDOW | DMC_RATE | DMC_PTC_CG,0);

										SetDMCflag(DMC_EXE_MODE,DMC_EXE_MODE);

										if(GetDMCflag(DMC_OSD_ON))
										{									
											Set_OSD_Titel(5);			
											cpPutMessage(UART1_BASE,0x10,sizeof(LensZoomWrite),(unsigned char *)&LensZoomWrite);  
										}					
									}													
									
								}
								
						}
 					}
				
				}
				
			break;

			case 0x08: // Point To Koordenet

				if((mode == 0x0C)) // PTC Mode
				{
						if(GetDMCmode() == ONLINE_STATE)
						{
				
							if( (!GetDMCflag(DMC_POINT_2_COORD)) || (!GetDMCflag(DMC_EXE_MODE)) )					
							{					

								SetDMCflag(DMC_POINT_2_COORD,DMC_POINT_2_COORD);							
								SetDMCflag(DMC_CAM_GUIDE | DMC_STOW | DMC_PILOT_WINDOW | DMC_RATE | DMC_PTC_CG,0);								
								
								SetDMCflag(DMC_EXE_MODE,DMC_EXE_MODE);

								if(GetDMCflag(DMC_OSD_ON))
								{								
									Set_OSD_Titel(4);			
									cpPutMessage(UART1_BASE,0x10,sizeof(LensZoomWrite),(unsigned char *)&LensZoomWrite);  
								}
							}													
						}


				}
				else
				if(cmd) // PTC+CG Mode
				{
 					if(mode == 0x8)
 					{
						if(GetDMCmode() == ONLINE_STATE)
						{						
							if((!GetDMCflag(DMC_PTC_CG)) || (!GetDMCflag(DMC_EXE_MODE)) )				
							{					
								SetDMCflag(DMC_PTC_CG,DMC_PTC_CG);															
								SetDMCflag(DMC_POINT_2_COORD | DMC_CAM_GUIDE | DMC_STOW | DMC_RATE | DMC_PILOT_WINDOW ,0);								
								
								SetDMCflag(DMC_EXE_MODE,DMC_EXE_MODE);
								if(GetDMCflag(DMC_OSD_ON))
								{								
									Set_OSD_Titel(6);			
									cpPutMessage(UART1_BASE,0x10,sizeof(LensZoomWrite),(unsigned char *)&LensZoomWrite);  
								}				
							}			
							
						}		
 					}
				}
				
			break;
			
			case 0x10: 
					
				if(cmd)
				{			
					if(GetDMCmode() == ONLINE_STATE)
					{				
						if( (!GetDMCflag(DMC_RATE)) || (!GetDMCflag(DMC_EXE_MODE)) )	
						{
							SetDMCflag(DMC_RATE,DMC_RATE);						
							SetDMCflag(DMC_STOW | DMC_PILOT_WINDOW | DMC_POINT_2_COORD | DMC_CAM_GUIDE | DMC_PTC_CG ,0); 																									

							SetDMCflag(DMC_EXE_MODE,DMC_EXE_MODE);
							if(GetDMCflag(DMC_OSD_ON))
							{										
								Set_OSD_Titel(1);			
								cpPutMessage(UART1_BASE,0x10,sizeof(LensZoomWrite),(unsigned char *)&LensZoomWrite);  
							}
							
						}
						
					}					
				}

			break;
			#endif
			
			
			default:

			break;


		}	
	
		idx<<=1;	
	}
}

void UavLongitude(unsigned char* msg)
{
	unsigned short cmd;
	unsigned short idx = 1;
	unsigned short i = 0;

	//mode = (msg[6] & 0x0F);
		
	for(i=1;i<8;i++)
	{
			
		cmd = (msg[6] & idx);
		
		switch(cmd)
		{


			case 0x01: // AGC decrease

				if(cmd)
				{
					if(GetDMCmode() == ONLINE_STATE)
					{
						if(Sens_level-SensState[0] >= MINI_SENS_LEVEL) 								
						{
							Sens_level-=SensState[0];					
							LensZoomWrite.write = 0x21;	//	Buf[2]
							LensZoomWrite.data[0]=0x2;	//	Buf[3]
							LensZoomWrite.data[1]=Sens_level; //	Buf[4]								
							memset(&LensZoomWrite.data[2],0,11);
							cpPutMessage(UART1_BASE,0x1A,sizeof(LensZoomWrite),(unsigned char *)&LensZoomWrite);
						}
						else
						{
							Sens_level = MINI_SENS_LEVEL;
							LensZoomWrite.write = 0x21;	//	Buf[2]
							LensZoomWrite.data[0]=0x2;	//	Buf[3]
							LensZoomWrite.data[1]=Sens_level; //	Buf[4]								
							memset(&LensZoomWrite.data[2],0,11);
							cpPutMessage(UART1_BASE,0x1A,sizeof(LensZoomWrite),(unsigned char *)&LensZoomWrite);							
						}					
					}					
				}
				
			break;

			case 0x02: // AGC increase

				if(cmd)
				{
					if(GetDMCmode() == ONLINE_STATE)
					{				

						if(Sens_level+SensState[1] <= MAX_SENS_LEVEL)					
						{					
							Sens_level+=SensState[1];							
							LensZoomWrite.write = 0x21;			//	Buf[2]
							LensZoomWrite.data[0]=0x2;			//	Buf[3]
							LensZoomWrite.data[1]=Sens_level; 	//	Buf[4]								
							memset(&LensZoomWrite.data[2],0,11);
							cpPutMessage(UART1_BASE,0x1A,sizeof(LensZoomWrite),(unsigned char *)&LensZoomWrite);
						}
						else
						{
							Sens_level = MAX_SENS_LEVEL;							
							memset(&LensZoomWrite.data,0,13);		
							LensZoomWrite.write = 0x21;		 //	Buf[2]
							LensZoomWrite.data[0]=0x2;		 //	Buf[3]
							LensZoomWrite.data[1]=Sens_level; //	Buf[4]								
							memset(&LensZoomWrite.data[2],0,11);
							cpPutMessage(UART1_BASE,0x1A,sizeof(LensZoomWrite),(unsigned char *)&LensZoomWrite);		
							
						}
					
					}									
				}
				
			break;
			


			case 0x03: // AGC decrease

				if(cmd)
				{
					if(GetDMCmode() == ONLINE_STATE)
					{
						if(Agc_level-GainLevelState[2] >= MINI_AGC_LEVEL) 								
						{
							Agc_level-=GainLevelState[2];					
							LensZoomWrite.write = 0x21;	//	Buf[2]
							LensZoomWrite.data[0]=0x1;	//	Buf[3]
							LensZoomWrite.data[1]=Agc_level; //	Buf[4]								
							memset(&LensZoomWrite.data[2],0,11);
							cpPutMessage(UART1_BASE,0x1A,sizeof(LensZoomWrite),(unsigned char *)&LensZoomWrite);
						}
						else
						{
							Agc_level = MINI_AGC_LEVEL;
							LensZoomWrite.write = 0x21;	//	Buf[2]
							LensZoomWrite.data[0]=0x1;	//	Buf[3]
							LensZoomWrite.data[1]=Agc_level; //	Buf[4]								
							memset(&LensZoomWrite.data[2],0,11);
							cpPutMessage(UART1_BASE,0x1A,sizeof(LensZoomWrite),(unsigned char *)&LensZoomWrite);							
						}					
					}					
				}
				
			break;

			case 0x04: // AGC increase

				if(cmd)
				{
					if(GetDMCmode() == ONLINE_STATE)
					{				

						if(Agc_level+GainLevelState[3] <= MAX_AGC_LEVEL)					
						{					
							Agc_level+=GainLevelState[3];							
							LensZoomWrite.write = 0x21;	//	Buf[2]
							LensZoomWrite.data[0]=0x1;	//	Buf[3]
							LensZoomWrite.data[1]=Agc_level; //	Buf[4]								
							memset(&LensZoomWrite.data[2],0,11);
							cpPutMessage(UART1_BASE,0x1A,sizeof(LensZoomWrite),(unsigned char *)&LensZoomWrite);
						}
						else
						{
							Agc_level = MAX_AGC_LEVEL;							
							memset(&LensZoomWrite.data,0,13);		
							LensZoomWrite.write = 0x21;
							LensZoomWrite.data[0]=0x1;	//	Buf[3]
							LensZoomWrite.data[1]=Agc_level; //	Buf[4]								
							memset(&LensZoomWrite.data[2],0,11);
							cpPutMessage(UART1_BASE,0x1A,sizeof(LensZoomWrite),(unsigned char *)&LensZoomWrite);		
							
						}
					
					}									
				}
				
			break;

			default:

			break;
			
			}
		
			idx<<=1;	
		
		}
}


void	UAV_Altitude_above_MSL(unsigned char* msg)
{
//float UAV_Altitude;
 //erez   who  is MSB        float
//	UAV_Altitude = (int)((msg[12]<<24) & 0xFF000000);
//	printf("Interpret - UAV_Altitude_above_MSL = %02X %f \n",input_data_com0[12],UAV_Altitude);

//	UAV_Altitude = (int)((msg[13]<<16) & 0xFF0000);
//	printf("Interpret - UAV_Altitude_above_MSL = %02X %f \n",input_data_com0[13],UAV_Altitude);

//	UAV_Altitude = (int)((msg[14]<<8) & 0xFF00);
//	printf("Interpret - UAV_Altitude_above_MSL = %02X %f \n",input_data_com0[14],UAV_Altitude);

//	UAV_Altitude = (int)((msg[15]<<8) & 0xFF00);
//	printf("Interpret - UAV_Altitude_above_MSL = %02X %f \n",input_data_com0[15],UAV_Altitude);

//	UAV_Altitude = (float)((int)(msg[15]) + (int)(msg[14]<<8) + (int)(msg[13]<<16) + (int)(msg[12]<<24)) ;

//	printf("Interpret - UAV_Altitude_above_MSL = %08X %f \n",UAV_Altitude,UAV_Altitude);
}

void 	Ground_Height_above_MSL(unsigned char* msg)
{
  	//int cmd;
 	//cmd = (msg[4] & 0x0F);
}


void 	UAV_Body_Azimuth(unsigned char* msg)
{
//int body_azimuth;

	//body_azimuth = (int)((int)(msg[10]<<8) +  msg[9]);

	//printf("Interpret - UAV_Body_Azimuth=%d  \n",body_azimuth);
}

void 	Ground_Speed()
{
	//printf("Interpret - Ground_Speed()@%d \n",__LINE__);
}

void 	Target_Latitude()
{
	//printf("Interpret - Target_Latitude()@%d \n",__LINE__);
}
void Target_Longitude()
{
	//printf("Interpret - Target_Longitude()@%d \n",__LINE__);
}
void 	Target_Altitude_above_MSL(unsigned char* msg)   //8
{
//int trg_alt_MSL ;
//char   tempString[9];
//erez  who MSB ?
	//trg_alt_MSL = (int)((int)(msg[13]<<8) +  msg[12]);
//	printf(" %x %x \n",input_data_com0[10]<<8,input_data_com0[9]);
//	sprintf(tempString, "%d%%", trg_alt_MSL);
//    drawText(tempString, 10, 20); 
	//printf("Interpret - Target_Altitude_above_MSL = %d \n",trg_alt_MSL);

}

void Dial_Address(unsigned char* msg)
{
	 //erez   who MSB		 float
	//	UAV_longitude = (float)((int)(input_data_com0[15]) + (int)(input_data_com0[14]<<8) + (int)(input_data_com0[13]<<16) + (int)(input_data_com0[12]<<24) );
	//	printf("Interpret - UavLongitude = %f \n",UAV_longitude);
		
}

void LOS_Azimuth(void)
{

	float Los_Azimut = (fRho6DOF*DEG2RAD);//(Phic * RAD2DEG);
	memcpy(&g_sendBuffer[14],&Los_Azimut,sizeof(float));
}

void LOS_Roll(void)
{

	float Los_Roll = fPhi6DOF*DEG2RAD;//Phic;
	memcpy(&g_sendBuffer[14],&Los_Roll,sizeof(float));
}

void LOS_Pitch(void)
{
	float LOSPitch = fThe6DOF*DEG2RAD;//Thetac;
	memcpy(&g_sendBuffer[14],&LOSPitch,sizeof(float));
}

void LOS_FitError(void)
{
	long LOSFitErrorpc = (long)fFitErrorpc;
	memcpy(&g_sendBuffer[14],&LOSFitErrorpc,sizeof(long));
}


	

// Interpreting the data of the correlator
void InterpretCorrelatorMode(unsigned char *bdData1)
{

	char CheckingCorr = (bdData1[11] & CHECKING_CORR_NUMBER)>>4;      // ORG  Nissim	
//	char CheckingCorr = (bdData1[8] & CHECKING_CORR_NUMBER)>>4;      // ORG  Nissim


	if(GetDMCflag(DMC_ENGINE_CONNECT))
	{
		SensorDebounce(bdData1);	//-- fill up majority table	
		UavLatitude(bdData1);
		#ifdef AGC_SENS
		//GainDebounce(bdData1);	//-- fill up majority table			
		UavLongitude(bdData1);
		#endif
	    ctrl.main_count++;
				
	    if ( (ctrl.main_count & 3) == 0 )  //-- write outputs
	    {				   
			Osd_Show(bdData1);			 //-- check overlay state
	    }
	    else
	    if ( (ctrl.main_count & 3) == 1 )  //-- read inputs
	    {	            
	        Mark_Show(bdData1);
	    }
		
		
	}

			memset(&g_sendBuffer[0],0,20);

			if ( CheckingCorr == UAV_LATITUDE)					// 0
			{
					Payload_corr = (CheckingCorr << 4) & 0xF0;
			}
			else if ( CheckingCorr == UAV_LONGITUDE)			// 1
				{
					Payload_corr = (CheckingCorr << 4) & 0xF0;
				}
			else if ( CheckingCorr == UAV_ALTITUDE_ABOVE_MSL)	// 2
				{
					Payload_corr = (CheckingCorr << 4) & 0xF0;
				}
			else if ( CheckingCorr == GROUND_HEIGHT_ABOVE_MSL)	// 3 
				{
					Payload_corr = (CheckingCorr << 4) & 0xF0;
					LOS_Azimuth();
				}
			else if ( CheckingCorr == UAV_BODY_AZIMUTH) 		// 4
				{
					Payload_corr = (CheckingCorr << 4) & 0xF0;
					LOS_Roll();
				}
			else if ( CheckingCorr == GROUND_SPEED) 			// 5
				{
					Payload_corr = (CheckingCorr << 4) & 0xF0;
					LOS_Pitch();
				}
			else if ( CheckingCorr == TARGET_LATITUDE)			//6
				{
					Payload_corr = (CheckingCorr << 4) & 0xF0;
				}
			else if ( CheckingCorr == TARGET_LONGITUDE) 		// 7
				{
					Payload_corr = (CheckingCorr << 4) & 0xF0;
					LOS_FitError();
				}
			else if ( CheckingCorr == TARGET_ALTITUDE_ABOVE_MSL) // 8
				{
					Payload_corr = (CheckingCorr << 4) & 0xF0;
				}
			else if ( CheckingCorr == DIAL_ADDRESS) 			//9
				{
					Payload_corr = (CheckingCorr << 4) & 0xF0;
				}
			else if ( CheckingCorr == ELEVATION_GIMBAL_ANGLE)	// 10
				{
					Payload_corr = (CheckingCorr << 4) & 0xF0;
				}
			else if ( CheckingCorr == SPARE11)			//6
				{
					Payload_corr = (CheckingCorr << 4) & 0xF0;
				}
			else if ( CheckingCorr == SPARE12) 		// 7
				{
					Payload_corr = (CheckingCorr << 4) & 0xF0;
				}
			else if ( CheckingCorr == SPARE13) // 8
				{
					Payload_corr = (CheckingCorr << 4) & 0xF0;
				}
			else if ( CheckingCorr == SPARE14) 			//9
				{
					Payload_corr = (CheckingCorr << 4) & 0xF0;
				}
			else if ( CheckingCorr == SPARE14)	// 10
				{
					Payload_corr = (CheckingCorr << 4) & 0xF0;
				}


	#ifdef _STAMP_CDU_

	if(1)//GetDMCflag(DMC_ENGINE_CONNECT))
	{
		if ( CheckingCorr == UAV_LATITUDE) 					// 0
		{
				UavLatitude(bdData1);
		}
		else if ( CheckingCorr == UAV_LONGITUDE) 			// 1
			{
				UavLongitude(bdData1);
			}
		else if ( CheckingCorr == UAV_ALTITUDE_ABOVE_MSL)  	// 2
			{
				UAV_Altitude_above_MSL(bdData1);
				//PilotWindowMode(bdData1);
			}
		else if ( CheckingCorr == GROUND_HEIGHT_ABOVE_MSL) 	// 3 
			{
				Ground_Height_above_MSL(bdData1);
			}
		else if ( CheckingCorr == UAV_BODY_AZIMUTH) 		// 4
			{
				UAV_Body_Azimuth( bdData1);
			}
		else if ( CheckingCorr == GROUND_SPEED) 			// 5
			{
				Ground_Speed();
			}
	//	else if ( CheckingCorr == GROUND_SPEED_AZIMUTH) //10
	//		{
	//			Ground_Speed_Azimuth();
	//		}
		else if ( CheckingCorr == TARGET_LATITUDE) 			//6
			{
				Target_Latitude();
			}
		else if ( CheckingCorr == TARGET_LONGITUDE) 		// 7
			{
				Target_Longitude();
			}
		else if ( CheckingCorr == TARGET_ALTITUDE_ABOVE_MSL) // 8
			{
				Target_Altitude_above_MSL(bdData1);
			}
		else if ( CheckingCorr == DIAL_ADDRESS) 			//9
			{
				Dial_Address(bdData1);
			}
		else if ( CheckingCorr == ELEVATION_GIMBAL_ANGLE) 	// 10
			{
				Elevation_gimbal_angle();
			}
		else
			{
				// ERROR
				//printf("Interpret.c --> ERROR  Could not interpret correlator @%d \n",__LINE__);
			}
	}
	#endif
}



void InterpretCmd(void)
{

		InterpretCmdMode(cpGetMessage(COM0));		
		InterpretCorrelatorMode(cpGetMessage(COM0));						
		ReportData();
}
