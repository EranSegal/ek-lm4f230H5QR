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
#include "A3906.h"
#include "rmb20d01.h"
#include "encoder.h"

// 3.x for SCD Engine version
 char  *BCVersion  = "Ver:3.9";

#define HEADER_1					 0xB0
#define HEADER_2					 0x3B
#define HEADER_3					 0x4F


#define SENSORS_NUM 				 2u
#define INPUTS_NUM 					 SENSORS_NUM
#define SENSORS_MASK             	 0x0003

#define until(B) {volatile unsigned long long dd=0xffffffff; do{if(!B) break;}while(--dd);}

#define  FOVmin  4.5
#define  FOVmax  48
#define  VERTICAL_PROPORTION_TO_COMMAND 1
#define  HORIZONTAL_PROPORTION_TO_COMMAND 1
#define TXSIZE  20
#define MP_OK 1
#define MP_ERROR -1
#define BEYOND2PIX 4096 / 360

#if 0
float A_HOR  = 0.382f;
float B_HOR  = 0.049f;

float A_SLOW_VER  = 0.00595;
float B_SLOW_VER  = 0.220f;

float A_FAST_VER  = 0.300f;
float B_FAST_VER  = 0.061f;

float g_fmaximum_ver_rate = 0.281f*47.0f+0.061f;
float g_fmaximum_hor_rate = 0.382f*47.0f+0.049f;
#else
float A_HOR  = 0.182f;
float B_HOR  = 0.049f;

float A_FAST_VER  = 0.100f;
float B_FAST_VER  = 0.061f;

float g_fmaximum_ver_rate = 0.100f*20.0f+0.061f;
float g_fmaximum_hor_rate = 0.382f*16.0f+0.049f;

#endif


static int opt_zoom = 8;
//*****************************************************************************
//
// The value that is to be modified via bit-banding.
//
//*****************************************************************************
BYTE SensState[INPUTS_NUM] = {0};
BYTE g_sendBuffer[30];

static char Payload_corr=0; // for Mintron

static int DMCflag = 0;
char g_curentmode = -1;
float  vertical,horizontal;
char Telemetry[16];
//char Payload_type=14;
char Payload_type=7; // For IR Golden
//char Payload_type=8; // for Mintron

extern unsigned short DMCmode;
extern ctrl_type ctrl;
extern BYTE	Stabilize_status;
extern float omega_vertical,omega_horizontal;  
extern float pitchangle;
extern char  *BCVersion;
extern char  *BBVersion;
extern tBoolean g_tbstabilizer;
extern float Phic,Thetac;
extern tBoolean ignorejoystick;
extern Cont_Brit_Fiels	m_Contrast_Set;
extern unsigned char g_ucvisible_lebel;


void ReportData(unsigned long ulBase)
{


	int checksum,i;
	float Roll_Report,Pitch_Report;
	

	Roll_Report = Motor[ROLL_MOTOR].aangle; // (Phic * RAD2DEG);;
	Roll_Report *= BEYOND2PIX;

	Pitch_Report = Motor[PITCH_MOTOR].aangle; // (Thetac* RAD2DEG);	;
	Pitch_Report*=BEYOND2PIX;

		
	g_sendBuffer[0] = HEADER_1;
	g_sendBuffer[1] = HEADER_2;
	g_sendBuffer[2] = HEADER_3;
	//STATUS W0  - TBD by Bluebird
	g_sendBuffer[3] = 0;	
	// STATUS W1- TBD by Bluebird
	g_sendBuffer[4] = 0;										
	g_sendBuffer[5] = 0;
	
	if(GetDMCmode() != ONLINE_STATE)
		g_sendBuffer[5] |= 0x1;

	// Byte #2 D0: payload is free to accept commands D1: 0- there are no payload fails
	g_sendBuffer[5] |= ((g_curentmode<<2) & 0x7C);
			
	g_sendBuffer[6] = 0;				// Byte #3 R for NUCps	spare	R for AGC ROIps	Iris Reportps
	g_sendBuffer[7] = 0;				// Byte #4

	// D1: 1 == Payload has completed initialization process
	if( GetDMCflag(DMC_ENGINE_CONNECT)) // D1: 
		g_sendBuffer[7] |= 0x02;
	
	// D6: 1 == Hot Blk/Wht Rptps	
	if( GetDMCflag(DMC_BLACK_HOT))
		g_sendBuffer[7] |= 0x40;
	
 	if( GetDMCflag(DMC_ZOOM_X2)) 
		g_sendBuffer[8] =  (unsigned char)((38.67f-(23.6f/2))/18.4)*129;
	else
	if( GetDMCflag(DMC_ZOOM_X4)) 
		g_sendBuffer[8] =  (unsigned char)((38.67f-(23.6f/4))/18.4)*129;
	else
		g_sendBuffer[8] =  (unsigned char)((38.67f-23.6f)/18.4)*129;

	// Pitch Report LSB 0-7 Bit
	g_sendBuffer[9] = ((int)Pitch_Report & 0xFF); 
	g_sendBuffer[10] = Payload_type<<4 | (((int)Pitch_Report >> 8) & 0x0F);	
	// Roll_Report_LSB 0-7 Bit;
 	g_sendBuffer[11] = ((int)Roll_Report & 0xFF); 			
	// Roll_Report_LSB 0-3 Bit;
	g_sendBuffer[12] = ((int)Roll_Report >> 8) & 0x0F; 			
	g_sendBuffer[13] = Payload_corr;

	for (checksum = 0,i=0;i<(18);i++)
		checksum+=g_sendBuffer[i];
	g_sendBuffer[19] = checksum % 256;

	/* write the users command out the serial port */
 	UART0Send(g_sendBuffer, 20);	
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

//vertical = ((float)g_fmaximum_ver_rate*(((float)cmd-512.0f)/511.0f))*cosf(Thetac) - ((float)g_fmaximum_ver_rate*(((float)cmd-512.0f)/511.0f))*sinf(Thetac);	

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
//horizontal = ((float)g_fmaximum_hor_rate*(((float)cmd-512.0f)/511.0f))*sinf(Thetac);		

}

char PointCoordinateMode(unsigned char* msg)
{
	if(GetDMCmode() == ONLINE_STATE)
	{
	
		if( (!GetDMCflag(DMC_POINT_2_COORD)) )			
		{					
	
			SetDMCflag(DMC_POINT_2_COORD,DMC_POINT_2_COORD);							
			SetDMCflag(DMC_BIT_MODE | DMC_CAM_GUIDE | DMC_STOW |  DMC_CENTER_MODE | DMC_PILOT_WINDOW | DMC_RATE | DMC_PTC_CG,0);								

			g_tbstabilizer = true;			
			omega_vertical = (Thetac * RAD2DEG);		
			omega_horizontal = (Phic * RAD2DEG);		
											
										

			return POINT_COORDINATE_MODE;
		}													
	}
	return -1;
}


char HoldCoordinateMode(unsigned char* msg)
{
	if(GetDMCmode() == ONLINE_STATE)
	{
		if( (!GetDMCflag(DMC_RATE)) )
		{	
				SetDMCflag(DMC_RATE,DMC_RATE);						
				SetDMCflag(DMC_BIT_MODE | DMC_STOW | DMC_PILOT_WINDOW |  DMC_CENTER_MODE | DMC_POINT_2_COORD | DMC_CAM_GUIDE | DMC_PTC_CG ,0);																									
				
	
				g_tbstabilizer = true;			
				omega_vertical = (Thetac * RAD2DEG);		
				omega_horizontal = (Phic * RAD2DEG);		
				

				return HOLD_COORDINATE_MODE;				
		}
	}

	return -1;
}

char RateMode(unsigned char* msg)
{
	g_tbstabilizer = true;			

	if(GetDMCmode() == ONLINE_STATE)
	{
		if( (!GetDMCflag(DMC_RATE)) )
		{	
				SetDMCflag(DMC_RATE,DMC_RATE);						
				SetDMCflag(DMC_BIT_MODE | DMC_STOW | DMC_PILOT_WINDOW |  DMC_CENTER_MODE | DMC_POINT_2_COORD | DMC_CAM_GUIDE | DMC_PTC_CG ,0);																									
				
				
				g_tbstabilizer = true;			
				ignorejoystick = false; 	
				
				omega_vertical = (Thetac * RAD2DEG);		
				omega_horizontal = (Phic * RAD2DEG);		

				MotorBrakePositionClean(PITCH_MOTOR);
				MotorBrakePositionClean(ROLL_MOTOR);
				
				return OBSERVATION_MODE;				
		}
	}

	return -1;
}

char PilotWindowMode(unsigned char* msg)
{

	if(GetDMCmode() == ONLINE_STATE)
	{
		if(!GetDMCflag(DMC_PILOT_WINDOW) )
		{
						
			SetDMCflag(DMC_PILOT_WINDOW,DMC_PILOT_WINDOW);	// signal for control center			
			SetDMCflag(DMC_BIT_MODE | DMC_STOW | DMC_RATE |  DMC_CENTER_MODE | DMC_POINT_2_COORD | DMC_CAM_GUIDE | DMC_PTC_CG,0); 																

 			ignorejoystick = true;		
			g_tbstabilizer = false;		
			
			EncoderInit2Point(PITCH_MOTOR,A3906_FORWARD,Motor[PITCH_MOTOR].max_position);		  
			EncoderInit2Point(ROLL_MOTOR,A3906_FORWARD,Motor[ROLL_MOTOR].zero_pos); // SCD Engine encoder 360
			

			return PILOT_WINDOW_MODE;
 		}		
	}	
	return -1;
}



char StowMode(unsigned char* msg)
{
	if(GetDMCmode() == ONLINE_STATE)
	{
		if(!GetDMCflag(DMC_STOW) )
		{
			SetDMCflag(DMC_STOW,DMC_STOW);		
			SetDMCflag(DMC_BIT_MODE | DMC_PILOT_WINDOW | DMC_RATE |  DMC_CENTER_MODE | DMC_POINT_2_COORD | DMC_CAM_GUIDE | DMC_PTC_CG,0); // signal for control center		
			
			ignorejoystick = true;	  
			g_tbstabilizer = false;																
			EncoderInit2Point(PITCH_MOTOR,A3906_FORWARD,1000);		  
			EncoderInit2Point(ROLL_MOTOR,A3906_FORWARD,1000); // SCD Engine encoder 360			


			return STOW_MODE;
 		}
	}
	return -1;
}


void GraphicsOffOn(unsigned char* msg)
{
 	unsigned short cmd;

 		cmd = (msg[3] & 0x20);
	
		switch(cmd)
		{

			case 0x00:


					if( GetDMCflag(DMC_OSD_ON) )
					{				
						SetDMCflag(DMC_OSD_ON,0);
						g_ucvisible_lebel = 0;
 					}


			break;

			case 0x20:


					if( (!GetDMCflag(DMC_OSD_ON)) )
					{
						SetDMCflag(DMC_OSD_ON,DMC_OSD_ON);	
						g_ucvisible_lebel = 1;
 					}


			break;

		
			default:

			break;
			
			}
}

void CameraCalibMode(unsigned char* msg)
{
	//printf("Interpret - CameraCalibMode()@%d \n",__LINE__);
}

void ScanMode(unsigned char* msg)
{
	//printf("Interpret - ScanMode()@%d \n",__LINE__);
}

void DriftCalibMode(unsigned char* msg)
{
	//printf("Interpret - DriftCalibMode()@%d \n",__LINE__);
}

void ScaleCalibMode(unsigned char* msg)
{
	//printf("Interpret - ScaleCalibMode()@%d \n",__LINE__);
}

void GimbalCalibMode(unsigned char* msg)
{
	//printf("Interpret - GimbalCalibMode()@%d \n",__LINE__);
}

char BitMode(unsigned char* msg)
{
	SetDMCflag(DMC_BIT_MODE,DMC_BIT_MODE);

	return BIT_MODE_CON; 
}

void FlirCalibMode(unsigned char* msg)
{
	//printf("Interpret - FlirCalibMode()@%d \n",__LINE__);
}

void PositionMode(unsigned char* msg)
{
	//printf("Interpret - PositionMode()@%d \n",__LINE__);
}

void VHCalibMode(unsigned char* msg)
{
	//printf("Interpret - VHCalibMode()@%d \n",__LINE__);
}

void TestScanMode(unsigned char* msg)
{
	//printf("Interpret - TestScanMode()@%d \n",__LINE__);
}

void GpsNavInitMode(unsigned char* msg)
{
	//printf("Interpret - GpsNavInitMode()@%d \n",__LINE__);
}

void NavInitMode(unsigned char* msg)
{
	//printf("Interpret - NavInitMode()@%d \n",__LINE__);
}

void Mode17Mode(unsigned char* msg)
{
	//printf("Interpret - Mode17Mode()@%d \n",__LINE__);
}

void GyrocompassMode(unsigned char* msg)
{
	//printf("Interpret - GyrocompassMode()@%d \n",__LINE__);
}

void GimbalCheckMode(unsigned char* msg)
{
	//printf("Interpret - GimbalCheckMode()@%d \n",__LINE__);
}

void GyroCalibMode(unsigned char* msg)
{
	//printf("Interpret - GyroCalibMode()@%d \n",__LINE__);
}

void ObstructionTest1Mode(unsigned char* msg)
{
	//printf("Interpret - ObstructionTest1Mode()@%d \n",__LINE__);
}

void ObstructionTest2Mode(unsigned char* msg)
{
	//printf("Interpret - ObstructionTest2Mode()@%d \n",__LINE__);
}

void ObstructionTest3Mode(unsigned char* msg)
{
	//printf("Interpret - ObstructionTest3Mode()@%d \n",__LINE__);
}

void BoresightCalibMode(unsigned char* msg)
{
	//printf("Interpret - BoresightCalibMode()@%d \n",__LINE__);
}

char CenterMode(unsigned char* msg)
{
	if(GetDMCmode() == ONLINE_STATE)
	{
		if(!GetDMCflag(DMC_CENTER_MODE) )
		{
						
			SetDMCflag(DMC_CENTER_MODE,DMC_CENTER_MODE);	// signal for control center			
			SetDMCflag(DMC_BIT_MODE | DMC_STOW | DMC_RATE | DMC_POINT_2_COORD | DMC_CAM_GUIDE | DMC_PTC_CG,0); 																

			ignorejoystick = true;	  
			g_tbstabilizer = false;		

			ctrl.state[TEXT_LABEL] = HIDE_SHOW_LABEL; 
			ctrl.time[TEXT_LABEL] = GetSysTickCount();
			
			EncoderInit2Point(PITCH_MOTOR,A3906_FORWARD,Motor[PITCH_MOTOR].zero_pos);		  
			EncoderInit2Point(ROLL_MOTOR,A3906_FORWARD,Motor[ROLL_MOTOR].zero_pos); // SCD Engine encoder 360
							
			return CENTER_MODE;
 		}		
	}	
	return -1;

}

void ImageFlipMode(unsigned char* msg)
{
	//int ret;
	//ret = ImageFlipVerticall();
	//if (ret==-1)printf("Interpret - ERROR ImageFlipMode()@%d \n",__LINE__);
}

void GimbalZeroMode(unsigned char* msg)
{
	//printf("Interpret - GimbalZeroMode()@%d \n",__LINE__);
}

void RatePositionMode(unsigned char* msg)
{
	//printf("Interpret - RatePositionMode()@%d \n",__LINE__);
}

void Mode29Mode(unsigned char* msg)
{
	//printf("Interpret - Mode29Mode()  @%d \n",__LINE__);
}

void CommlossMode(unsigned char* msg)
{
	//printf("Interpret - CommlossMode()@%d \n",__LINE__);
}

void MaintenanceMode(unsigned char* msg)
{
	//printf("Interpret - MaintenanceMode()@%d \n",__LINE__);
}	


// Interpreting the data of the mode
void InterpretCmdMode(unsigned char *bdData1)
{	
//	printf("Interpret - bdData[3] = 0x%02X   @%d \n",bdData1[3],__LINE__);
//	printf("Interpret - mode = 0x%02X   @%d \n",mode,__LINE__);
	
	char mode = bdData1[3] & 0x1F;

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
			else if ( mode == STOW_MODE) //4// 
				g_curentmode = StowMode(bdData1);
			else if ( mode == CENTER_MODE) //5// 
				g_curentmode = CenterMode(bdData1);			
			else if ( mode == BIT_MODE_CON) //10
				g_curentmode = BitMode(bdData1);
			
		}
		
		ObservationMode(bdData1);
		
		#if 0
		else if ( mode == POINT_COORDINATE_MODE) //1//
			{
				PointCoordinateMode(bdData1);
			}
		else if (mode == HOLD_COORDINATE_MODE) //2//
			{
				HoldCoordinateMode(bdData1);
			}
		else if ( mode == PILOT_WINDOW_MODE) //3// 
			{
				PilotWindowMode(bdData1);
			}
		else if ( mode == STOW_MODE) //4// 
			{
				StowMode(bdData1);
			}
		else if ( mode == CAMERA_CALIB_MODE) //5
			{
				CameraCalibMode(bdData1);
			}
		else if ( mode == SCAN_MODE) //6
			{
				ScanMode(bdData1);
			}
		else if ( mode == DRIFT_CALIB_MODE) //7
			{
				DriftCalibMode(bdData1);
			}
		else if ( mode == SCALE_CALIB_MODE) //8
			{
				ScaleCalibMode(bdData1);
			}
		else if (mode == GIMBAL_CALIB_MODE) //9
			{
				GimbalCalibMode(bdData1);
			}
		else if ( mode == BIT_MODE_CON) //10
			{
				BitMode(bdData1);
			}
		else if ( mode == FLIR_CALIB_MODE) //11
			{
				FlirCalibMode(bdData1);
			}
		else if (mode == POSITION_MODE) //12
			{
				PositionMode(bdData1);	
			}
		else if ( mode == V_H_CALIB_MODE) //13
			{
				VHCalibMode(bdData1);
			}
		else if (mode == TEST_SCAN_MODE) //14 
			{
				TestScanMode(bdData1);
			}
		else if (mode == GPS_NAV_INIT_MODE) //15
			{
				GpsNavInitMode(bdData1);
			}
		else if (mode == NAV_INIT_MODE) //16
			{
				NavInitMode(bdData1);
			}
		else if (  mode == MODE_17_MODE) //17
			{
				Mode17Mode(bdData1);
			}
		else if (  mode == GYROCOMPASS_MODE) //18
			{
				GyrocompassMode(bdData1);
			}
		else if (  mode == GIMBAL_CHECK_MODE) //19
			{
				GimbalCheckMode(bdData1);
			}
		else if (  mode == GYRO_CALIB_MODE) //20
			{
				GyroCalibMode(bdData1);
			}
		else if (  mode == OBSTRUCTION_TEST1_MODE) //21
			{
				ObstructionTest1Mode(bdData1);
			}
		else if (  mode == OBSTRUCTION_TEST2_MODE) //22
			{
				ObstructionTest2Mode(bdData1);
			}
		else if (  mode == OBSTRUCTION_TEST3_MODE) //23
			{
				ObstructionTest3Mode(bdData1);
			}
		else if (  mode == BORESIGHT_CALIB_MODE) //24
			{
				BoresightCalibMode(bdData1);
			}
		else if (  mode == CENTER_MODE) //25
			{
				CenterMode(bdData1);
			}
		else if (  mode == IMAGE_FLIP_MODE) //26
			{
				ImageFlipMode(bdData1);
			}
		else if (  mode == GIMBAL_ZERO_MODE) //27
			{
				GimbalZeroMode(bdData1);
			}
		else if (  mode == RATE_POSITION_MODE) //28
			{
				RatePositionMode(bdData1);
			}
		else if (  mode == MODE_29_MODE) //29
			{
				Mode29Mode(bdData1);
			}
		else if (  mode == COMMLOSS_MODE) //30
			{
				CommlossMode(bdData1);
			}
		else if (  mode == MAINTENANCE_MODE) //31
			{
				MaintenanceMode(bdData1);
			}
		else
			{			
				// ERROR
				//printf("Interpret.h -->   Could not interpret mode  @%d \n",__LINE__);
			}
		#endif
	}
}

void UavLatitude(unsigned char* msg)
{
	unsigned short cmd;
	unsigned short mode;	
	unsigned short idx = 1;
	unsigned short i = 0;
	
	//mode = (msg[6] & 0x03);
		
	//for(i=1;i<3;i++)
	//{
			
		cmd = (msg[6] & 0x03);//idx);
		
		switch(cmd)
		{

			case 0x01: // ZOOM OUT

				//if(cmd)
				//{
					if ((WORD)(GetSysTickCount() - ctrl.time[TIME_ZOOM]) >= (WORD)1000) //-- every 1 sec
					{						
				
					  if(GetDMCmode() == ONLINE_STATE)
					  {
						if(GetDMCflag(DMC_ZOOM_X4))
						{

							SetDMCflag(DMC_ZOOM_X2,DMC_ZOOM_X2);	 // signal for control center					 
							SetDMCflag(DMC_DISABLE_ZOOM | DMC_ZOOM_X4,0);
							
							g_fmaximum_ver_rate = A_FAST_VER*20.0f+B_FAST_VER;  //// PITCH /////
							g_fmaximum_hor_rate = A_HOR*20.0f+B_HOR;			//// ROLL /////			
							ctrl.state[TIME_DIS] = DIS_SENT_MESSAGE;							
							ctrl.time[TIME_ZOOM] = GetSysTickCount();						
							putDMCmode(UPDATE_DISABLE_ZOOM);					// ZOOM X2		
						}
						else
						if(GetDMCflag(DMC_ZOOM_X2))
						{

							SetDMCflag(DMC_DISABLE_ZOOM,DMC_DISABLE_ZOOM);	 // signal for control center					 
							SetDMCflag(DMC_ZOOM_X2 | DMC_ZOOM_X4,0);
								
							g_fmaximum_ver_rate = A_FAST_VER*30.0f+B_FAST_VER;	//// PITCH /////
							g_fmaximum_hor_rate = A_HOR*30.0f+B_HOR;			//// ROLL /////
				
							ctrl.state[TIME_DIS] = DIS_SENT_MESSAGE;							
							ctrl.time[TIME_ZOOM] = GetSysTickCount();							
							putDMCmode(UPDATE_DISABLE_ZOOM); 			// ZOOM Out
							
						}
					  }
												
					}		
					
				//}

				
			break;

			case 0x02: // ZOOM IN

				//if(cmd)
				//{
 					if ((WORD)(GetSysTickCount() - ctrl.time[TIME_ZOOM]) >= (WORD)1000) //-- every 3 sec
					{
						if(GetDMCmode() == ONLINE_STATE)
						{						
							if(GetDMCflag(DMC_DISABLE_ZOOM))
							{

								SetDMCflag(DMC_ZOOM_X2,DMC_ZOOM_X2);	 // signal for control center					 
								SetDMCflag(DMC_DISABLE_ZOOM | DMC_ZOOM_X4,0);

								g_fmaximum_ver_rate = A_FAST_VER*20.0f+B_FAST_VER;
								g_fmaximum_hor_rate = A_HOR*20.0f+B_HOR;		
								ctrl.state[TIME_DIS] = DIS_SENT_MESSAGE;							
								ctrl.time[TIME_ZOOM] = GetSysTickCount();															
								putDMCmode(UPDATE_DISABLE_ZOOM);					// ZOOM X2
							
							}
							else
							if(GetDMCflag(DMC_ZOOM_X2) )
							{

								SetDMCflag(DMC_ZOOM_X4,DMC_ZOOM_X4);	 // signal for control center					 
								SetDMCflag(DMC_ZOOM_X2 | DMC_DISABLE_ZOOM,0);

								g_fmaximum_ver_rate = A_FAST_VER*16.0f+B_FAST_VER;
								g_fmaximum_hor_rate = A_HOR*16.0f+B_HOR;								
								ctrl.state[TIME_DIS] = DIS_SENT_MESSAGE;							
								ctrl.time[TIME_ZOOM] = GetSysTickCount();								
								putDMCmode(UPDATE_DISABLE_ZOOM);					// ZOOM X4
							}
						}													
					}

 				//}


				
			break;
			
			default:

			break;


		}	
	
		//idx<<=1;	
	//}
}


void UavLongitude(unsigned char* msg)
{
 	unsigned short cmd;
 	unsigned short idx = 1;
 	unsigned short i = 0;


		//for(i=0;i<8;i++)
		//{
		
 		cmd = (msg[5] & 0x40);//idx);
	
		//if(cmd)//switch(i)
		//switch(cmd)
		//{

			//case 0x6:

				if(GetDMCmode() == ONLINE_STATE)
				{
					if(cmd)
					{				
						if ((WORD)(GetSysTickCount() - ctrl.time[WHITE_BLACK]) >= (WORD)1000) //-- every 1 sec
						{

								if((!GetDMCflag(DMC_BLACK_HOT)) )
								{							
									SetDMCflag(DMC_BLACK_HOT,DMC_BLACK_HOT);  // signal for control center
									SetDMCflag(DMC_WHITE_HOT,0);
									
									ctrl.time[WHITE_BLACK] = GetSysTickCount();
									ctrl.state[TIME_BH] = BH_SENT_MESSAGE;
									putDMCmode(BLACK_HOT);																			
								}
						}
						
					}
					else
					{
						if ((WORD)(GetSysTickCount() - ctrl.time[WHITE_BLACK]) >= (WORD)1000) //-- every 1 sec
						{
						
								if((!GetDMCflag(DMC_WHITE_HOT)) )	
								{						
									SetDMCflag(DMC_WHITE_HOT,DMC_WHITE_HOT);  // signal for control center
									SetDMCflag(DMC_BLACK_HOT,0);
									
									ctrl.time[WHITE_BLACK] = GetSysTickCount();
									ctrl.state[TIME_BH] = BH_SENT_MESSAGE;
									putDMCmode(BLACK_HOT);	
								}
						}

					}
				}
				
			//break;

			#if 0
			case 0x1:
			
			
					if(cmd)
					{

						if ((WORD)(GetSysTickCount() - ctrl.time[TIME_STABILIZ]) >= (WORD)1000) //-- every 3 sec
						{					
							if(GetDMCmode() == ONLINE_STATE)
							{					
									
								if(!GetDMCflag(DMC_STABILIZE_ON))
								{
									SetDMCflag(DMC_STABILIZE_ON,DMC_STABILIZE_ON);	// signal for control center
									SetDMCflag(DMC_STABILIZE_OFF,0);
									//Stabilize_status = 0x1;
									ctrl.time[TIME_STABILIZ] = GetSysTickCount();
									//putDMCmode(ON_OFF_STABILIZE);	
									putDMCmode(REFRESH_NUC);	
								}																
								else
								if(!GetDMCflag(DMC_STABILIZE_OFF))
								{
									SetDMCflag(DMC_STABILIZE_OFF,DMC_STABILIZE_OFF);  // signal for control center
									SetDMCflag(DMC_STABILIZE_ON,0);									
									//Stabilize_status = 0x0;
									ctrl.time[TIME_STABILIZ] = GetSysTickCount();
									//putDMCmode(ON_OFF_STABILIZE);
									putDMCmode(REFRESH_NUC);
								}
							}
						}
					}
				
			break;
			#endif
			
			//default:

			//break;
			
			//}
		
			//idx<<=1;	
		
		//}
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

void Set_Brightness_Gain(unsigned char* msg)
{
	unsigned short cmd;
	unsigned short mode;	
	unsigned short idx = 1;
	unsigned short i = 0;
	
	//for(i=1;i<3;i++)
	//{
			
		cmd = (msg[4] & 0x03);//idx);
		
		switch(cmd)
		{

			case 0x01: // ZOOM OUT

				if(cmd)
				{
					if ((WORD)(GetSysTickCount() - ctrl.time[CON_BRIT]) >= (WORD)100) //-- every 1 sec
					{						
				
					  if(GetDMCmode() == ONLINE_STATE)
					  {
							m_Contrast_Set.brightness += 1;
				  			if(m_Contrast_Set.brightness > 100)
				  				m_Contrast_Set.brightness = 0;

							ctrl.time[CON_BRIT] = GetSysTickCount();
							putDMCmode(SET_CONTRAST_BRITHNESS); 			// ZOOM Out
							
							
					  	}
												
					}		
					
				}

				
			break;

			case 0x02: // ZOOM IN

				if(cmd)
				{
 					if ((WORD)(GetSysTickCount() - ctrl.time[CON_BRIT]) >= (WORD)100) //-- every 3 sec
					{
						if(GetDMCmode() == ONLINE_STATE)
						{						

								m_Contrast_Set.brightness -= 1;
					  			if(m_Contrast_Set.brightness < 0)
					  				m_Contrast_Set.brightness = 0;

					  			ctrl.time[CON_BRIT] = GetSysTickCount();
								putDMCmode(SET_CONTRAST_BRITHNESS);				
						}													
					}

 				}


				
			break;
			
			default:

			break;


		}	
	
		//idx<<=1;	
	//}

}


void Set_Contrast_Gain(unsigned char* msg)
{
	unsigned short cmd;
	unsigned short mode;	
	unsigned short idx = 1;
	unsigned short i = 0;
	
	mode = (msg[6] & 0x03);
		
	for(i=1;i<3;i++)
	{
			
		cmd = (msg[6] & idx);
		
		switch(cmd)
		{

			case 0x01: // ZOOM OUT

				if(cmd)
				{
					if ((WORD)(GetSysTickCount() - ctrl.time[CON_BRIT]) >= (WORD)100) //-- every 1 sec
					{						
				
					  if(GetDMCmode() == ONLINE_STATE)
					  {
							m_Contrast_Set.contrast -= 1;
				  			if(m_Contrast_Set.contrast < 0)
				  				m_Contrast_Set.contrast = 0;

							ctrl.time[CON_BRIT] = GetSysTickCount();
							putDMCmode(SET_CONTRAST_BRITHNESS); 			// ZOOM Out
							
							
					  	}
												
					}		
					
				}

				
			break;

			case 0x02: // ZOOM IN

				if(cmd)
				{
 					if ((WORD)(GetSysTickCount() - ctrl.time[CON_BRIT]) >= (WORD)100) //-- every 3 sec
					{
						if(GetDMCmode() == ONLINE_STATE)
						{						

								m_Contrast_Set.contrast += 1;
					  			if(m_Contrast_Set.contrast > 100)
					  				m_Contrast_Set.contrast = 100;

					  			ctrl.time[CON_BRIT] = GetSysTickCount();
								putDMCmode(SET_CONTRAST_BRITHNESS);				
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

void	Elevation_gimbal_angle()
{
	//printf("Interpret - Elevation_gimbal_angle()@%d \n",__LINE__);
}


void LOS_Roll(void)
{

	float Los_Roll = Phic;//(Phic * RAD2DEG);
	memcpy(&g_sendBuffer[14],&Los_Roll,sizeof(float));
}

void LOS_Pitch(void)
{

	float LOSPitch = Thetac;//(Thetac* RAD2DEG);
	memcpy(&g_sendBuffer[14],&LOSPitch,sizeof(float));

}

// Interpreting the data of the correlator
void InterpretCorrelatorMode(unsigned char *bdData1)
{

	char CheckingCorr = (bdData1[11] & CHECKING_CORR_NUMBER)>>4;


			if(GetDMCflag(DMC_ENGINE_CONNECT))
			{
				UavLatitude(bdData1);
		 		UavLongitude(bdData1);
				//Set_Brightness_Gain(bdData1);
				//Set_Contrast_Gain(bdData1);
		 		GraphicsOffOn(bdData1); 		
			}

			#if 1
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
			#endif

}


void InterpretCmd(void)
{

		InterpretCmdMode(cpGetMessage(COM0));		
		InterpretCorrelatorMode(cpGetMessage(COM0));						
		ReportData(UART0_BASE);
}
