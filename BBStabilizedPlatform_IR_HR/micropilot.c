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

static long UserTelemetryData[20] = {0};

int mpGetUserTelemetry(char *pData)
{

	int i;
	int txSize = 20;
	int checksum;
	int ret = -1;
	int header_chs = 0xff+0x00;
	char input_data_com1[100];


	while(pData[i] != 0xFE)
	{
		if(pData[i] == 0xFD)
		{
			if(pData[i] == 0x00)	
				input_data_com1[i] = 0xFD;
			else
			if(pData[i] == 0x01)							
				input_data_com1[i] = 0xFE;								
			else
			if(pData[i] == 0x02)							
				input_data_com1[i] = 0xFF;							 							

			
		}
		else
			input_data_com1[i] = pData[i];
								
	}
	

	if((input_data_com1[0] == 0xff) && (input_data_com1[1] == 0x00) && (input_data_com1[2] != 0x05))
	{
				
		 //===User Telemetry Data decripted =========
		 //==========================================
		 // 5Hz Data
		 UserTelemetryData[0] = htonl(input_data_com1[3]);

		 UserTelemetryData[1] = input_data_com1[7]<<24;
		 UserTelemetryData[1] |= (input_data_com1[8]<<16);
		 UserTelemetryData[1] |= (input_data_com1[9]<<8);
		 UserTelemetryData[1] |= (input_data_com1[10]);
		 UserTelemetryData[1] = htonl(UserTelemetryData[1]);
		 
		 UserTelemetryData[2] = input_data_com1[11]<<24;
		 UserTelemetryData[2] |= (input_data_com1[12]<<16);
		 UserTelemetryData[2] |= (input_data_com1[13]<<8);
		 UserTelemetryData[2] |= (input_data_com1[14]);
		 UserTelemetryData[2] = htonl(UserTelemetryData[2]);
		 
		 UserTelemetryData[3] = htonl(input_data_com1[15]);
		 
		 UserTelemetryData[4] = htonl(input_data_com1[19]);
		 
		 UserTelemetryData[5] = htonl(input_data_com1[23]);
		 
		 UserTelemetryData[6] = htonl(input_data_com1[27]);
		 
		 UserTelemetryData[7] = htonl(input_data_com1[31]);
		 
		 UserTelemetryData[8] = htonl(input_data_com1[35]);
		 
		 UserTelemetryData[9] = htonl(input_data_com1[39]);
		 
		 UserTelemetryData[10] = htonl(input_data_com1[43]);
		 
		 UserTelemetryData[11] = htonl(input_data_com1[47]);
		 
		 UserTelemetryData[12] = htonl(input_data_com1[51]);
		 
		 UserTelemetryData[13] = htonl(input_data_com1[55]);

		 UserTelemetryData[14] = input_data_com1[59];
		 UserTelemetryData[14] |= (input_data_com1[60]<<8);
		 UserTelemetryData[14] |= (input_data_com1[61]<<16);
		 UserTelemetryData[14] |= (input_data_com1[62]<<24);
		 UserTelemetryData[14] = htonl(UserTelemetryData[14]);
		 
		 UserTelemetryData[15] = htonl(input_data_com1[63]);

		 // 1Hz Data
		 switch (input_data_com1[2]){
			case 0:
				 // id=0
				 UserTelemetryData[16] = htonl(input_data_com1[67]); // Camera Target Enable. 0:Off,1:Absolute,2:Relative
				 UserTelemetryData[17] = htonl(input_data_com1[71]); //Camera Target EAST Co-ordinates
				 UserTelemetryData[18] = htonl(input_data_com1[75]); //Camera Target North Co-ordinates
				 UserTelemetryData[19] = htonl(input_data_com1[79]); //Camera Target UP Co-ordinates 
				 break;
			case 1:
				 UserTelemetryData[16] = htonl(input_data_com1[67]);
				 UserTelemetryData[17] = htonl(input_data_com1[71]);
				 UserTelemetryData[18] = htonl(input_data_com1[75]);
				 UserTelemetryData[19] = htonl(input_data_com1[79]);
				 break;
			case 2:
				 UserTelemetryData[16] = htonl(input_data_com1[67]);
				 UserTelemetryData[17] = htonl(input_data_com1[71]);
				 UserTelemetryData[18] = htonl(input_data_com1[75]);
				 UserTelemetryData[19] = htonl(input_data_com1[79]);
				 break;
			case 3:
				 UserTelemetryData[16] = htonl(input_data_com1[67]);
				 UserTelemetryData[17] = htonl(input_data_com1[71]);
				 UserTelemetryData[18] = htonl(input_data_com1[75]);
				 UserTelemetryData[19] = htonl(input_data_com1[79]);

				 break;
			case 4:
				 UserTelemetryData[16] = htonl(input_data_com1[67]);
				 UserTelemetryData[17] = htonl(input_data_com1[71]);
				 UserTelemetryData[18] = htonl(input_data_com1[75]);				 
				 UserTelemetryData[19] = htonl(input_data_com1[79]);		 
				 break;


		}


	}
	else
		return -1;

return 1;	
}

#if 0
int mpGetStandardTelemetry(MPSTDTELEMETRYDATA *StdTlm)
{

	int i=0;
	int txSize = 20;
	int checksum;
	int ret = -1;
	int header_chs = 0xff+0x00;

	//printf("mpGetStandardTelemetry input_data_com1[2] %d @ %d\n",input_data_com1[2],__LINE__);

	if((input_data_com1[0] == 0xff) && (input_data_com1[1] == 0x00) && (input_data_com1[2] == 0x05))
	{
		//printf("mpGetStandardTelemetry input_data_com1[3] %d @ %d\n",input_data_com1[3],__LINE__);
		
		if((input_data_com1[3] == 0x00))
		{

			//printf("mpGetStandardTelemetry 00 Data -> ");
				//for(i=4;i<20;i++)
			//printf("[%02X]",input_data_com1[i]);
			//printf("\n");

			i=4;
			/*!< @brief Autopilot Pitch (Radians*1024 - integer scaled radians) */
			StdTlm->tlmPitch=input_data_com1[i++];	  									
			/*!< @brief Autopilot Roll (Radians*1024 - integer scaled radians) */			
			StdTlm->tlmRoll=input_data_com1[i++]; 	  									
			
			/*!< @brief Speed (ft/s)				*/
			StdTlm->speed = input_data_com1[i++];									
			/*!< @brief GPS speed (ft/s)			*/
			StdTlm->gpsSpeed = input_data_com1[i++];									

			/*!< @brief Status bit fields */
			StdTlm->status=input_data_com1[i++];	  	
			/*!< @brief Status bit fields */
			StdTlm->status2=input_data_com1[i++]; 	  									

			/*!< @brief Instruction pointer position */
			StdTlm->ipStep=input_data_com1[i++];
			StdTlm->ipStep <<= 8;
			StdTlm->ipStep|= input_data_com1[i++]; 		
			
			/*!< @brief Target speed (ft/s - feet per second) */
			StdTlm->targetSpeed_fps=input_data_com1[i++];
			StdTlm->targetSpeed_fps <<= 2;
			StdTlm->targetSpeed_fps|=input_data_com1[i++];
			
			/*!< @brief GCS id of the owner of this UAV ( 0 = no owner ) */
			StdTlm->ownerGcsId=input_data_com1[i++];									
			/*!< @brief Status of the LRC */
			StdTlm->lrcStatus=input_data_com1[i++];										

			// Reserved
			input_data_com1[i++]; 														
			input_data_com1[i++];

			
			
		}
		else
		if((input_data_com1[3] == 0x01))
		{

			i=4;
			/*!< @brief Autopilot Pitch (Radians*1024 - integer scaled radians) */
			StdTlm->tlmPitch=input_data_com1[i++];	  					
			/*!< @brief Autopilot Roll (Radians*1024 - integer scaled radians) */
			StdTlm->tlmRoll=input_data_com1[i++]; 	  									

			/*!< @brief Altitude (-8*ft - integer scaled negative feet) */
			StdTlm->alt=input_data_com1[i++]<<3;
			//printf("input_data_com1[6] == [%08X] StdTlm->alt == [%08X]\n",input_data_com1[6],StdTlm->alt);
			StdTlm->alt+=(input_data_com1[i++]<<11);	
			//StdTlm->alt = htonl(StdTlm->alt);
			//printf("StdTlm->alt == [%08X]\n",StdTlm->alt);
			if(StdTlm->alt > 0x40000)
			{
				StdTlm->alt -=0x80000;
			}
			
			/*!< @brief Servo throttle position. (FINE-SERVO units mapped to 0..255) */			
			StdTlm->sTh=input_data_com1[i++];	

			/*!< @brief Servo battery voltage (Volts*100 - integer scaled Volts) */
			StdTlm->sbatV=input_data_com1[i++];	  									
			StdTlm->sbatV <<=3;
			//printf("StdTlm->sbatV == [%08X]\n",StdTlm->sbatV);
			
			/*!< @brief Instruction being executed */
			StdTlm->ipCmd = input_data_com1[i++];	
			StdTlm->ipCmd <<= 8;
			StdTlm->ipCmd |= input_data_com1[i++];	  
			StdTlm->ipCmd = htonl(StdTlm->ipCmd);
			
			/*!< @brief Target altitude (-8*ft - integer scaled negative feet) */		
			StdTlm->targetAlt_ft=input_data_com1[i++]<<3;	
			StdTlm->targetAlt_ft+=(input_data_com1[i++]<<11);
			//StdTlm->targetAlt_ft = htonl(StdTlm->targetAlt_ft);
			//printf("StdTlm->targetAlt_ft == [%08X]\n",StdTlm->targetAlt_ft);
			
			if(StdTlm->targetAlt_ft > 0x40000)
			{
				StdTlm->targetAlt_ft -=0x80000;
			}
			
			/*!< @brief GPS altitude of the plane (-8*ft - integer scaled negative feet) */
			StdTlm->gpsAlt=input_data_com1[i++];	
			StdTlm->gpsAlt <<= 8;
			StdTlm->gpsAlt|=input_data_com1[i++];	  								
			StdTlm->gpsAlt = htonl(StdTlm->gpsAlt);
			
			// Reserved	2 Byte
			input_data_com1[i++]; 														
			input_data_com1[i++]; 														

			

		}
		else
		if((input_data_com1[3] == 0x02))
		{

			i=4;
			/*!< @brief Autopilot Pitch (Radians*1024 - integer scaled radians) */
			StdTlm->tlmPitch=input_data_com1[i++];	  					
			/*!< @brief Autopilot Roll (Radians*1024 - integer scaled radians) */
			StdTlm->tlmRoll=input_data_com1[i++]; 	  									

			/*!< @brief GPS Longitude (Radians*500000000 - integer scaled radians) */
		
			StdTlm->e =input_data_com1[i++]<< 8;
			//StdTlm->e <<= 8;
			StdTlm->e |=input_data_com1[i++]<< 8;
			//StdTlm->e <<= 8;			
			StdTlm->e |=input_data_com1[i++]<< 8;
			//StdTlm->e <<= 8;						
			StdTlm->e |=input_data_com1[i++];

			StdTlm->e = htonl(StdTlm->e);
			//StdTlm->e/= 5.0e8;

			/*!< @brief Pattern invoked if applicable */
			StdTlm->patternId=input_data_com1[i++];

			/*!< @brief Waypoint Version (incremented on waypoint move) */
			StdTlm->waypointversion=input_data_com1[i++];
			StdTlm->waypointversion<<=8;
			StdTlm->waypointversion|=input_data_com1[i++];
			StdTlm->waypointversion = htonl(StdTlm->waypointversion);
			
			/*!< @brief GPS Latitude (Radians*500000000 - integer scaled radians) */
			StdTlm->n =input_data_com1[i++]<< 8;
			//StdTlm->n <<= 8;
			StdTlm->n |=input_data_com1[i++]<< 8;
			//StdTlm->n <<= 8;			
			StdTlm->n |=input_data_com1[i++]<< 8;
			//StdTlm->n <<= 8;						
			StdTlm->n |=input_data_com1[i++];
			StdTlm->n = htonl(StdTlm->n);
			//StdTlm->n/= 5.0e8;

			/*!< @brief Disable pattern origin movement for certain command conditions*/
			StdTlm->disableNewOriginSet=input_data_com1[i++];
			
						
		}	
		else
		if((input_data_com1[3] == 0x03))
		{

	
			i=4;
			/*!< @brief Autopilot Pitch (Radians*1024 - integer scaled radians) */
			StdTlm->tlmPitch=input_data_com1[i++];	  					
			/*!< @brief Autopilot Roll (Radians*1024 - integer scaled radians) */
			StdTlm->tlmRoll=input_data_com1[i++]; 	  									

			/*!< @brief GPS Latitude (Radians*500000000 - integer scaled radians) */
			StdTlm->n =input_data_com1[i++]<< 8;
			//StdTlm->n <<= 8;
			StdTlm->n |=input_data_com1[i++]<< 8;
			//StdTlm->n <<= 8;			
			StdTlm->n |=input_data_com1[i++]<< 8;
			//StdTlm->n <<= 8;						
			StdTlm->n |=input_data_com1[i++];
			StdTlm->n = htonl(StdTlm->n);

			/*!< @brief Failure pattern invoked if applicable */
			StdTlm->failureId=input_data_com1[i++];
			
			/*!< @brief Target heading (degrees*100 - integer scaled degrees) 0..360 degrees */
			StdTlm->targetHeading_deg=input_data_com1[i++];
			StdTlm->targetHeading_deg <<= 8;
			StdTlm->targetHeading_deg|=input_data_com1[i++];
			StdTlm->targetHeading_deg = htonl(StdTlm->targetHeading_deg);
			
			/*!< @brief GCS id of the owner of this UAV ( 0 = no owner ) */
			StdTlm->ownerGcsId=input_data_com1[i++];

			// Reserved	4 Byte
			input_data_com1[i++];
			input_data_com1[i++];
			input_data_com1[i++];
			input_data_com1[i++];


		}	
		else
		if((input_data_com1[3] == 0x04))
		{
		
			i=4;
			/*!< @brief Autopilot Pitch (Radians*1024 - integer scaled radians) */
			StdTlm->tlmPitch=input_data_com1[i++];	  					
			/*!< @brief Autopilot Roll (Radians*1024 - integer scaled radians) */
			StdTlm->tlmRoll=input_data_com1[i++]; 	  									

			/*!< @brief Altitude 1st derivative (-8*ft/s - integer scaled negative feet per second ) */
			StdTlm->altDot=input_data_com1[i++];

			/*!< @brief Target heading (degrees*100 - integer scaled degrees) 0..360 degrees */
			StdTlm->targetHeading_deg=input_data_com1[i++];

			/*!< @brief Main battery voltage (Volts*100 - integer scaled Volts) */
			StdTlm->batV=input_data_com1[i++];

			/*!< @brief Last error code (see mperror.h for possible error code values) */
			StdTlm->err=input_data_com1[i++];

			/*!< @brief Event warning or non-fatal error status from autopilot */
			StdTlm->mEvent=input_data_com1[i++];

			// Reserved	7 Byte
			input_data_com1[i++];
			input_data_com1[i++];
			input_data_com1[i++];
			input_data_com1[i++];
			input_data_com1[i++];
			input_data_com1[i++];
			input_data_com1[i++];
			
			
		}		
		else
			return -1;

	}
	else
		return -1;

return 1;
}
#endif
