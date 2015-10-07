
//*****************************************************************************
//
// comproc.c - Example for reading data from and writing data to the UART in
//               an interrupt driven fashion.
//
// Copyright (c) 2005-2009 Luminary Micro, Inc.  All rights reserved.
// Software License Agreement
// 
// Luminary Micro, Inc. (LMI) is supplying this software for use solely and
// exclusively on LMI's microcontroller products.
// 
// The software is owned by LMI and/or its suppliers, and is protected under
// applicable copyright laws.  All rights are reserved.  You may not combine
// this software with "viral" open-source software in order to form a larger
// program.  Any use in violation of the foregoing restrictions may subject
// the user to criminal sanctions under applicable laws, as well as to civil
// liability for the breach of the terms and conditions of this license.
// 
// THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED
// OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
// MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.
// LMI SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR
// CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
// 
// This is part of revision 4201 of the EK-LM3S811 Firmware Package.
//
//*****************************************************************************
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>

#include <math.h>
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
#include "SysTick.h"
#include "Interpret.h"
#include "comproc.h"
#include "Interpret.h"
#include "modeid.h"
#include "globals.h"
#include "reg.h"
#include "led.h"
#include "uart_echo.h"
#include "A3906.h"
#include "rmb20d01.h"

//*****************************************************************************
//
// The value that is to be modified ctrl
//
//*****************************************************************************
ctrl_type ctrl = {0};

//*****************************************************************************
//
// The value that is to be modified
//
//*****************************************************************************
unsigned short DMCmode = INITIALIZATION_UNIT;

char status_shutter[9] ={ 0x00, 0xA5, 0x90, 0x20, 0x20, 0x20,0x20, 0x20, 0x68};
char set_temperature[8] = { 0xA4, 0x90, 0x2C, 0x10, 0x17, 0x10,  0x00, 0x50};		
char close_shutter[8] = { 0xA4, 0x90, 0x17, 0x10, 0x00, 0x10,  0x00, 0x50}; 		
char open_shutter[8] =	{ 0xA4, 0x90, 0x17, 0x10, 0x01, 0x10,  0x00, 0x50}; 

unsigned long lasttm;
unsigned char g_cCount = 0;

BYTE Stabilize_status = 0x0;

ComProcBuffer cpib[2]; 

extern unsigned long t_ulFlags;
extern ctrl_type ctrl;

tBoolean zoom_in_out = false;
BYTE optical_zoom=0;

PCBYTE  cmsg, emsg;

OSD_TITEL_ITEM_S Osd_Titel_Item;
MINTRON_TXRX_S 	 LensZoomWrite;
MINTRON_TXRX_S 	 AgcSensLevel;
unsigned short 	 target_pos = 0x1000;
unsigned short 	 Agc_level = 3;
unsigned short 	 Sens_level = 0;

unsigned char message[2][COM_PROC_BUF_LEN]; // receiver message buffer
unsigned char BbTxBuffer[TX_MSG_SIZE];
LabelCmd LabelCmdMsg[MAX_LABEL_LENGTH];

typedef WORD (* ModeType)(WORD);

const ModeType ModeVec[]  =	
{
// received commands from Control Center (via communication):
    NullMode,					// 			         		0x00
    NullMode,					// 			         		0x01
	OnlineState,				// ONLINE_STATE (stop)		0x02
	NullMode,					//						0x03 
	NullMode,					//						0x04
    NullMode,					// 			         		0x05 
	NullMode,					//						0x06
	NullMode,					//						0x07
	NullMode,					//						0x08
    NullMode,					// 			         		0x09 
	NullMode,					//						0x0A
	NullMode,					//						0x0B 
	NullMode,					//						0x0C
    NullMode,					// 			         		0x0D 
	NullMode,					//						0x0E
	NullMode,					//						0x0F	
	OfflineState,				// OFFLINE_STATE (stop) 		0x10
	NullMode,					//						0x11 
	NullMode,					//						0x12
    NullMode,					// 			         		0x13 
	NullMode,					//						0x14
	NullMode,					//						0x15 
	NullMode,					//						0x16
    NullMode,					// 			         		0x17 
	NullMode,					//						0x18
	NullMode,					//						0x19 
	NullMode,					//						0x1A	
	NullMode,					//						0x1B
	NullMode,					//						0x1C	
	Set_AGC_Level,				// SET_AGC_LEVEL			0x1D	
	Set_SENS_Level,				// SET_SENS_LEVEL			0x1E
	Set_OSD_On,					// SET_OSD_ON			0x1F	
	Set_OSD_Off,				// SET_OSD_OFF			0x20
	Mintron_Zoom_In_Out, 		// MINTRON_ZOOM_IN_OUT	0x21
	Mintron_Read_Zoom_Position, // MINTRON_READ_ZOOM		0x22
	NullMode,					// OSD_TITEL_ITEM			0x23
	Set_OSD_Position,			// OSD_TITEL_POSITION		0x24
	Zoom_Mag_On,				// ZOOM_MAG_ON			0x25
	Zoom_Mag_Off,				// ZOOM_MAG_OFF			0x26
	Set_Mark_On,				// SET_MARK_ON			0x27
	Set_Mark_Off,				// SET_MARK_OFF			0x28
	InitializationUnit			// INITIALIZATION_UNIT		0x29

};

#define MODE_NUM (sizeof(ModeVec) / sizeof(ModeType))

void init_ctrl(void)
{
  memset(&LensZoomWrite,0,sizeof(LensZoomWrite));
  LensZoomWrite.stx = 0x2;
  LensZoomWrite.write = 0x21;
  LensZoomWrite.data[12] = 0x00;
  LensZoomWrite.etx = 0x03;  
  ctrl.state[MIN_ZOOM] = 0;
  
  ctrl.time[MAIN] = 0;
  ctrl.time[TIME_ZOOM] = 0;
  
  ctrl.time[ONLINE] = g_ulTickCount;  
  ctrl.time[MES_TIME] = g_ulTickCount; 
  ctrl.time[WHITE_BLACK] = g_ulTickCount;
  ctrl.time[TIME_STABILIZ] = g_ulTickCount;
  ctrl.time[TIME_STOW] = g_ulTickCount;
  ctrl.time[TIME_PILOT] = g_ulTickCount;
  ctrl.time[TIME_RATE] = g_ulTickCount;
  ctrl.time[TIME_BIT] = g_ulTickCount;

  

  ctrl.state[UPDATE_2_P] = DISABLED_CIC_2_P;
  ctrl.state[ACT_4] = SET_TEMPERATURE;
  ctrl.state[TEXT_LABEL] = HIDE_SHOW_LABEL; 				  
  
  ctrl.state[ACT_5] = 0;
  ctrl.main_count = 0;
  ctrl.try[ACT_11]= 0;
  ctrl.try[OFFLINE]= 0;
  ctrl.try[ONLINE] = 0;
  ctrl.try[TIME_ZOOM2] = 0;
  ctrl.try[TIME_ZOOM4] = 0;  
  ctrl.try[TIME_AGC_SENS] = 0;  

}


void cpReset(unsigned char cid)
{
   cpib[cid].checksum = 0;
   cpib[cid].idx= 0;
   cpib[cid].len=0;
   cpib[cid].state = 0;
   cpib[cid].ready = 0;
   cpib[cid].mresp = 0;
   cpib[cid].bufidx = 0;
   cpib[cid].crc16 = 0;
   message[cid][MSG_CMD_MSB] = 0;
   message[cid][MSG_CMD_LSB] = 0;
}

/*
 * FUNCTION: CRC16 - Perform CRC16 algorith on input data
 *           and updates cpib[cid].crc16 var.
 * PARAM: data - received data
 */
void CRC16(BYTE cid, HWORD data)
{
 
   static const HWORD oddparity[16] = { 0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0 };

   data = (data ^ cpib[cid].crc16 ) & 0x00ff;
   cpib[cid].crc16 >>= 8;

   if (oddparity[data & 0xf] ^ oddparity[data >> 4])
      cpib[cid].crc16 ^= 0xc001;

   data <<= 6;
   cpib[cid].crc16 ^= data;
   data <<= 1;
   cpib[cid].crc16 ^= data;
		 
}

/*
 * FUNCTION: pCheckCRC16 - Returns non-zero value for crc16 OK
 */
HWORD cpCheckCRC16(BYTE cid)
{
   return (! cpib[cid].crc16) ;
}

/*
 * FUNCTION: pCheckCRC16 - Returns  value for crc16 OK
 */
HWORD cpGetCRC16(BYTE cid)
{
   return cpib[cid].crc16;
}

/*
 * FUNCTION: pCheckCRC16 - Returns  value for crc16 OK
 */
void cpClearCRC16(BYTE cid)
{
  cpib[cid].crc16 = 0x0000;
}

/*
 * FUNCTION:  COM1 MintronMessageLoop - Main Com Proc message processing loop
 * RETURN: Non-zero of message detected  
 */
int
MintronMessageLoop(unsigned long ulBase,unsigned char cid)
{
   char ch;  
   //int i = 0;
   unsigned short Checksum = 0;
   //unsigned char*  cmsg, emsg;
   
   /*
    * If there is byte in comm. stream 
    */
   do 
   {
	  //
	  // Prompt for text to be entered.
	  //
   
       //ch = UARTCharGet(ulBase);
      ch = UARTCharGetNonBlocking(ulBase);	  	  
	  	  
      message[cid][cpib[cid].idx++] = ch;

     /*
      * Overflow check
      */ 
      if (cpib[cid].idx >= COM_PROC_BUF_LEN)
      {
         cpReset(cid);
         return cpib[cid].ready; 
      }

     /*
      * Switch for state of tge message protocol processing
      */
      switch (cpib[cid].state)
      {
          case MSG_ACK: // Locks for a FLAG character
          
             if (ch == ACK_OK)
             {             				 				 
                 if (cpib[cid].idx > 1u)
                 {
                    message[cid][0] = ch;
                    cpib[cid].idx = 1u;
                 }
                 cpib[cid].state=MSG_STX; // Gos to next state
             }
             
             break;

          case MSG_STX: // Receive the message comman
		  
             cpib[cid].state= MSG_RESPONSE;
             
             break;

		  
		  case MSG_CMD_READ: // Receive the message comman
				 cpib[cid].state++;
			 break;


          case MSG_CMD_WRITE: // Receive the message comman
             cpib[cid].state++;
             break;

          case MSG_RESPONSE: // Receive message length (2 bytes) 

	
			 if(ch == OK_RESPONSE)
			 {
			  	 cpib[cid].state=MSG_DATA;
			 }
			 else
			 if(ch == INVALID_RESPONSE_BUF2)
				 cpib[cid].state++;
			 else
			 if(ch == INVALID_RESPONSE_BUF15)
				 cpib[cid].state++;
			 else
			 if(ch == PRESERVE_ERROR_RESPONSE)
				 cpib[cid].state++;
			 else
			 if(ch == OTHER_RESPONSE)
				 cpib[cid].state++;
			         
			 
             break;
 
         case MSG_DATA:

             if (cpib[cid].idx >= 20u) // Counts the received bytes
             {
               cpib[cid].idx = 0;
               cpib[cid].state = 0;
			   cpib[cid].crc16 = 0;

			   Checksum = MAKEHWORD(message[cid][18],message[cid][19]);

			   for(cmsg=&message[cid][2],emsg=&message[cid][17]; cmsg < emsg;)
				  CRC16(1, *cmsg++);
			   
			   if(Checksum == cpib[cid].crc16)
				   cpib[cid].ready = (unsigned short) message[cid][MSG_FLAG];
			   				
                  cpReset(COM1);
                  cpib[cid].ready = 0;
                  cpib[cid].len = 0;
             }   
			 
			 
             break;

          default:
             cpib[cid].state = 0;
             cpib[cid].idx = 0;
             cpib[cid].ready = 0;
             break;
       }
   } while (UARTCharsAvail(UART1_BASE));

   return cpib[cid].ready;
}


/*
 * FUNCTION:  COM0 ContropMessageLoop - Main Com Proc message processing loop
 * RETURN: Non-zero of message detected  
 */
int
ContropMessageLoop(unsigned long ulBase,unsigned char cid)
{
   char ch;
   int i = 0;	
   /*
    * If there is byte in comm. stream 
    */
   do 
   {
   
      //ch = UARTCharGet(ulBase);
	  ch = UARTCharGetNonBlocking(ulBase);
	  /*
	     * Checks Checksum
	     */
	  //Checksum(COM0, ch);

      message[cid][cpib[cid].idx++] = ch;
 
	  
     /*
      * Overflow check
      */ 
      if (cpib[cid].idx >= COM_PROC_BUF_LEN)
      {
         cpReset(cid);
         return cpib[cid].ready;
      }

     /*
      * Switch for state of tge message protocol processing
      */
      switch (cpib[cid].state)
      {
          case BCP_MSG_FLAG: // Locks for a FLAG character
          
             if (ch == BCP_HEADER_1_SYNC)
             {
				 
	             if (cpib[cid].idx > 1u)
                 {
                    message[cid][0] = ch;
                    cpib[cid].idx = 1u;
                 }
                 cpib[cid].state++; // Gos to next state
             }
             
             break;

          case BCP_HEADER_1: // Receive the message comman

             if (ch == BCP_HEADER_2_SYNC)    
             	cpib[cid].state++;
			 else
				 cpReset(COM1);
			 
             break;

          case BCP_HEADER_2: // Receive the message comman

             if (ch == BCP_HEADER_3_SYNC)
             {
          
             	cpib[cid].state++;
				cpib[cid].len = 20;
				
             }             
			 else
				 cpReset(COM1);

			 
             break;

 
         case BCP_MSG_DATA:
		 	
             if (cpib[cid].idx >= cpib[cid].len) // Counts the received bytes
             {
               cpib[cid].idx = 0;
               cpib[cid].state = 0;
			   cpib[cid].checksum = 0;
			   
               /*
                		* Checks Checksum
                		*/
               for(i=0;i<18;i++)
			   	Checksum(cid, message[cid][i]);
			   
				 cpib[cid].checksum %= 256;
				  
				 if (cpib[cid].checksum == message[cid][19])
				 {
					cpib[cid].ready = 1;			
					InterpretCmd();
					if ((WORD)(g_ulTickCount - ctrl.time[MES_TIME]) >= (WORD)1000) //-- every 1 sec
					{	
						  ctrl.time[MES_TIME] = g_ulTickCount;	  
						  BBLedToggle();
					}	
					
					//cpib[cid].len -= 2u;
			   //	return cpib[cid].ready;
			   
				 }
				 else
				 {	 
					cpib[cid].ready = CHECKSUM_ERROR_DETECT;
					cpib[cid].len = 0;
					//BBLedToggle();
					
				 }
			   }			 
			   break;


          default:
             cpib[cid].state = 0;
             cpib[cid].idx = 0;
             cpib[cid].ready = 0;
             break;
       }
   } while (UARTCharsAvail(UART0_BASE));

   return cpib[cid].ready;
}

/*!=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-  FUNCTION  HEADER  -=-=-=-=-=-=-=-=-=
 
FUNCTION:  OzSetTxMessage   
 
INTERNAL DESCRIPTION: preper the TX msg for transmit
 			  			  
----------------------------------------------------------------------------!*/

void cpPutMessage(unsigned long ulBase,unsigned char cmd,unsigned short data_len,unsigned char *msg)
{
	//int i = 0;


	LensZoomWrite.command = cmd;//0x45;		
		
	cpib[COM1].crc16 = 0;
	for(cmsg=&LensZoomWrite.write,emsg=&LensZoomWrite.hcrc16; cmsg < emsg;)
	   CRC16(1, *cmsg++);

	LensZoomWrite.hcrc16 = HIBYTE(cpib[COM1].crc16);
	LensZoomWrite.lcrc16 = LOBYTE(cpib[COM1].crc16);


	UART1Send(&LensZoomWrite.stx,19);		
	
}

/*
 * FUNCTION: cpMessageReady - Return Message Ready flag
 */
unsigned short cpMessageReady(unsigned char cid)
{
     return cpib[cid].ready;
}

/*
 * FUNCTION: cpGetMessageCommand - Returns received message command
 * and zero Message Ready flag
 */
unsigned short cpGetMessageCommand(unsigned char cid)
{
   unsigned short m = cpib[cid].ready;
   cpib[cid].ready = 0;    
   return m;
}

/*
 * FUNCTION: cpGetMessage - Returns received message
 */
unsigned char * cpGetMessage(unsigned char cid)
{
   return message[cid];
}

/*
 * FUNCTION: cpGetMessageData - Returns received message data 
 */ 
unsigned char * cpGetMessageData(unsigned char cid,unsigned short mgs_data)
{
   return (message[cid] + MSG_DATA + mgs_data); // ne virno !!!
}

/*
 * FUNCTION: cpGetMessageDataLen - Returns received message data length
 */
unsigned short cpGetMessageDataLen(unsigned char cid)
{
   return (cpib[cid].len - 6u);
}


/*
 * FUNCTION: Checksum - Perform Checksum calculation on input data
 *           and updates cpib[cid].checksum var.
 * PARAM: data - received data
 */
void Checksum(unsigned char cid, unsigned char data)
{

   cpib[cid].checksum+=data;
 
}

/*
 * FUNCTION: Checksum - Perform Checksum calculation on input data
 *           and updates cpib[cid].checksum var.
 * PARAM: data - received data
 */
unsigned char CalculateChecksumMsg(unsigned char cid, unsigned char *msg,unsigned short data_len)
{

  unsigned short i=0;
  int  checksum = 0; // Checksum message value

   for(i=0;i<data_len;i++)
   	checksum+=msg[i];
   checksum = (unsigned char)((~checksum) + 1);

   return checksum;   
}

void putDMCmode(WORD mode)
{
	DMCmode=mode;
}

unsigned short GetDMCmode(void)
{
	return DMCmode;
}


void MainModeLoop(void)
{

	if (DMCmode < MODE_NUM)
	{
		 DMCmode = ModeVec[DMCmode](DMCmode);
	}
	else
	{
		 DMCmode = ONLINE_STATE;
	}
	
}

// stay on this tempoary mode
WORD NullMode(WORD mode)
{
    return mode;
}

WORD ModeExec(WORD mode)
{
   if (mode < MODE_NUM)
       mode  = ModeVec[mode](mode);

   return mode;
}

/*
 * FUNCTION: OnlineState - Super state on ONLINE mode
 */
WORD OnlineState(WORD mode)
{

   return mode;
}



	
/*
 * FUNCTION: InitializationUnit - Super state on OFFLINE mode
 */
WORD InitializationUnit(WORD mode)
{

   if ((WORD)(g_ulTickCount - ctrl.time[ONLINE]) >= (WORD)50000) //-- every 1 sec
   {		
	   ctrl.time[ONLINE] = g_ulTickCount;  		 
	   
	   SetDMCflag(DMC_ENGINE_CONNECT,DMC_ENGINE_CONNECT);			   
	   return ONLINE_STATE;//MINTRON_READ_ZOOM;	   
   }
   else
   {

   
	   return INITIALIZATION_UNIT;   	

	   
   }
}


/*
 * FUNCTION: OnlineState - Super state on ONLINE mode
 */
WORD OfflineState(WORD mode)
{


      if (cpMessageReady(COM1))
      {      
      
         // Read Comm message
         WORD m = cpGetMessageCommand(COM1);         
         WORD f = GetDMCflag(0xffff);

      }
	  else 
	  if ((WORD)(g_ulTickCount - ctrl.time[OFFLINE]) >= (WORD)5000) //-- every 10 sec
	  {	  
				ctrl.time[OFFLINE] = g_ulTickCount;
				//if(ctrl.try[OFFLINE]++ >= 3)
				//{
					//if (InitUART(UART1_BASE,SysCtlClockGet(),115200,
						//(UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE)) == -1)
						//while (1);	  // hang
					//else
					//{
						//ctrl.try[OFFLINE] = 0;
						//mode = OFFLINE_STATE; 
					//}
				//}
				//else				
					mode = DOWNLOAD_VERSION;						
	  }
	  
   return mode;
}


/*

*/
WORD Mintron_Read_Zoom_Position(WORD mode)
{
  //switch (casein)
  /*
    * Switch for state of tge message protocol processing
    */
  //char charin = 0;
  
  switch (ctrl.state[MIN_ZOOM])
  {	

    case 0x0:
		
			UARTCharPut(UART1_BASE,0x05);
			ctrl.state[MIN_ZOOM] = 1;

	break;


	case 0x1:
		
			if(message[COM1][0] == 0x06)
				ctrl.state[MIN_ZOOM] = 2;
			else
				ctrl.state[MIN_ZOOM] = 0;
			
			cpReset(COM1);

  	  break;

	case 0x2:
				
			LensZoomWrite.write = 0x31;
			cpPutMessage(UART1_BASE,0x45,sizeof(LensZoomWrite),(unsigned char *)&LensZoomWrite);			
			ctrl.state[MIN_ZOOM] = 3;
			
      break;

    case 0x3:

			//target_pos = MAKEHWORD(message[COM1][5],message[COM1][6]);
			LensZoomWrite.write = 0x21;
			ctrl.state[MIN_ZOOM] = 0;
			mode = ZOOM_MAG_ON;
			
      break;    

    default:
		
      break;

  }
  return mode;
}


/*

*/
WORD Mintron_Zoom_In_Out(WORD mode)
{
  //switch (casein)
  /*
    * Switch for state of tge message protocol processing
    */
  //char zoom;
  
  switch (ctrl.state[MIN_ZOOM])
  {	

    case 0x0:

			LensZoomWrite.write = 0x21;
			cpPutMessage(UART1_BASE,0x45,sizeof(LensZoomWrite),(unsigned char *)&LensZoomWrite);			
			ctrl.state[MIN_ZOOM] = 0;				
			mode = ONLINE_STATE;
			
      break;

    default:
		
      break;

  }
  return mode;
}



/*

*/
void Set_OSD_Titel(unsigned short text_id)
{
  //switch (casein)
  /*
    * Switch for state of tge message protocol processing
    */

  //SetDMCflag(DMC_EXE_MODE,DMC_EXE_MODE);

  LensZoomWrite.write = 0x21;
  LensZoomWrite.data[0]=0x1;  //  
  LensZoomWrite.data[1]=0x0;  // 0x01 (n=11:character 11 ~ 15)			 
  
  LensZoomWrite.data[2] = 0;
  LensZoomWrite.data[4] = 0;  
  LensZoomWrite.data[6] = 0;  
  LensZoomWrite.data[8] = 0;  
  LensZoomWrite.data[10] = 0;
  
  switch (text_id)
  {	


    case 0x0:
			optical_zoom = (target_pos - 0x1000) / 111;
 			
			LensZoomWrite.data[2] = 0;
			LensZoomWrite.data[3] = 0x23;

			LensZoomWrite.data[4] = 1;
			LensZoomWrite.data[5] = 0x21;

			if(optical_zoom == 0xA )
			{
				LensZoomWrite.data[6] = 0;							
				LensZoomWrite.data[7] = 1;

				LensZoomWrite.data[8] = 0;				
				LensZoomWrite.data[9] = 0;
			}
			else
			{
				LensZoomWrite.data[6] = 0;				
				LensZoomWrite.data[7] = optical_zoom>>4;

				LensZoomWrite.data[8] = 0;
				LensZoomWrite.data[9] = optical_zoom & 0x0F;
			}
			
			LensZoomWrite.data[10] = 0;
			LensZoomWrite.data[11] = 0xfd;
			
			cpPutMessage(UART1_BASE,0x10,sizeof(LensZoomWrite),(unsigned char *)&LensZoomWrite);	
			
			
      break;

    case 0x1: // RATE

			LensZoomWrite.data[3] = 0x1B;
			LensZoomWrite.data[5] = 0x0A;
			LensZoomWrite.data[7] = 0x1D;
			LensZoomWrite.data[9] = 0x0E;
			LensZoomWrite.data[11] = 0xfd;
			
						
      break;


  case 0x2:	// SAFE

			LensZoomWrite.data[3] = 0x1C;
			LensZoomWrite.data[5] = 0x0A;
			LensZoomWrite.data[7] = 0x0F;
			LensZoomWrite.data[9] = 0x0E;
			LensZoomWrite.data[11] = 0xfd;
			
			
      break;

  case 0x3:	// PILOT

			LensZoomWrite.data[3] = 0x19;
			LensZoomWrite.data[5] = 0x12;
			LensZoomWrite.data[7] = 0x15;
			LensZoomWrite.data[9] = 0x18;
			LensZoomWrite.data[11] = 0x1D;
			
			
      break;

	case 0x4: // PTC
	
			  LensZoomWrite.data[3] = 0x19;
			  LensZoomWrite.data[5] = 0x1D;
			  LensZoomWrite.data[7] = 0x0C;
			  LensZoomWrite.data[9] = 0xfd;
			  LensZoomWrite.data[11] = 0xfd;			  
			  
		break;

	case 0x5: // C.G
	
			  LensZoomWrite.data[3] = 0x0C;
			  LensZoomWrite.data[5] = 0x10;
			  LensZoomWrite.data[7] = 0xFD;
			  LensZoomWrite.data[9] = 0xFD;
			  LensZoomWrite.data[11] = 0xFD;
			  
			  
		break;

	case 0x6: // PTC+CG
	
			  LensZoomWrite.data[3] = 0x19;
			  LensZoomWrite.data[5] = 0x1D;
			  LensZoomWrite.data[7] = 0x0C;
			  LensZoomWrite.data[9] = 0x0C;
			  LensZoomWrite.data[11] = 0x10;
			  
			  
		break;

	case 0x7: // Clean  0x02 (n=11:character 11 ~ 15)
	
			  LensZoomWrite.data[1]=0x0;  // 0x01 (n=11:character 11 ~ 15)
			  			  
			  LensZoomWrite.data[3] = 0xFD;
			  LensZoomWrite.data[5] = 0xFD;
			  LensZoomWrite.data[7] = 0xFD;
			  LensZoomWrite.data[9] = 0xFD;
			  LensZoomWrite.data[11] = 0xFD;
			  
			  
		break;

	case 0x8: // 
	
			  LensZoomWrite.data[3] = 0x0C;
			  LensZoomWrite.data[5] = 0x75;
			  LensZoomWrite.data[7] = 0x0;
			  LensZoomWrite.data[9] = 0xFD;
			  LensZoomWrite.data[11] = 0xFD;
			  
			  
		break;

	case 0x9: // 
	
			  LensZoomWrite.data[3] = 0xC;
			  LensZoomWrite.data[5] = 0x75;
			  LensZoomWrite.data[7] = 0x1;
			  LensZoomWrite.data[9] = 0xFD;
			  LensZoomWrite.data[11] = 0xFD;
			  
			  
		break;

	case 0xA: // 
	
			  LensZoomWrite.data[3] = 0xC;
			  LensZoomWrite.data[5] = 0x75;
			  LensZoomWrite.data[7] = 0x02;
			  LensZoomWrite.data[9] = 0xFD;
			  LensZoomWrite.data[11] = 0xFD;
			  
			  
		break;

	case 0xB: // 
	
			  LensZoomWrite.data[3] = 0xC;
			  LensZoomWrite.data[5] = 0x75;
			  LensZoomWrite.data[7] = 0x03;
			  LensZoomWrite.data[9] = 0xFD;
			  LensZoomWrite.data[11] = 0xFD;
			  
			  
		break;		

	case 0xC: // 
	
			  LensZoomWrite.data[3] = 0xC;
			  LensZoomWrite.data[5] = 0x75;
			  LensZoomWrite.data[7] = 0x04;
			  LensZoomWrite.data[9] = 0xFD;
			  LensZoomWrite.data[11] = 0xFD;
			  
			  
		break;				

	case 0xD: // BIT
	
			  LensZoomWrite.data[3] = 0xC;
			  LensZoomWrite.data[5] = 0x75;
			  LensZoomWrite.data[7] = 0x05;
			  LensZoomWrite.data[9] = 0xFD;
			  LensZoomWrite.data[11] = 0xFD;
			  
			  
		break;				


	case 0xE: // BIT
	
			  LensZoomWrite.data[3] = 0x0B;
			  LensZoomWrite.data[5] = 0x12;
			  LensZoomWrite.data[7] = 0x1D;
			  LensZoomWrite.data[9] = 0xFD;
			  LensZoomWrite.data[11] = 0xFD;
			  
			  
		break;

	case 0xF: // ERR
	
			  LensZoomWrite.data[3] = 0x0E;
			  LensZoomWrite.data[5] = 0x1B;
			  LensZoomWrite.data[7] = 0x1B;
			  LensZoomWrite.data[9] = 0xFD;
			  LensZoomWrite.data[11] = 0xFD;
			  
			  
		break;
		
    default:
		
      break;

  }
  
}





/*

*/
WORD Set_OSD_Position(WORD mode)
{
  //switch (casein)
  /*
    * Switch for state of tge message protocol processing
    */
  
  switch (ctrl.state[TIME_OSD_POS])
  {	

    case 0x0:

			LensZoomWrite.write = 0x21;
			LensZoomWrite.data[0]=0x3;	//	
			LensZoomWrite.data[1]=0x1;	//	0x00 (n=1:character 1 ~ 5)			
			
			cpPutMessage(UART1_BASE,0x10,sizeof(LensZoomWrite),(unsigned char *)&LensZoomWrite);			
			ctrl.state[TIME_OSD_POS] = 0;
			mode = ONLINE_STATE;
			
      break;

    default:
		
      break;

  }
  return mode;
}

WORD Set_AGC_Level(WORD mode)
{

	SetDMCflag(DMC_EXE_MODE,DMC_EXE_MODE);
	if ((WORD)(GetSysTickCount() - ctrl.time[TIME_ZOOM]) >= (WORD)1000) //-- every 1.0 sec  
	{
		ctrl.time[TIME_ZOOM] = GetSysTickCount(); 
		AgcSensLevel.write = 0x21;	// 	Buf[2]
		AgcSensLevel.data[0]=0x1;	//	Buf[3]
		AgcSensLevel.data[1]=0x01; //	Buf[4]	
		memset(&AgcSensLevel.data[2],0,11);
		cpPutMessage(UART1_BASE,0x1A,sizeof(AgcSensLevel),(unsigned char *)&AgcSensLevel);			
		return ONLINE_STATE;
		
	}
    return mode;
}


WORD Set_SENS_Level(WORD mode)
{

	SetDMCflag(DMC_EXE_MODE,DMC_EXE_MODE);
	if ((WORD)(GetSysTickCount() - ctrl.time[TIME_ZOOM]) >= (WORD)1000) //-- every 1.0 sec  
	{
		ctrl.time[TIME_ZOOM] = GetSysTickCount(); 
		AgcSensLevel.write = 0x21;	// 	Buf[2]
		AgcSensLevel.data[0]=0x2;	//	Buf[3]
		AgcSensLevel.data[1]=0x00; //	Buf[4]	
		memset(&AgcSensLevel.data[2],0,11);
		cpPutMessage(UART1_BASE,0x1A,sizeof(AgcSensLevel),(unsigned char *)&AgcSensLevel);			
		return ONLINE_STATE;
		
	}
    return mode;
}

WORD Zoom_Mag_On(WORD mode)
{

	SetDMCflag(DMC_EXE_MODE,DMC_EXE_MODE);
	if ((WORD)(GetSysTickCount() - ctrl.time[TIME_ZOOM]) >= (WORD)1000) //-- every 1.0 sec  
	{
		ctrl.time[TIME_ZOOM] = GetSysTickCount(); 
		LensZoomWrite.write = 0x21;
		LensZoomWrite.data[0]=0x9;	//	
		LensZoomWrite.data[1]=0x01; //	0x01==ZOOM MAG OFF	
		memset(&LensZoomWrite.data[2],0,11);
		cpPutMessage(UART1_BASE,0x45,sizeof(LensZoomWrite),(unsigned char *)&LensZoomWrite);			
		return ONLINE_STATE;
		
	}
    return mode;
}


WORD Zoom_Mag_Off(WORD mode)
{

	SetDMCflag(DMC_EXE_MODE,DMC_EXE_MODE);
	if ((WORD)(GetSysTickCount() - ctrl.time[TIME_ZOOM]) >= (WORD)1000) //-- every 1.0 sec  
	{
		ctrl.time[TIME_ZOOM] = GetSysTickCount(); 
		LensZoomWrite.write = 0x21;
		LensZoomWrite.data[0]=0x9;	//	
		LensZoomWrite.data[1]=0x00; //	0x00==ZOOM MAG OFF	
		memset(&LensZoomWrite.data[2],0,11);
		cpPutMessage(UART1_BASE,0x45,sizeof(LensZoomWrite),(unsigned char *)&LensZoomWrite);			
		return ONLINE_STATE;
		
	}
    return mode;
}

WORD Set_Mark_On(WORD mode)
{

	SetDMCflag(DMC_EXE_MODE,DMC_EXE_MODE);

	LensZoomWrite.write = 0x21;
	LensZoomWrite.data[0]=0x0;	//	
	if(GetDMCflag(DMC_OSD_ON) )
		LensZoomWrite.data[1]=0x1; // mark on == 1 /off	== 0
	else
		LensZoomWrite.data[1]=0x0; // mark on == 1 /off	== 0
		
	LensZoomWrite.data[12]=0x1; // mark on						
	memset(&LensZoomWrite.data[2],0,10);	
	cpPutMessage(UART1_BASE,0x56,sizeof(LensZoomWrite),(unsigned char *)&LensZoomWrite);				

    return ONLINE_STATE;
		
}



WORD Set_Mark_Off(WORD mode)
{

	SetDMCflag(DMC_EXE_MODE,DMC_EXE_MODE);

	LensZoomWrite.write = 0x21;
	LensZoomWrite.data[0]=0x0;	//	
	if(GetDMCflag(DMC_OSD_ON) )
		LensZoomWrite.data[1]=0x1; // mark on == 1 /off	== 0
	else
		LensZoomWrite.data[1]=0x0; // mark on == 1 /off	== 0
		
	LensZoomWrite.data[12]=0x1; // mark on						
	memset(&LensZoomWrite.data[2],0,10);	
	cpPutMessage(UART1_BASE,0x56,sizeof(LensZoomWrite),(unsigned char *)&LensZoomWrite);				

    return ONLINE_STATE;
		
}


WORD Set_OSD_On(WORD mode)
{

	if ((WORD)(GetSysTickCount() - ctrl.time[TIME_ZOOM]) >= (WORD)1000) //-- every 1.0 sec  
	{
		if(GetDMCflag(DMC_STOW) )
			Set_OSD_Titel(2);			
		else
		if(GetDMCflag(DMC_PILOT_WINDOW) )
			Set_OSD_Titel(3);			
		else
		if(GetDMCflag(DMC_RATE) )
			Set_OSD_Titel(1);								
		else
		if(GetDMCflag(DMC_POINT_2_COORD) )						
			Set_OSD_Titel(4);	
		else
		if(GetDMCflag(DMC_PTC_CG) )
			Set_OSD_Titel(6);
		else
		if(GetDMCflag(DMC_CAM_GUIDE) )
			Set_OSD_Titel(5);
						
		cpPutMessage(UART1_BASE,0x10,sizeof(LensZoomWrite),(unsigned char *)&LensZoomWrite); 
		ctrl.time[TIME_ZOOM] = GetSysTickCount();
		return ZOOM_MAG_ON;
		
	}
    return mode;
}




WORD Set_OSD_Off(WORD mode)
{

	if ((WORD)(GetSysTickCount() - ctrl.time[TIME_ZOOM]) >= (WORD)1000) //-- every 1.0 sec  
	{
		Set_OSD_Titel(7);			
		cpPutMessage(UART1_BASE,0x10,sizeof(LensZoomWrite),(unsigned char *)&LensZoomWrite);  
		ctrl.time[TIME_ZOOM] = GetSysTickCount();
		return ZOOM_MAG_OFF;
		
	}
    return mode;
}

