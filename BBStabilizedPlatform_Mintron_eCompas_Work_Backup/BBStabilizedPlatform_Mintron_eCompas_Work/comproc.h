//*****************************************************************************
//
// interrupt.h - Prototypes for the NVIC Interrupt Controller Driver.
//
// Copyright (c) 2005-2010 Texas Instruments Incorporated.  All rights reserved.
// Software License Agreement
// 
// Texas Instruments (TI) is supplying this software for use solely and
// exclusively on TI's microcontroller products. The software is owned by
// TI and/or its suppliers, and is protected under applicable copyright
// laws. You may not combine this software with "viral" open-source
// software in order to form a larger program.
// 
// THIS SOFTWARE IS PROVIDED "AS IS" AND WITH ALL FAULTS.
// NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT
// NOT LIMITED TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. TI SHALL NOT, UNDER ANY
// CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
// DAMAGES, FOR ANY REASON WHATSOEVER.
// 
// This is part of revision 6288 of the Stellaris Peripheral Driver Library.
//
//*****************************************************************************

#ifndef __COMPROC_H__
#define __COMPROC_H__

#ifndef __MTYPE_H__
#include "mtype.h"
#endif
//*****************************************************************************
//
// If building with a C++ compiler, make all of the definitions in this header
// have a C binding.
//
//*****************************************************************************
#ifdef __cplusplus
extern "C"
{
#endif

#define COM0		0u
#define COM1		1u

#define BIRD384_HEADER_LEN	  	5
#define TAILER_SIZE  			1


#define MAX_LABEL_LENGTH			16
#define MAX_LABEL_LENGTH_TEXT		4
#define MAX_LABEL_LENGTH_PARAM		4

#define CHECKSUM_ERROR_DETECT   0x2F

#define COM_PROC_BUF_LEN   256

#define TX_MSG_SIZE   255


#define HEADER_SYNC 0x55
#define ACK_OK 		0x06


#define MSG_FLAG   	 	0
#define MSG_CMD_LSB     1u
#define MSG_CMD_MSB     2u
#define MSG_LEN_LSB 	3u
#define MSG_LEN_MSB 	4u
#define MSG_DATA    	5u

#define MSG_ACK   	 	0
#define MSG_STX		    1u
#define MSG_CMD_WRITE   2u
#define MSG_CMD_READ 	3u
#define MSG_RESPONSE 	4u
#define MSG_DATA    	5u

#define BCP_HEADER_1_SYNC 0xB0
#define BCP_HEADER_2_SYNC 0x3B
#define BCP_HEADER_3_SYNC 0x4F

#define BCP_MSG_FLAG   	 	0u
#define BCP_HEADER_1   	 	1u
#define BCP_HEADER_2   	 	2u
#define BCP_MSG_DATA    	3u


//At the occasion of response line
#define OK_RESPONSE   	 		0xA0
#define INVALID_RESPONSE_BUF2 	0xA2
#define INVALID_RESPONSE_BUF15  0xA3
#define PRESERVE_ERROR_RESPONSE 0xA4
#define OTHER_RESPONSE    		0xA5

typedef struct _OSD_TITEL_ITEM_  
{
   unsigned char   stx, 	// Start code		0x2
   				   write,   //Write commend 	0x21
				   command,	//Control item 		0x00 - 0xFF	
				   char_set,// character setting
				   num_char,// character number n+x
				   data[10],//Write data
				   non_use,	// non-use
				   etx,		//Ending code		0x03
				   hcrc16,	//CRC code (calculation：X16 + X15 + X2 + 1)HIGH BYTE
				   lcrc16;	//CRC code (calculation：X16 + X15 + X2 + 1)LOW BYTE

   
}OSD_TITEL_ITEM_S, *P_OSD_TITEL_ITEM_S;

/*
 * Transmit  messages protocol data sructure control
 */
typedef struct _MINTRON_TXRX_  
{
   unsigned char   stx, 	// Start code		0x2
   				   write,   //Write commend 	0x21
				   command,	//Control item 		0x00 - 0xFF				   
				   data[13],//Write data
				   etx,		//Ending code		0x03
				   hcrc16,	//CRC code (calculation：X16 + X15 + X2 + 1)HIGH BYTE
				   lcrc16;	//CRC code (calculation：X16 + X15 + X2 + 1)LOW BYTE

   
}MINTRON_TXRX_S, *P_MINTRON_TXRX_S;


/*
 * Transmit messages protocol data sructure control
 */
typedef struct  
{
   unsigned char   stx, 	// Start code		0x2
   				   read,   //Read commend 	0x31
				   cmd,		//Control item 		0x00 - 0xFF				   
				   data[13],//Write data
				   etx,		//Ending code		0x03
				   hcrc16,	//CRC code (calculation：X16 + X15 + X2 + 1)HIGH BYTE
				   lcrc16;	//CRC code (calculation：X16 + X15 + X2 + 1)LOW BYTE

   
} MintronReadCommand;

/*
 * Receiver messages protocol data sructure control
 */
typedef struct  
{
   unsigned char   ack,   	// Start code		0x5- ENQ , 0x6- ACK:OK , 0x15- NAK:ERROR
   				   stx, 	// Start code	0x2
				   response,	   //	Buf [1] = 0xA0 (OK response),			   
								   //	0xA2 (Buf [1] invalid response),
								   //	0xA3 (Buf [2]. Buf [15] invalid response),
								   //	0xA4 ( setting to preserve an error response),
								   //	0xA5 the other response					
				   data[14],//Write data
				   etx,		//Ending code		0x03
				   hcrc16,	//CRC code (calculation：X16 + X15 + X2 + 1)HIGH BYTE
				   lcrc16;	//CRC code (calculation：X16 + X15 + X2 + 1)LOW BYTE
   
} MintronResponseCommand;


/*
 * Receiver messages protocol data sructure control
 */
typedef struct  
{
   unsigned short state; // The internal state of of the received message
   unsigned short idx;   //index to the next free place in receiver buffer  
   unsigned short len;   // len to store the message  length
   unsigned short tick;  // time tike tracking
   int 			  checksum; // Checksum message value
   unsigned short ready; // Message readty flag
   unsigned char mresp; // Modem response bitmap register
   unsigned short bufidx;  //index to the read buf1  
   unsigned short crc16; // CRC16 message value
} ComProcBuffer;

typedef struct _BB_TXRX_
{
	unsigned char	sync;   
	unsigned char	command[2];
	unsigned char	msg_len[2];                     // Alignment problem! becuase that not used as HWORD
	unsigned char	buffer[COM_PROC_BUF_LEN];
	unsigned char	checksum;
	
}BB_TXRX_S, *P_BB_TXRX_S;

typedef struct _LabelCmd
{
		unsigned char			Label_id_Visibility[2];
		unsigned char			Position_id_X_Y[5];
		unsigned char			Text_id;
				char			Text[22];
		unsigned char			Style_id_Color_Size_BackGro_text[7];					
		
} LabelCmd;

#define VISIBILITY 		1
#define UNVISIBILITY 	0 

#if 0
// RX/TX commands from Control  (via communication):
enum LabelIdx {
	STWO			   = 0,	// 0x00
	PILOT_WIN,				// 0x01
	RATE,					// 0x02
	VERSION,			// 0x03   	
	WHITE_HOT_OSD,				// 0x04
	BLACK_HOT_OSD, 			// 0x05	
	ZOOMX2,				// 0x06
	ZOOMX4,				// 0x07
	COM_ERROR,				// 0x08
	
	DATA1,				// 0x09
	DATA2,				// 0x0A	
	DATA3,				// 0x0B 	
	DATA4,				// 0x0C 		
	UNUSE2			// 0x0D
};
#else
// RX/TX commands from Control  (via communication):
enum LabelIdx {
	STWO			   = 0,	// 0x00
	PILOT_WIN,				// 0x01
	RATE,					// 0x02
	CAM_GUIDE,				// 0x03
	POINT_2_COORD,			// 0x04
	POT2COOR_CAMGUIED,		// 0x05
	ZOOMX2, 				// 0x06
	ZOOMX4, 				// 0x07	
	WHITE_HOT_OSD,			// 0x08
	VERSION,				// 0x09		
	BLACK_HOT_OSD, 			// 0x0A
	COM_ERROR,				// 0x0B	
	DATA1,					// 0x0C
	DATA2,					// 0x0D
	DATA3,					// 0x0E	
	DATA4,					// 0x0F		
	UNUSE2					// 0x10
};
#endif
	

extern void init_ctrl(void);
extern void Init_m_DRC_Params(void);
void Init_m_Zoom_Flip(void);
void Init_m_CIC_Params(void);
void Init_m_ALG_Params(void);
void Init_m_Frm_Avg_Params(void);
void Init_m_NUC_Params(void);
void Init_m_ARC_Params(void);
void Init_LABEL_Roll_Pitch(int Visibility);
void Init_m_LABEL_Phi_Theta(int Visibility);
void Show_Text_Label(unsigned char id,unsigned char Visib);   
void Set_LABEL_Visibility(unsigned char idx,int Visib);
void cpPutMessage(unsigned long ulBase,unsigned char cmd,unsigned short data_len,unsigned char *msg);
void Set_OSD_Titel(unsigned short text_id);
unsigned long GetSysTickCount(void);
void putDMCmode(unsigned short mode);
void MainModeLoop(void);
unsigned short GetDMCmode(void);
int	ContropMessageLoop(unsigned long ulBase,unsigned char cid);
int	MintronMessageLoopMessageLoop(unsigned long ulBase,unsigned char cid);
void cpReset(unsigned char cid);
void Checksum(unsigned char cid, unsigned char data);
unsigned char * cpGetMessage(unsigned char cid);
unsigned char * cpGetMessageData(unsigned char cid,unsigned short mgs_data);
unsigned char CalculateChecksumMsg(unsigned char cid, unsigned char *msg,unsigned short data_len);
void ModeVecF(void);
double GetCaseTemperature(void);
void LedToggle();
void LedToggle1();
void InitWatchDogTimer(void);
void ResetWatchDog(void);

#ifdef __cplusplus
}
#endif

#endif // __COMPROC_H__

