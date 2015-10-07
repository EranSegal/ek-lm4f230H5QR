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

//#define memcpy my_memcpy
//efine SCD_ENGINE
//#define MINTRON

#define COM0		0u
#define COM1		1u

#define BIRD384_HEADER_LEN	  	5
#define TAILER_SIZE  			1


#define MAX_LABEL_LENGTH			10
#define MAX_LABEL_ID				10

#define CHECKSUM_ERROR_DETECT   0x2F

#define COM_PROC_BUF_LEN   256

#define TX_MSG_SIZE   255


#define HEADER_SYNC 0x55
#define ACK_OK 		0x06

#define PAYLOAD_MODE    3u

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
#define BCP_MSG_CHEKSUM    	4u

typedef struct _CONT_BRIG {
	/* see definitions above */
	BYTE contrast;
	BYTE brightness;
	
} Cont_Brit_Fiels;


typedef struct _ZOOM_FLIP {
	/* see definitions above */
	BYTE byte1;
	BYTE byte2;
	WORD columns;
	WORD rows;
	WORD center_x;
	WORD center_y;
	
} Zoom_Flip;

typedef struct _RECT
{
	WORD top;
	WORD left;
	WORD bottom;
	WORD right;
} RECT;


typedef struct _FRM_ID
{
	WORD Row;
	WORD Column;
	WORD Mask;
	WORD Value;
} Frame_ID;


typedef struct _ALG_Params
{
BYTE ALG_NONE,		
	 ALG_CDS,
	 ALG_CIC,
	 ALG_NUC,
	 ALG_BPR,
	 ALG_DRC,
	 ALG_FRM,
	 ALG_SGF;
WORD Op_Type;
	
} ALG_Params;
	
typedef struct _CIC_Params
{
	WORD Op_Type;
	WORD Rows;
	WORD Columns;
	RECT Net_Image;
	Frame_ID Frm_ID;
	WORD Case_Ref_Col_1;
	WORD Case_Ref_Col_2;
	WORD SDC_Mode;
	char CIC_Filename[256];
} CIC_Params;


typedef struct _DRC_Params
{
	WORD Op_Type;				//	2 Byte
	WORD DRC_Type;				//	2 Byte
	WORD Rows;					//	2 Byte
	WORD Columns;				//	2 Byte
	RECT Net_Image;				//	8 Byte
	WORD Hist_Cut_Off;			//	2 Byte	
	WORD Hist_Cut_Off_Frm;		//	2 Byte		
	BYTE Polarity;				//	1 Byte
	BYTE Digital_Video;			//    1 byte
	WORD Hist_Comb_Param;		//	2 Byte	
	UINT DRC_Alg_Type_Param; 	//    4 byte
	WORD LUT_Update_freq;		//	2 Byte
	#ifdef VERSION_1_3
	WORD gain_minimum_input;	//	2 Byte
	WORD gain_maximum_input;	//	2 Byte
	BYTE gain_minimum_output;	//	1 Byte
	BYTE gain_maximum_output;	//	1 Byte
	#endif
	
} DRC_Params;

typedef struct _Gain_Params
{

	WORD gain_minimum_input;	//	2 Byte
	WORD gain_maximum_input;	//	2 Byte
	BYTE gain_minimum_output;	//	1 Byte
	BYTE gain_maximum_output;	//	1 Byte
	
} Gain_Params;

typedef struct _ARC_Params
{
	short			Enable;
	short			Threshold_Fine;
	short			Threshold_Coarse;
	short			Timeout_Trigger;
} ARC_Params;

typedef struct _NUC_Params
{
	WORD Op_Type;
	WORD Rows;
	WORD Columns;
	RECT Net_Image;
	Frame_ID Frm_ID;
	WORD Auxilliary_Col;
	WORD NUC_Type;
	char NUC_Filename[256];
} NUC_Params;

typedef struct _Frm_Avg_Params
{
	WORD	Op_Type;
	WORD	Rows;
	WORD	Columns;
	WORD	Buffer_ID;
	WORD	Num_Of_Frames;
	char	Frm_Avg_Filename[256];
} Frm_Avg_Params;

typedef struct _TEC_Params
{
	double 			Substrate_temp;	// Substrate temperature
	double 			Case_tem;  		//	Case temperature
	double 			Target_temp;	//	Target temperature
	double 			tec_current;	//	TEC current
	double 			diode_current;	//	Diode Temperature	
} TEC_Params;


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

typedef struct _POS_LABEL
{
	unsigned char Id;
	unsigned int XCoord;
	unsigned int YCoord;

} Pos_Lab;

typedef struct _LabelCmd
{
		unsigned char			Label_id_Visibility[2];
		unsigned char			Position_id_X_Y[5];
		unsigned char			Text_id;
				 char			Text[20];
		unsigned char			Style_id_Color_Size_BackGro_text[7];					
		
} LabelCmd;

#define VISIBILITY 		1
#define UNVISIBILITY 	0 

// RX/TX commands from Control  (via communication):
enum LabelIdx {
	OSD_MODE_ID		   = 0,	// 0x00
	OSD_ZOOM_ID,			// 0x01
	OSD_BH_WH_ID,			// 0x02
	OSD_VERSION_ID,			// 0x03
	OSD_ROLL_LEB, 			// 0x04
	OSD_PITCH_LEB,			// 0x05	
	OSD_ROLL_POS,			// 0x06	
	OSD_PITCH_POS,			// 0x07		
	OSD_AGC_ON_OFF,			// 0x08	
	OSD_ELEVET,				// 0x09			
};
	
void InitParams(void);
void init_ctrl(void);
void Init_m_DRC_Params(void);
void Init_m_Zoom_Flip(void);
void Init_m_CIC_Params(void);
void Init_m_ALG_Params(void);
void Init_m_Frm_Avg_Params(void);
void Init_m_NUC_Params(void);
void Init_m_ARC_Params(void);
void Init_LABEL_Roll_Pitch(BYTE Visibility);
void Set_LABEL_Visibility(unsigned char idx,BYTE Visib);
int Set_Text_Label(unsigned char id,unsigned char Visib);   
void Put_Text_Label(unsigned char id,unsigned char Visib);
void Text_Label(unsigned char id,unsigned char Visib);   
unsigned long GetSysTickCount(void);
void putDMCmode(unsigned short mode);

void MainModeLoop(void);
unsigned short GetDMCmode(void);
int	ContropMessageLoop(unsigned long ulBase,unsigned char cid);
int OmapMessageLoop(unsigned long ulBase,unsigned char cid);
int mpGetUserTelemetry(char *pData);
void cpReset(unsigned char cid);
void Checksum(unsigned char cid, unsigned char data);
unsigned char * cpGetMessage(unsigned char cid);
unsigned char * cpGetMessageData(unsigned char cid,unsigned short mgs_data);
unsigned char CalculateChecksumMsg(unsigned char cid, unsigned char *msg,unsigned short data_len);
void ModeVecF(void);
double GetCaseTemperature(void);
void InitWatchDogTimer(void);
void ResetWatchDog(void);


#ifdef __cplusplus
}
#endif

#endif // __COMPROC_H__

