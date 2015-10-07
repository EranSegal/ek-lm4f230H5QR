
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
#include "comproc.h"
#include "SysTick.h"
#include "Interpret.h"
#include "Interpret.h"
#include "modeid.h"
#include "globals.h"
#include "reg.h"
#include "led.h"
#include "uart_echo.h"
#include "A3906.h"
#include "rmb20d01.h"
#include "ITG3200.h"
#include "bma180.h"
#include "encoder.h"

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


static BYTE Freeze_Unfreeze = 0x0;
BYTE Stabilize_status = 0x0;

ComProcBuffer cpib[2]; 

extern unsigned long t_ulFlags;
extern ctrl_type ctrl;
extern char  *BCVersion;
extern float e_pos_roll;
extern float e_pos_pitch;

extern char  *BBVersion;
extern float Phic,Thetac;
extern float Ax,Ay,Az,g,Axm,Aym,Azm;
extern unsigned dcycle[2]; 	  // current duty cycle

Frm_Avg_Params	m_Frm_Avg_Params;
NUC_Params 		m_NUC_Params;
ARC_Params 		m_ARC_Params;
ALG_Params 		m_ALG_Params;
CIC_Params 		m_CIC_Params;
DRC_Params 		m_DRC_Params;
Zoom_Flip 		m_Zoom_Flip;
TEC_Params 		m_TEC_Params;

unsigned char message[2][COM_PROC_BUF_LEN]; // receiver message buffer
unsigned char BbTxBuffer[TX_MSG_SIZE];
LabelCmd LabelCmdMsg[MAX_LABEL_LENGTH];

typedef WORD (* ModeType)(WORD);

const ModeType ModeVec[]  =	
{
    Download_Engine_Version,    // DOWNLOAD_VERSION 		0x00
	Upload_Engine_Version,		// DOWNLOAD_VERSION 		0x01       
    OnlineState,		  		// ONLINE_STATE (stop)		0x02  
    Upload_Black_Hot,      		//  BLACK_HOT           		0x03   
    Upload_White_Hot,      		//  WHITE_HOT          		0x04
    Upload_General_Parameters,	// UPLOAD_G_PARAM    	 	0x05	
    NullMode,					// 			         		0x06 
    Open_Close_Shutter,			// OPEN_CLODE_SHUTTER		0x07
    Shutter_Read_Status,		// SHUTTER_READ_STATUS 	0x08
    Shutter_Write_Status,		// SHUTTER_WRITE_STATUS 	0x09
    Enabled_ARC,          		// ENABLED_ARC 			0x0A   
    Disabled_ARC,          		// DISABLED_ARC    		0x0B
    Enabled_CIC,          		// ENABLED_CIC  			0x0C
    Disabled_CIC,     			// DISABLED_CIC			0x0D
    Update_2_P,					// UPDATE2_P				0x0E   
    NullMode,     				//                        			0x0F  
    OfflineState,	  			// OFFLINE_STATE (stop)		0x10
    Freeze_Unfreeze_Video,		// FREEZE_UNFREEZE		0x11
    Upload_V_Flip_Enable,		// UPDATE_V_FLIP			0x12
    Upload_V_Flip_Disable,		// UPDATE_V_FLIP_D		0x13    
    Upload_H_Flip_Enable,		// UPDATE_H_FLIP			0x14
    Upload_H_Flip_Disable,		// UPDATE_H_FLIP_D		0x15  
    Upload_ZoomX2,				// UPDATE_ZOOMX2			0x16
    Upload_ZoomX4,				// UPDATE_ZOOMX4			0x17 
    Disable_Zoom,				// UPDATE_DISABLE_ZOOM	0x18
    Upload_Zoom_Position,		// UPDATE_ZOOM_FLIP		0x19    
    Tec_Read_Handler,			// UPDATE_ZOOM_FLIP		0x1A    
	InitializationUnit,			// INITIALIZATION_UNIT		0x1B
	InitializationUnit,			// INITIALIZATION_UNIT		0x1C
	InitializationUnit,			// INITIALIZATION_UNIT		0x1D	
	InitializationUnit,			// INITIALIZATION_UNIT		0x1E
	Set_Text_Label_Command,		// SET_LABEL_COMMAND		0x1F
	On_Off_Stabilize,			// ON_OFF_STABILIZE		0x20
	RefreshNUCDataCommand,		// REFRESH_NUC			0x21
	NullMode,					//						0x22
	NullMode,					//						0x23	
	NullMode,					//						0x24		
	NullMode,					//						0x25			
	NullMode,					//						0x26
    InitializationUnit			// INITIALIZATION_UNIT		0x27
};

#define MODE_NUM (sizeof(ModeVec) / sizeof(ModeType))

void init_ctrl(void)
{

  ctrl.time[MAIN] = 0;
  ctrl.time[TIME_ZOOM] = 0;
  
  ctrl.time[ONLINE] = g_ulTickCount;  
  ctrl.time[MES_TIME] = g_ulTickCount; 
  ctrl.time[WHITE_BLACK] = g_ulTickCount;
  ctrl.time[TIME_STABILIZ] = g_ulTickCount;
  ctrl.time[TIME_STOW] = g_ulTickCount;
  ctrl.time[TIME_PILOT] = g_ulTickCount;
  ctrl.time[TIME_RATE] = g_ulTickCount;
  ctrl.time[TIME_BH] = g_ulTickCount;
  ctrl.time[TIME_WH] = g_ulTickCount;
  ctrl.time[TEXT_LABEL]= g_ulTickCount;
  ctrl.time[TEXT_LABEL1]= g_ulTickCount;
  ctrl.time[TIME_NUC] = g_ulTickCount;

  ctrl.state[ONLINE] = 0;
  ctrl.state[UPDATE_2_P] = DISABLED_CIC_2_P;
  ctrl.state[ACT_4] = SET_TEMPERATURE;
  ctrl.state[TEXT_LABEL] = HIDE_SHOW_LABEL; 				  
  ctrl.state[TEXT_LABEL1] = HIDE_SHOW_LABEL; 				      
  ctrl.state[TIME_BH] = BH_SENT_MESSAGE;
  ctrl.state[TIME_WH] = WH_SENT_MESSAGE;
  ctrl.state[TIME_ZOOM2] = Z2_SENT_MESSAGE;
  ctrl.state[TIME_ZOOM4] = Z4_SENT_MESSAGE;  
  ctrl.state[TIME_DIS] = DIS_SENT_MESSAGE;  
  ctrl.state[TIME_NUC] = REFRESH_NUC_CMD;
  
  //ctrl.main_count = 0;
  ctrl.try[ACT_11]= 0;
  ctrl.try[OFFLINE]= 0;
  ctrl.try[ONLINE] = 0;
  ctrl.try[TIME_ZOOM2] = 0;
  ctrl.try[TIME_ZOOM4] = 0;  
  ctrl.try[TIME_DIS] = 0;  

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

void Init_m_DRC_Params(void)
{

	m_DRC_Params.Op_Type=0;					//	2 Byte
	m_DRC_Params.DRC_Type = DRC_TYPE_CLHE;	//	2 Byte
	m_DRC_Params.Rows = IMAGE_HIGHT;		//	2 Byte
	m_DRC_Params.Columns = IMAGE_WIDTH;		//	2 Byte
 	m_DRC_Params.Net_Image.top = 0x1;
	m_DRC_Params.Net_Image.left = NET_IMAGE_START;
	m_DRC_Params.Net_Image.bottom = NUM_OF_ROWS-1;
	m_DRC_Params.Net_Image.right = NET_IMAGE_END-1;
	m_DRC_Params.Hist_Cut_Off = 0x93;		//	2 Byte	
	m_DRC_Params.Hist_Cut_Off_Frm = 0;		//	2 Byte		
	m_DRC_Params.Polarity = B_HOT_POLARITY;	//	1 Byte
	m_DRC_Params.Digital_Video = 0;			//    1 byte
	m_DRC_Params.Hist_Comb_Param =	0;		//	2 Byte	
	m_DRC_Params.DRC_Alg_Type_Param = 0x14;	//    4 byte
	m_DRC_Params.LUT_Update_freq =	  0x0A;	//	2 Byte
	
	#ifdef VERSION_1_3
	m_DRC_Params.gain_minimum_input = 0;	//	2 Byte
	m_DRC_Params.gain_maximum_input = 0x3FFF; //	2 Byte
	m_DRC_Params.gain_minimum_output = 0x10; //	1 Byte
	m_DRC_Params.gain_maximum_output= 0xEB;	//	1 Byte
	#endif

}


void Init_m_Zoom_Flip(void)
{

	/* see definitions above */
	m_Zoom_Flip.byte1 = 0x0;
	m_Zoom_Flip.byte2 = 0x10;
	m_Zoom_Flip.columns = IMAGE_WIDTH;
	m_Zoom_Flip.rows = IMAGE_HIGHT;
	m_Zoom_Flip.center_x = 320;
	m_Zoom_Flip.center_y = 200;
}

void Init_m_CIC_Params(void)
{

	/*  CIC  params  */
	m_CIC_Params.Op_Type = 0;
	m_CIC_Params.Rows = IMAGE_HIGHT;
	m_CIC_Params.Columns = IMAGE_WIDTH;
	
	m_CIC_Params.Net_Image.top = 1;
	m_CIC_Params.Net_Image.left = 7;
	m_CIC_Params.Net_Image.bottom = NUM_OF_ROWS;
	m_CIC_Params.Net_Image.right = NUM_OF_COLUMNS;

	m_CIC_Params.Frm_ID.Row = 0x0001;
	m_CIC_Params.Frm_ID.Column = 0x0001;
	m_CIC_Params.Frm_ID.Mask = 0;
	m_CIC_Params.Frm_ID.Value = 0;
	
	m_CIC_Params.Case_Ref_Col_1 = CASE_REFERENCE_COLUMN_1;
	m_CIC_Params.Case_Ref_Col_2 = CASE_REFERENCE_COLUMN_2;
	m_CIC_Params.SDC_Mode = 0;
	strcpy(m_CIC_Params.CIC_Filename,"/mnt/config/CIC.bin");
	//m_CIC_Params.CIC_Filename[] = "/mnt/config/CIC.bin";
	
}

void Init_m_ALG_Params(void)
{

	 m_ALG_Params.ALG_NONE = 0x0,		
	 m_ALG_Params.ALG_CDS = 0x0,
	 m_ALG_Params.ALG_CIC = 0x2,
	 m_ALG_Params.ALG_NUC = 0x6,
	 m_ALG_Params.ALG_BPR = 0x3,
	 m_ALG_Params.ALG_DRC = 0x5,
	 m_ALG_Params.ALG_FRM = 0x4,
	 m_ALG_Params.ALG_SGF = 0x0;
	 m_ALG_Params.Op_Type = 0;
}

void Init_m_ARC_Params(void)
{
	 m_ARC_Params.Enable = 0x01;
	 m_ARC_Params.Threshold_Fine=ARC_THRESHOLD_FINE_DEFAULT;
	 m_ARC_Params.Threshold_Coarse=ARC_THRESHOLD_COARSE_DEFAULT;
	 m_ARC_Params.Timeout_Trigger =ARC_TIME_TRIGGER;
}


	
void Init_m_NUC_Params(void)
{

	/*  NUC  params  */
	m_NUC_Params.Op_Type = 0;
	m_NUC_Params.Rows = IMAGE_HIGHT;
	m_NUC_Params.Columns = IMAGE_WIDTH;
	
	m_NUC_Params.Net_Image.top = 0;
	m_NUC_Params.Net_Image.left = NET_IMAGE_START;
	m_NUC_Params.Net_Image.bottom = NUM_OF_ROWS-1;
	m_NUC_Params.Net_Image.right = NET_IMAGE_END;
	
	m_NUC_Params.Frm_ID.Row = 0x0001;
	m_NUC_Params.Frm_ID.Column = 0x0001;
	m_NUC_Params.Frm_ID.Mask = 0x0000;
	m_NUC_Params.Frm_ID.Value = 0x0000;
	
	m_NUC_Params.Auxilliary_Col = NUM_OF_AUX_COLUMNS;
	m_NUC_Params.NUC_Type = 0;
	strcpy(m_NUC_Params.NUC_Filename,"/mnt/config/NUC.bin");
	//m_NUC_Params.NUC_Filename[] = "/mnt/config/NUC.bin";	// 26-281	
};


void Init_m_Frm_Avg_Params(void)
{
	/*	Frm_Avg  params  */
	m_Frm_Avg_Params.Op_Type = 0;
	m_Frm_Avg_Params.Rows = IMAGE_WIDTH;
	m_Frm_Avg_Params.Columns = IMAGE_HIGHT;
	m_Frm_Avg_Params.Buffer_ID = 0;
	m_Frm_Avg_Params.Num_Of_Frames = 0x20;
	strcpy(m_Frm_Avg_Params.Frm_Avg_Filename,"/mnt/config/Avg.bin");
	//m_Frm_Avg_Params.Frm_Avg_Filename[] = "/mnt/config/Avg.bin";	
}

void Init_LABEL_Roll_Pitch(int Visibility)
{
		unsigned char id = 0;		
		unsigned char idx = 0;
		
		memset(&LabelCmdMsg,0,sizeof(LabelCmdMsg));

		LabelCmdMsg[idx].Label_id_Visibility[0]		= id;	// 0
		LabelCmdMsg[idx].Label_id_Visibility[1]		= Visibility;
		LabelCmdMsg[idx].Position_id_X_Y[0] 		= id;
		LabelCmdMsg[idx].Position_id_X_Y[1]		= 180;	// X
		LabelCmdMsg[idx].Position_id_X_Y[3]		= 10;	// Y
		memcpy(&LabelCmdMsg[idx].Style_id_Color_Size_BackGro_text[0],(unsigned char *)&id,sizeof(unsigned char));			
		memset(&LabelCmdMsg[idx].Style_id_Color_Size_BackGro_text[1],0,sizeof(unsigned short));		
		memset(&LabelCmdMsg[idx].Style_id_Color_Size_BackGro_text[3],0x10,sizeof(unsigned char));		
		memset(&LabelCmdMsg[idx].Style_id_Color_Size_BackGro_text[4],0xFF,sizeof(unsigned short));		
		memset(&LabelCmdMsg[idx].Style_id_Color_Size_BackGro_text[6],1,sizeof(unsigned char));		
		LabelCmdMsg[idx].Text_id			= id;
		//strcpy(LabelCmdMsg[idx].Text,"STOW");	// 0
		//LabelCmdMsg[idx].Text[strlen(LabelCmdMsg[idx].Text)] = 0;

		id++; // 1
		idx++;
		LabelCmdMsg[idx].Label_id_Visibility[0]		= id; 	// 1
		LabelCmdMsg[idx].Label_id_Visibility[1]		= Visibility;
		LabelCmdMsg[idx].Position_id_X_Y[0]		= id;
		LabelCmdMsg[idx].Position_id_X_Y[1]		= 7;	// X
		LabelCmdMsg[idx].Position_id_X_Y[3]		= 100;	// Y
		memcpy(&LabelCmdMsg[idx].Style_id_Color_Size_BackGro_text[0],(unsigned char *)&id,sizeof(unsigned char));	
		memset(&LabelCmdMsg[idx].Style_id_Color_Size_BackGro_text[1],0xFF,sizeof(unsigned short));
		memset(&LabelCmdMsg[idx].Style_id_Color_Size_BackGro_text[3],14,sizeof(unsigned char));
		memset(&LabelCmdMsg[idx].Style_id_Color_Size_BackGro_text[4],0,sizeof(unsigned short));
		memset(&LabelCmdMsg[idx].Style_id_Color_Size_BackGro_text[6],1,sizeof(unsigned char));
		LabelCmdMsg[idx].Text_id			= id;
		//strcpy(LabelCmdMsg[idx].Text,"PILOT");		// 1
		//LabelCmdMsg[idx].Text[strlen(LabelCmdMsg[idx].Text)] = 0;

		id++;	// 2
		idx++;
		LabelCmdMsg[idx].Label_id_Visibility[0]		= id;	// 2
		LabelCmdMsg[idx].Label_id_Visibility[1]		= Visibility;
		LabelCmdMsg[idx].Position_id_X_Y[0]		= id;
		LabelCmdMsg[idx].Position_id_X_Y[1]		= 30;	// X
		LabelCmdMsg[idx].Position_id_X_Y[3]		= 30;	// Y
		memcpy(&LabelCmdMsg[idx].Style_id_Color_Size_BackGro_text[0],(unsigned char *)&id,sizeof(unsigned char));	
		memset(&LabelCmdMsg[idx].Style_id_Color_Size_BackGro_text[1],0,sizeof(unsigned short));
		memset(&LabelCmdMsg[idx].Style_id_Color_Size_BackGro_text[3],0x10,sizeof(unsigned char));
		memset(&LabelCmdMsg[idx].Style_id_Color_Size_BackGro_text[4],0xFF,sizeof(unsigned short));
		memset(&LabelCmdMsg[idx].Style_id_Color_Size_BackGro_text[6],1,sizeof(unsigned char));
		LabelCmdMsg[idx].Text_id			= id;
		//strcpy(LabelCmdMsg[idx].Text,"RATE");		// 2
		//LabelCmdMsg[idx].Text[strlen(LabelCmdMsg[idx].Text)] = 0;

		id++;	// 3
		idx++;
		LabelCmdMsg[idx].Label_id_Visibility[0]		= id;	// 3
		LabelCmdMsg[idx].Label_id_Visibility[1]		= Visibility;
		LabelCmdMsg[idx].Position_id_X_Y[0]		= id;
		LabelCmdMsg[idx].Position_id_X_Y[1]		= 30;//250;	// X
		LabelCmdMsg[idx].Position_id_X_Y[3]		= 20;//10;	// Y
		memcpy(&LabelCmdMsg[idx].Style_id_Color_Size_BackGro_text[0],(unsigned char *)&id,sizeof(unsigned char));	
		memset(&LabelCmdMsg[idx].Style_id_Color_Size_BackGro_text[1],0,sizeof(unsigned short));
		memset(&LabelCmdMsg[idx].Style_id_Color_Size_BackGro_text[3],10,sizeof(unsigned char));
		memset(&LabelCmdMsg[idx].Style_id_Color_Size_BackGro_text[4],0xFF,sizeof(unsigned short));
		memset(&LabelCmdMsg[idx].Style_id_Color_Size_BackGro_text[6],1,sizeof(unsigned char));
		LabelCmdMsg[idx].Text_id			= id;
		//strcpy(LabelCmdMsg[idx].Text,"CG    ");		//	3
		//LabelCmdMsg[idx].Text[strlen(LabelCmdMsg[idx].Text)] = 0;


		id++;	// 4
		idx++;
		LabelCmdMsg[idx].Label_id_Visibility[0]		= id;	// 4
		LabelCmdMsg[idx].Label_id_Visibility[1]		= Visibility;
		LabelCmdMsg[idx].Position_id_X_Y[0]		= id;
		LabelCmdMsg[idx].Position_id_X_Y[1]		= 35;	// X
		LabelCmdMsg[idx].Position_id_X_Y[3]		= 250;	// Y
		memcpy(&LabelCmdMsg[idx].Style_id_Color_Size_BackGro_text[0],(unsigned char *)&id,sizeof(unsigned char));	
		memset(&LabelCmdMsg[idx].Style_id_Color_Size_BackGro_text[1],0xFF,sizeof(unsigned short));
		memset(&LabelCmdMsg[idx].Style_id_Color_Size_BackGro_text[3],14,sizeof(unsigned char));
		memset(&LabelCmdMsg[idx].Style_id_Color_Size_BackGro_text[4],0,sizeof(unsigned short));
		memset(&LabelCmdMsg[idx].Style_id_Color_Size_BackGro_text[6],1,sizeof(unsigned char));
		LabelCmdMsg[idx].Text_id			= id;
		//strcpy(LabelCmdMsg[idx].Text,"CG    ");		//	3
		//LabelCmdMsg[idx].Text[strlen(LabelCmdMsg[idx].Text)] = 0;

		id++;	// 5
		idx++;
		LabelCmdMsg[idx].Label_id_Visibility[0]		= id;	// 5
		LabelCmdMsg[idx].Label_id_Visibility[1]		= Visibility;
		LabelCmdMsg[idx].Position_id_X_Y[0]		= id;
		LabelCmdMsg[idx].Position_id_X_Y[1]		= 100;	// X
		LabelCmdMsg[idx].Position_id_X_Y[3]		= 250;	// Y
		memcpy(&LabelCmdMsg[idx].Style_id_Color_Size_BackGro_text[0],(unsigned char *)&id,sizeof(unsigned char));	
		memset(&LabelCmdMsg[idx].Style_id_Color_Size_BackGro_text[1],0xFF,sizeof(unsigned short));
		memset(&LabelCmdMsg[idx].Style_id_Color_Size_BackGro_text[3],14,sizeof(unsigned char));
		memset(&LabelCmdMsg[idx].Style_id_Color_Size_BackGro_text[4],0,sizeof(unsigned short));
		memset(&LabelCmdMsg[idx].Style_id_Color_Size_BackGro_text[6],1,sizeof(unsigned char));
		LabelCmdMsg[idx].Text_id			= id;
		//strcpy(LabelCmdMsg[idx].Text,"CG    ");		//	3
		//LabelCmdMsg[idx].Text[strlen(LabelCmdMsg[idx].Text)] = 0;

		id++;	// 6
		idx++;
		LabelCmdMsg[idx].Label_id_Visibility[0]		= id;	// 6
		LabelCmdMsg[idx].Label_id_Visibility[1]		= Visibility;
		LabelCmdMsg[idx].Position_id_X_Y[0]		= id;
		LabelCmdMsg[idx].Position_id_X_Y[1]		= 56;	// X
		LabelCmdMsg[idx].Position_id_X_Y[3]		= 250;	// Y
		memcpy(&LabelCmdMsg[idx].Style_id_Color_Size_BackGro_text[0],(unsigned char *)&id,sizeof(unsigned char));	
		memset(&LabelCmdMsg[idx].Style_id_Color_Size_BackGro_text[1],0xFF,sizeof(unsigned short));
		memset(&LabelCmdMsg[idx].Style_id_Color_Size_BackGro_text[3],14,sizeof(unsigned char));
		memset(&LabelCmdMsg[idx].Style_id_Color_Size_BackGro_text[4],0,sizeof(unsigned short));
		memset(&LabelCmdMsg[idx].Style_id_Color_Size_BackGro_text[6],1,sizeof(unsigned char));
		LabelCmdMsg[idx].Text_id			= id;
		//strcpy(LabelCmdMsg[idx].Text,"CG    ");		//	3
		//LabelCmdMsg[idx].Text[strlen(LabelCmdMsg[idx].Text)] = 0;	

		id++;	// 7
		idx++;
		LabelCmdMsg[idx].Label_id_Visibility[0]		= id;	// 7
		LabelCmdMsg[idx].Label_id_Visibility[1]		= Visibility;
		LabelCmdMsg[idx].Position_id_X_Y[0]		= id;
		LabelCmdMsg[idx].Position_id_X_Y[1]		= 125;	// X
		LabelCmdMsg[idx].Position_id_X_Y[3]		= 250;	// Y
		memcpy(&LabelCmdMsg[idx].Style_id_Color_Size_BackGro_text[0],(unsigned char *)&id,sizeof(unsigned char));	
		memset(&LabelCmdMsg[idx].Style_id_Color_Size_BackGro_text[1],0xFF,sizeof(unsigned short));
		memset(&LabelCmdMsg[idx].Style_id_Color_Size_BackGro_text[3],14,sizeof(unsigned char));
		memset(&LabelCmdMsg[idx].Style_id_Color_Size_BackGro_text[4],0,sizeof(unsigned short));
		memset(&LabelCmdMsg[idx].Style_id_Color_Size_BackGro_text[6],1,sizeof(unsigned char));
		LabelCmdMsg[idx].Text_id			= id;
		//strcpy(LabelCmdMsg[idx].Text,"CG    ");		//	3
		//LabelCmdMsg[idx].Text[strlen(LabelCmdMsg[idx].Text)] = 0;
		

		id++;	// 8
		idx++;
		LabelCmdMsg[idx].Label_id_Visibility[0]		= id;	// 8
		LabelCmdMsg[idx].Label_id_Visibility[1]		= Visibility;
		LabelCmdMsg[idx].Position_id_X_Y[0]		= id;
		LabelCmdMsg[idx].Position_id_X_Y[1]		= 0;	// X
		LabelCmdMsg[idx].Position_id_X_Y[3]		= 88;	// Y
		memcpy(&LabelCmdMsg[idx].Style_id_Color_Size_BackGro_text[0],(unsigned char *)&id,sizeof(unsigned char));	
		memset(&LabelCmdMsg[idx].Style_id_Color_Size_BackGro_text[1],0,sizeof(unsigned short));
		memset(&LabelCmdMsg[idx].Style_id_Color_Size_BackGro_text[3],14,sizeof(unsigned char));
		memset(&LabelCmdMsg[idx].Style_id_Color_Size_BackGro_text[4],0xFF,sizeof(unsigned short));
		memset(&LabelCmdMsg[idx].Style_id_Color_Size_BackGro_text[6],1,sizeof(unsigned char));
		LabelCmdMsg[idx].Text_id			= id;
		//strcpy(LabelCmdMsg[idx].Text,"CG    ");		//	3
		//LabelCmdMsg[idx].Text[strlen(LabelCmdMsg[idx].Text)] = 0;		

		id++;	// 9
		idx++;
		LabelCmdMsg[idx].Label_id_Visibility[0]		= id;	// 9
		LabelCmdMsg[idx].Label_id_Visibility[1]		= Visibility;
		LabelCmdMsg[idx].Position_id_X_Y[0]		= id;
		LabelCmdMsg[idx].Position_id_X_Y[1]		= 5;	// X
		LabelCmdMsg[idx].Position_id_X_Y[3]		= 100;	// Y
		memcpy(&LabelCmdMsg[idx].Style_id_Color_Size_BackGro_text[0],(unsigned char *)&id,sizeof(unsigned char));	
		memset(&LabelCmdMsg[idx].Style_id_Color_Size_BackGro_text[1],0xFF,sizeof(unsigned short));
		memset(&LabelCmdMsg[idx].Style_id_Color_Size_BackGro_text[3],14,sizeof(unsigned char));
		memset(&LabelCmdMsg[idx].Style_id_Color_Size_BackGro_text[4],0,sizeof(unsigned short));
		memset(&LabelCmdMsg[idx].Style_id_Color_Size_BackGro_text[6],1,sizeof(unsigned char));
		LabelCmdMsg[idx].Text_id			= id;
		//strcpy(LabelCmdMsg[idx].Text,"CG    ");		//	3
		//LabelCmdMsg[idx].Text[strlen(LabelCmdMsg[idx].Text)] = 0;		

		id++;	// 10
		idx++;
		LabelCmdMsg[idx].Label_id_Visibility[0]		= id;	// 10
		LabelCmdMsg[idx].Label_id_Visibility[1]		= Visibility;
		LabelCmdMsg[idx].Position_id_X_Y[0]		= id;
		LabelCmdMsg[idx].Position_id_X_Y[1]		= 24;	// X
		LabelCmdMsg[idx].Position_id_X_Y[3]		= 110;	// Y
		memcpy(&LabelCmdMsg[idx].Style_id_Color_Size_BackGro_text[0],(unsigned char *)&id,sizeof(unsigned char));	
		memset(&LabelCmdMsg[idx].Style_id_Color_Size_BackGro_text[1],0,sizeof(unsigned short));
		memset(&LabelCmdMsg[idx].Style_id_Color_Size_BackGro_text[3],14,sizeof(unsigned char));
		memset(&LabelCmdMsg[idx].Style_id_Color_Size_BackGro_text[4],0xFF,sizeof(unsigned short));
		memset(&LabelCmdMsg[idx].Style_id_Color_Size_BackGro_text[6],1,sizeof(unsigned char));
		LabelCmdMsg[idx].Text_id			= id;
		//strcpy(LabelCmdMsg[idx].Text,"CG    ");		//	3
		//LabelCmdMsg[idx].Text[strlen(LabelCmdMsg[idx].Text)] = 0;		

		id++;	// 11
		idx++;
		LabelCmdMsg[idx].Label_id_Visibility[0]		= id;	// 11
		LabelCmdMsg[idx].Label_id_Visibility[1]		= Visibility;
		LabelCmdMsg[idx].Position_id_X_Y[0]		= id;
		LabelCmdMsg[idx].Position_id_X_Y[1]		= 24;	// X
		LabelCmdMsg[idx].Position_id_X_Y[3]		= 120;	// Y
		memcpy(&LabelCmdMsg[idx].Style_id_Color_Size_BackGro_text[0],(unsigned char *)&id,sizeof(unsigned char));	
		memset(&LabelCmdMsg[idx].Style_id_Color_Size_BackGro_text[1],0,sizeof(unsigned short));
		memset(&LabelCmdMsg[idx].Style_id_Color_Size_BackGro_text[3],14,sizeof(unsigned char));
		memset(&LabelCmdMsg[idx].Style_id_Color_Size_BackGro_text[4],0xFF,sizeof(unsigned short));
		memset(&LabelCmdMsg[idx].Style_id_Color_Size_BackGro_text[6],1,sizeof(unsigned char));
		LabelCmdMsg[idx].Text_id			= id;
		//strcpy(LabelCmdMsg[idx].Text,"CG    ");		//	3
		//LabelCmdMsg[idx].Text[strlen(LabelCmdMsg[idx].Text)] = 0;

		id++;	// 12
		idx++;
		LabelCmdMsg[idx].Label_id_Visibility[0]		= id;	// 12
		LabelCmdMsg[idx].Label_id_Visibility[1]		= Visibility;
		LabelCmdMsg[idx].Position_id_X_Y[0]		= id;
		LabelCmdMsg[idx].Position_id_X_Y[1]		= 24;	// X
		LabelCmdMsg[idx].Position_id_X_Y[3]		= 130;	// Y
		memcpy(&LabelCmdMsg[idx].Style_id_Color_Size_BackGro_text[0],(unsigned char *)&id,sizeof(unsigned char));	
		memset(&LabelCmdMsg[idx].Style_id_Color_Size_BackGro_text[1],0,sizeof(unsigned short));
		memset(&LabelCmdMsg[idx].Style_id_Color_Size_BackGro_text[3],14,sizeof(unsigned char));
		memset(&LabelCmdMsg[idx].Style_id_Color_Size_BackGro_text[4],0xFF,sizeof(unsigned short));
		memset(&LabelCmdMsg[idx].Style_id_Color_Size_BackGro_text[6],1,sizeof(unsigned char));
		LabelCmdMsg[idx].Text_id			= id;
		//strcpy(LabelCmdMsg[idx].Text,"CG    ");		//	3
		//LabelCmdMsg[idx].Text[strlen(LabelCmdMsg[idx].Text)] = 0;		

		id++;	//13
		idx++;
		LabelCmdMsg[idx].Label_id_Visibility[0]		= id;	// 13
		LabelCmdMsg[idx].Label_id_Visibility[1]		= Visibility;
		LabelCmdMsg[idx].Position_id_X_Y[0]		= id;
		LabelCmdMsg[idx].Position_id_X_Y[1]		= 24;	// X
		LabelCmdMsg[idx].Position_id_X_Y[3]		= 140;	// Y
		memcpy(&LabelCmdMsg[idx].Style_id_Color_Size_BackGro_text[0],(unsigned char *)&id,sizeof(unsigned char));	
		memset(&LabelCmdMsg[idx].Style_id_Color_Size_BackGro_text[1],0,sizeof(unsigned short));
		memset(&LabelCmdMsg[idx].Style_id_Color_Size_BackGro_text[3],14,sizeof(unsigned char));
		memset(&LabelCmdMsg[idx].Style_id_Color_Size_BackGro_text[4],0xFF,sizeof(unsigned short));
		memset(&LabelCmdMsg[idx].Style_id_Color_Size_BackGro_text[6],1,sizeof(unsigned char));
		LabelCmdMsg[idx].Text_id			= id;
		//strcpy(LabelCmdMsg[idx].Text,"CG    ");		//	3
		//LabelCmdMsg[idx].Text[strlen(LabelCmdMsg[idx].Text)] = 0;		

		id++;	// 14
		idx++;
		LabelCmdMsg[idx].Label_id_Visibility[0]		= id;	// 14
		LabelCmdMsg[idx].Label_id_Visibility[1]		= Visibility;
		LabelCmdMsg[idx].Position_id_X_Y[0]		= id;
		LabelCmdMsg[idx].Position_id_X_Y[1]		= 24;	// X
		LabelCmdMsg[idx].Position_id_X_Y[3]		= 150;	// Y
		memcpy(&LabelCmdMsg[idx].Style_id_Color_Size_BackGro_text[0],(unsigned char *)&id,sizeof(unsigned char));	
		memset(&LabelCmdMsg[idx].Style_id_Color_Size_BackGro_text[1],0,sizeof(unsigned short));
		memset(&LabelCmdMsg[idx].Style_id_Color_Size_BackGro_text[3],14,sizeof(unsigned char));
		memset(&LabelCmdMsg[idx].Style_id_Color_Size_BackGro_text[4],0xFF,sizeof(unsigned short));
		memset(&LabelCmdMsg[idx].Style_id_Color_Size_BackGro_text[6],1,sizeof(unsigned char));
		LabelCmdMsg[idx].Text_id			= id;
		//strcpy(LabelCmdMsg[idx].Text,"CG    ");		//	3
		//LabelCmdMsg[idx].Text[strlen(LabelCmdMsg[idx].Text)] = 0;		

		id++;	// 15
		idx++;
		LabelCmdMsg[idx].Label_id_Visibility[0]		= id;	// 15
		LabelCmdMsg[idx].Label_id_Visibility[1]		= Visibility;
		LabelCmdMsg[idx].Position_id_X_Y[0]		= id;
		LabelCmdMsg[idx].Position_id_X_Y[1]		= 24;	// X
		LabelCmdMsg[idx].Position_id_X_Y[3]		= 160;	// Y
		memcpy(&LabelCmdMsg[idx].Style_id_Color_Size_BackGro_text[0],(unsigned char *)&id,sizeof(unsigned char));	
		memset(&LabelCmdMsg[idx].Style_id_Color_Size_BackGro_text[1],0,sizeof(unsigned short));
		memset(&LabelCmdMsg[idx].Style_id_Color_Size_BackGro_text[3],14,sizeof(unsigned char));
		memset(&LabelCmdMsg[idx].Style_id_Color_Size_BackGro_text[4],0xFF,sizeof(unsigned short));
		memset(&LabelCmdMsg[idx].Style_id_Color_Size_BackGro_text[6],1,sizeof(unsigned char));
		LabelCmdMsg[idx].Text_id			= id;
		//strcpy(LabelCmdMsg[idx].Text,"CG    ");		//	3
		//LabelCmdMsg[idx].Text[strlen(LabelCmdMsg[idx].Text)] = 0;		

		id++;	// 16
		idx++;
		LabelCmdMsg[idx].Label_id_Visibility[0]		= id;	// 16
		LabelCmdMsg[idx].Label_id_Visibility[1]		= Visibility;
		LabelCmdMsg[idx].Position_id_X_Y[0]		= id;
		LabelCmdMsg[idx].Position_id_X_Y[1]		= 5;	// X
		LabelCmdMsg[idx].Position_id_X_Y[3]		= 170;	// Y
		memcpy(&LabelCmdMsg[idx].Style_id_Color_Size_BackGro_text[0],(unsigned char *)&id,sizeof(unsigned char));	
		memset(&LabelCmdMsg[idx].Style_id_Color_Size_BackGro_text[1],0xFF,sizeof(unsigned short));
		memset(&LabelCmdMsg[idx].Style_id_Color_Size_BackGro_text[3],14,sizeof(unsigned char));
		memset(&LabelCmdMsg[idx].Style_id_Color_Size_BackGro_text[4],0,sizeof(unsigned short));
		memset(&LabelCmdMsg[idx].Style_id_Color_Size_BackGro_text[6],1,sizeof(unsigned char));
		LabelCmdMsg[idx].Text_id			= id;
		//strcpy(LabelCmdMsg[idx].Text,"CG    ");		//	3
		//LabelCmdMsg[idx].Text[strlen(LabelCmdMsg[idx].Text)] = 0;		

		id++;	// 17
		idx++;
		LabelCmdMsg[idx].Label_id_Visibility[0]		= id;	// 17
		LabelCmdMsg[idx].Label_id_Visibility[1]		= Visibility;
		LabelCmdMsg[idx].Position_id_X_Y[0]		= id;
		LabelCmdMsg[idx].Position_id_X_Y[1]		= 24;	// X
		LabelCmdMsg[idx].Position_id_X_Y[3]		= 180;	// Y
		memcpy(&LabelCmdMsg[idx].Style_id_Color_Size_BackGro_text[0],(unsigned char *)&id,sizeof(unsigned char));	
		memset(&LabelCmdMsg[idx].Style_id_Color_Size_BackGro_text[1],0,sizeof(unsigned short));
		memset(&LabelCmdMsg[idx].Style_id_Color_Size_BackGro_text[3],14,sizeof(unsigned char));
		memset(&LabelCmdMsg[idx].Style_id_Color_Size_BackGro_text[4],0xFF,sizeof(unsigned short));
		memset(&LabelCmdMsg[idx].Style_id_Color_Size_BackGro_text[6],1,sizeof(unsigned char));
		LabelCmdMsg[idx].Text_id			= id;
		//strcpy(LabelCmdMsg[idx].Text,"CG    ");		//	3
		//LabelCmdMsg[idx].Text[strlen(LabelCmdMsg[idx].Text)] = 0;				

		id++;	// 18
		idx++;
		LabelCmdMsg[idx].Label_id_Visibility[0]		= id;	// 18
		LabelCmdMsg[idx].Label_id_Visibility[1]		= Visibility;
		LabelCmdMsg[idx].Position_id_X_Y[0]		= id;
		LabelCmdMsg[idx].Position_id_X_Y[1]		= 24;	// X
		LabelCmdMsg[idx].Position_id_X_Y[3]		= 190;	// Y
		memcpy(&LabelCmdMsg[idx].Style_id_Color_Size_BackGro_text[0],(unsigned char *)&id,sizeof(unsigned char));	
		memset(&LabelCmdMsg[idx].Style_id_Color_Size_BackGro_text[1],0,sizeof(unsigned short));
		memset(&LabelCmdMsg[idx].Style_id_Color_Size_BackGro_text[3],14,sizeof(unsigned char));
		memset(&LabelCmdMsg[idx].Style_id_Color_Size_BackGro_text[4],0xFF,sizeof(unsigned short));
		memset(&LabelCmdMsg[idx].Style_id_Color_Size_BackGro_text[6],1,sizeof(unsigned char));
		LabelCmdMsg[idx].Text_id			= id;
		//strcpy(LabelCmdMsg[idx].Text,"CG    ");		//	3
		//LabelCmdMsg[idx].Text[strlen(LabelCmdMsg[idx].Text)] = 0;				

		id++;	// 19
		idx++;
		LabelCmdMsg[idx].Label_id_Visibility[0]		= id;	// 19
		LabelCmdMsg[idx].Label_id_Visibility[1]		= Visibility;
		LabelCmdMsg[idx].Position_id_X_Y[0]		= id;
		LabelCmdMsg[idx].Position_id_X_Y[1]		= 24;	// X
		LabelCmdMsg[idx].Position_id_X_Y[3]		= 200;	// Y
		memcpy(&LabelCmdMsg[idx].Style_id_Color_Size_BackGro_text[0],(unsigned char *)&id,sizeof(unsigned char));	
		memset(&LabelCmdMsg[idx].Style_id_Color_Size_BackGro_text[1],0,sizeof(unsigned short));
		memset(&LabelCmdMsg[idx].Style_id_Color_Size_BackGro_text[3],14,sizeof(unsigned char));
		memset(&LabelCmdMsg[idx].Style_id_Color_Size_BackGro_text[4],0xFF,sizeof(unsigned short));
		memset(&LabelCmdMsg[idx].Style_id_Color_Size_BackGro_text[6],1,sizeof(unsigned char));
		LabelCmdMsg[idx].Text_id			= id;
		//strcpy(LabelCmdMsg[idx].Text,"CG    ");		//	3
		//LabelCmdMsg[idx].Text[strlen(LabelCmdMsg[idx].Text)] = 0;				

		id++;	// 20
		idx++;
		LabelCmdMsg[idx].Label_id_Visibility[0]		= id;	// 20
		LabelCmdMsg[idx].Label_id_Visibility[1]		= Visibility;
		LabelCmdMsg[idx].Position_id_X_Y[0]		= id;
		LabelCmdMsg[idx].Position_id_X_Y[1]		= 24;	// X
		LabelCmdMsg[idx].Position_id_X_Y[3]		= 210;	// Y
		memcpy(&LabelCmdMsg[idx].Style_id_Color_Size_BackGro_text[0],(unsigned char *)&id,sizeof(unsigned char));	
		memset(&LabelCmdMsg[idx].Style_id_Color_Size_BackGro_text[1],0,sizeof(unsigned short));
		memset(&LabelCmdMsg[idx].Style_id_Color_Size_BackGro_text[3],14,sizeof(unsigned char));
		memset(&LabelCmdMsg[idx].Style_id_Color_Size_BackGro_text[4],0xFF,sizeof(unsigned short));
		memset(&LabelCmdMsg[idx].Style_id_Color_Size_BackGro_text[6],1,sizeof(unsigned char));
		LabelCmdMsg[idx].Text_id			= id;
		//strcpy(LabelCmdMsg[idx].Text,"CG    ");		//	3
		//LabelCmdMsg[idx].Text[strlen(LabelCmdMsg[idx].Text)] = 0;				


		id++;	// 21
		idx++;
		LabelCmdMsg[idx].Label_id_Visibility[0]		= id;	// 21
		LabelCmdMsg[idx].Label_id_Visibility[1]		= Visibility;
		LabelCmdMsg[idx].Position_id_X_Y[0]		= id;
		LabelCmdMsg[idx].Position_id_X_Y[1]		= 24;	// X
		LabelCmdMsg[idx].Position_id_X_Y[3]		= 220;	// Y
		memcpy(&LabelCmdMsg[idx].Style_id_Color_Size_BackGro_text[0],(unsigned char *)&id,sizeof(unsigned char));	
		memset(&LabelCmdMsg[idx].Style_id_Color_Size_BackGro_text[1],0,sizeof(unsigned short));
		memset(&LabelCmdMsg[idx].Style_id_Color_Size_BackGro_text[3],14,sizeof(unsigned char));
		memset(&LabelCmdMsg[idx].Style_id_Color_Size_BackGro_text[4],0xFF,sizeof(unsigned short));
		memset(&LabelCmdMsg[idx].Style_id_Color_Size_BackGro_text[6],1,sizeof(unsigned char));
		LabelCmdMsg[idx].Text_id			= id;
		//strcpy(LabelCmdMsg[idx].Text,"CG    ");		//	3
		//LabelCmdMsg[idx].Text[strlen(LabelCmdMsg[idx].Text)] = 0;				

		id++;	// 22
		idx++;
		LabelCmdMsg[idx].Label_id_Visibility[0]		= id;	// 22
		LabelCmdMsg[idx].Label_id_Visibility[1]		= Visibility;
		LabelCmdMsg[idx].Position_id_X_Y[0]		= id;
		LabelCmdMsg[idx].Position_id_X_Y[1]		= 24;	// X
		LabelCmdMsg[idx].Position_id_X_Y[3]		= 230;	// Y
		memcpy(&LabelCmdMsg[idx].Style_id_Color_Size_BackGro_text[0],(unsigned char *)&id,sizeof(unsigned char));	
		memset(&LabelCmdMsg[idx].Style_id_Color_Size_BackGro_text[1],0,sizeof(unsigned short));
		memset(&LabelCmdMsg[idx].Style_id_Color_Size_BackGro_text[3],14,sizeof(unsigned char));
		memset(&LabelCmdMsg[idx].Style_id_Color_Size_BackGro_text[4],0xFF,sizeof(unsigned short));
		memset(&LabelCmdMsg[idx].Style_id_Color_Size_BackGro_text[6],1,sizeof(unsigned char));
		LabelCmdMsg[idx].Text_id			= id;
		//strcpy(LabelCmdMsg[idx].Text,"CG    ");		//	3
		//LabelCmdMsg[idx].Text[strlen(LabelCmdMsg[idx].Text)] = 0;


		id++;	// 23
		idx++;
		LabelCmdMsg[idx].Label_id_Visibility[0]		= id;	// 23
		LabelCmdMsg[idx].Label_id_Visibility[1]		= Visibility;
		LabelCmdMsg[idx].Position_id_X_Y[0]		= id;
		LabelCmdMsg[idx].Position_id_X_Y[1]		= 5;	// X
		LabelCmdMsg[idx].Position_id_X_Y[3]		= 240;	// Y
		memcpy(&LabelCmdMsg[idx].Style_id_Color_Size_BackGro_text[0],(unsigned char *)&id,sizeof(unsigned char));	
		memset(&LabelCmdMsg[idx].Style_id_Color_Size_BackGro_text[1],0xFF,sizeof(unsigned short));
		memset(&LabelCmdMsg[idx].Style_id_Color_Size_BackGro_text[3],14,sizeof(unsigned char));
		memset(&LabelCmdMsg[idx].Style_id_Color_Size_BackGro_text[4],0,sizeof(unsigned short));
		memset(&LabelCmdMsg[idx].Style_id_Color_Size_BackGro_text[6],1,sizeof(unsigned char));
		LabelCmdMsg[idx].Text_id			= id;
		//strcpy(LabelCmdMsg[idx].Text,"CG    ");		//	3
		//LabelCmdMsg[idx].Text[strlen(LabelCmdMsg[idx].Text)] = 0;	

		id++;	// 24
		idx++;
		LabelCmdMsg[idx].Label_id_Visibility[0]		= id;	// 24
		LabelCmdMsg[idx].Label_id_Visibility[1]		= Visibility;
		LabelCmdMsg[idx].Position_id_X_Y[0]		= id;
		LabelCmdMsg[idx].Position_id_X_Y[1]		= 56;	// X
		LabelCmdMsg[idx].Position_id_X_Y[3]		= 14;	// Y
		LabelCmdMsg[idx].Position_id_X_Y[4]		= 1;	// Y		
		memcpy(&LabelCmdMsg[idx].Style_id_Color_Size_BackGro_text[0],(unsigned char *)&id,sizeof(unsigned char));	
		memset(&LabelCmdMsg[idx].Style_id_Color_Size_BackGro_text[1],0,sizeof(unsigned short));
		memset(&LabelCmdMsg[idx].Style_id_Color_Size_BackGro_text[3],14,sizeof(unsigned char));
		memset(&LabelCmdMsg[idx].Style_id_Color_Size_BackGro_text[4],0xFF,sizeof(unsigned short));
		memset(&LabelCmdMsg[idx].Style_id_Color_Size_BackGro_text[6],1,sizeof(unsigned char));
		LabelCmdMsg[idx].Text_id			= id;
		//strcpy(LabelCmdMsg[idx].Text,"CG    ");		//	3
		//LabelCmdMsg[idx].Text[strlen(LabelCmdMsg[idx].Text)] = 0;


		id++;	// 25
		idx++;
		LabelCmdMsg[idx].Label_id_Visibility[0]		= id;	// 25
		LabelCmdMsg[idx].Label_id_Visibility[1]		= Visibility;
		LabelCmdMsg[idx].Position_id_X_Y[0]		= id;
		LabelCmdMsg[idx].Position_id_X_Y[1]		= 125;	// X
		LabelCmdMsg[idx].Position_id_X_Y[3]		= 14;	// Y
		LabelCmdMsg[idx].Position_id_X_Y[4]		= 1;	// Y		
		memcpy(&LabelCmdMsg[idx].Style_id_Color_Size_BackGro_text[0],(unsigned char *)&id,sizeof(unsigned char));	
		memset(&LabelCmdMsg[idx].Style_id_Color_Size_BackGro_text[1],0,sizeof(unsigned short));
		memset(&LabelCmdMsg[idx].Style_id_Color_Size_BackGro_text[3],14,sizeof(unsigned char));
		memset(&LabelCmdMsg[idx].Style_id_Color_Size_BackGro_text[4],0xFF,sizeof(unsigned short));
		memset(&LabelCmdMsg[idx].Style_id_Color_Size_BackGro_text[6],1,sizeof(unsigned char));
		LabelCmdMsg[idx].Text_id			= id;
		//strcpy(LabelCmdMsg[idx].Text,"CG    ");		//	3
		//LabelCmdMsg[idx].Text[strlen(LabelCmdMsg[idx].Text)] = 0;	

		
}

 
/*
 * FUNCTION:  COM1 OmapMessageLoop - Main Com Proc message processing loop
 * RETURN: Non-zero of message detected  
 */
int
OmapMessageLoop(unsigned long ulBase,unsigned char cid)
{
   char ch;
   //int i = 0;
   unsigned short Checksum = 0;
   
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
          case MSG_FLAG: // Locks for a FLAG character
          
             if (ch == HEADER_SYNC)
             {             				 				 
                 if (cpib[cid].idx > 1u)
                 {
                    message[cid][0] = ch;
                    cpib[cid].idx = 1u;
                 }
                 cpib[cid].state++; // Gos to next state
             }
             
             break;

          case MSG_CMD_LSB: // Receive the message comman
             cpib[cid].state++;
             
             break;

          case MSG_CMD_MSB: // Receive the message comman
             cpib[cid].state++;
             
             break;

          case MSG_LEN_LSB: // Receive message length (2 bytes) 
          
             cpib[cid].len = (unsigned short)ch;
             cpib[cid].state++;
			         
			 
             break;

          case MSG_LEN_MSB:
             cpib[cid].len |= (unsigned short)ch << 8;
             cpib[cid].state++;
			 cpib[cid].len += 6u;
			
             break;
 
         case MSG_DATA:

             if (cpib[cid].idx >= (cpib[cid].len)) // Counts the received bytes
             {
			   cpib[cid].len -= 6u;             
               cpib[cid].idx = 0;
               cpib[cid].state = 0;
			   cpib[cid].checksum = 0;
			   Checksum = 0;

               /*
                		* Checks Checksum
                		*/
               Checksum = CalculateChecksumMsg(COM1,&message[cid][0],cpib[cid].len+MSG_DATA);	

			   
               //for(i=0;i<cpib[cid].len-1;i++)
			   	//Checksum(cid, message[cid][i]);
			   
			   //cpib[cid].checksum %= 256;
			   //cpib[cid].checksum = (unsigned char)((~cpib[cid].checksum) + 1);

				
               if (Checksum == message[cid][cpib[cid].len+MSG_DATA])
               {		  
               
				  cpib[cid].ready = (unsigned short) message[cid][MSG_CMD_LSB];               
                  cpib[cid].ready |= (unsigned short) message[cid][MSG_CMD_MSB]<<8;

               }
               else
               {   
                  cpReset(COM1);
                  cpib[cid].ready = 0;
                  cpib[cid].len = 0;
               }
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
             //   return cpib[cid].ready;

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
 
FUNCTION:  cpPutMessage   
 
INTERNAL DESCRIPTION: preper the TX msg for transmit
 			  			  
----------------------------------------------------------------------------!*/

void cpPutMessage(unsigned long ulBase,unsigned short cmd,unsigned short data_len,unsigned char *msg)
{
	//unsigned char chacksum = 0;
	unsigned short total_len,headerDataLen;
	BB_TXRX_S *pBbMsg=(BB_TXRX_S *)BbTxBuffer;

	headerDataLen=(BIRD384_HEADER_LEN+data_len);
	total_len=(headerDataLen+TAILER_SIZE);

	pBbMsg->sync=0xAA;
	memcpy(pBbMsg->command,(unsigned short *)&cmd,sizeof(unsigned short));	
	memcpy(pBbMsg->msg_len,(unsigned short *)&data_len,sizeof(unsigned short));
	memcpy(pBbMsg->buffer,msg,data_len);
	pBbMsg->buffer[data_len]=CalculateChecksumMsg(COM1,(unsigned char *)pBbMsg,(data_len+BIRD384_HEADER_LEN));
 	UART1Send(BbTxBuffer, total_len);  		
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


double GetCaseTemperature(void)
{
   return m_TEC_Params.Case_tem;
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

	  //char zoom;
	  //char zoom;
	  //WORD	 id = 0;
	  //static float k_alt = 34.0f,k_lon = 32.0f;	  
	  float tmpPhic=0,tmpThetac=0;

      if (cpMessageReady(COM1))
      {            
         // Read Comm message
         WORD m = cpGetMessageCommand(COM1);         
         WORD f = GetDMCflag(0xffff);

         if((f && 0x0010) )
         {
            mode = m;
 
         }      
      }
	  #if 1
	  else 
	  if ((WORD)(g_ulTickCount - ctrl.time[ONLINE]) >= (WORD)500) //-- every 1 sec
	  {	  
				
	  			#if 1
				//cAz+=0.01;
			    //tmpPhic = (Phic * RAD2DEG);	  
			    //tmpThetac= (Thetac* RAD2DEG);   	  

			    sprintf(LabelCmdMsg[24].Text,"%.2f",(Phic * RAD2DEG));	   //  9	Roll
			    sprintf(LabelCmdMsg[25].Text,"%.2f",(Thetac* RAD2DEG));	   //  9	Pitch		

				sprintf(LabelCmdMsg[OSD_ROLL_DEG].Text,"%.1f",Motor[ROLL_MOTOR].aangle);	   //  9	Roll
			    sprintf(LabelCmdMsg[OSD_PITCH_DEG].Text,"%.1f",Motor[PITCH_MOTOR].aangle);	   //  9	Pitch		

			    //sprintf(LabelCmdMsg[OSD_VERSION_ID].Text,"EncP:%.2f  EncR:%.2f",Motor[PITCH_MOTOR].aangle,Motor[ROLL_MOTOR].aangle);	   //  9					  

				if(MotorPositionError(PITCH_MOTOR) == -1)
					sprintf(LabelCmdMsg[OSD_VERSION_ID].Text,"PITCH ERROR!!!");	   //  9													
				else
				if(MotorPositionError(ROLL_MOTOR) == -1)
					sprintf(LabelCmdMsg[OSD_VERSION_ID].Text,"ROLL ERROR!!!");	   //  9													
				else			    
			    sprintf(LabelCmdMsg[OSD_VERSION_ID].Text,"DirR:%u pwmP:%u  pwmR:%u",mdirection[ROLL_MOTOR],dcycle[PITCH_MOTOR],dcycle[ROLL_MOTOR]);	   //  9					  								
			    //sprintf(LabelCmdMsg[OSD_VERSION_ID].Text,"Tmp:%d pwmP:%u  pwmR:%u",bmaregs[TEMPERATURE],dcycle[PITCH_MOTOR],dcycle[ROLL_MOTOR]);	   //  9					  								
			    
			    //sprintf(LabelCmdMsg[OSD_VERSION_ID].Text,"Axm:%.2f  Aym:%.2f Azm:%.2f g:%.2f",Axm,Aym,Azm,g);	   //  9					  			    
			    LabelCmdMsg[OSD_PITCH_DEG].Text[strlen(LabelCmdMsg[OSD_PITCH_DEG].Text)] = 0; 
				LabelCmdMsg[OSD_ROLL_DEG].Text[strlen(LabelCmdMsg[OSD_ROLL_DEG].Text)] = 0;
			    LabelCmdMsg[OSD_VERSION_ID].Text[strlen(LabelCmdMsg[OSD_VERSION_ID].Text)] = 0;
				LabelCmdMsg[24].Text[strlen(LabelCmdMsg[24].Text)] = 0;
			    LabelCmdMsg[25].Text[strlen(LabelCmdMsg[25].Text)] = 0;
				
			    ctrl.state[TEXT_LABEL1] = LEBEL_TEXT; 					  
			    Put_Text_Label(OSD_PITCH_DEG,1);
				
			    ctrl.state[TEXT_LABEL1] = LEBEL_TEXT; 					  
			    Put_Text_Label(OSD_ROLL_DEG,1);
				
				ctrl.state[TEXT_LABEL1] = LEBEL_TEXT; 
			    Put_Text_Label(OSD_VERSION_ID,1);  

				ctrl.state[TEXT_LABEL1] = LEBEL_TEXT; 
			    Put_Text_Label(24,1); 

				ctrl.state[TEXT_LABEL1] = LEBEL_TEXT; 
			    Put_Text_Label(25,1); 				
  				#endif
				
				ctrl.time[ONLINE] = g_ulTickCount;	
				BBLedToggle1();
	  }	  
	  #endif
   return mode;
}



	
/*
 * FUNCTION: InitializationUnit - Super state on OFFLINE mode
 */
WORD InitializationUnit(WORD mode)
{
   if ((WORD)(g_ulTickCount - ctrl.time[ONLINE]) >= (WORD)35000) //-- every 1 sec
   {   
       ctrl.time[OFFLINE] = GetSysTickCount();
	   return DOWNLOAD_VERSION;
   }
   else
	  ctrl.time[OFFLINE] = g_ulTickCount;

   return INITIALIZATION_UNIT;   

   
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

      }
	  else 
	  if ((WORD)(g_ulTickCount - ctrl.time[OFFLINE]) >= (WORD)25000) //-- every 15 sec
	  {	  
				ctrl.time[OFFLINE] = g_ulTickCount;
					mode = DOWNLOAD_VERSION;						
	  }
	  
   return mode;
}


WORD Download_Engine_Version(WORD mode)
{
	
	
	ctrl.ret_mode[ACT_11] = 0xF011;
	cpPutMessage(UART1_BASE,0xF011,0,NULL);
	return UPLOAD_VERSION;

}


WORD Upload_Engine_Version(WORD mode)
{

	WORD n;
	WORD   id = 0;
	WORD i = 4;
	WORD ret = 0;
										
	if (cpMessageReady(COM1))
	{
		n = cpGetMessageCommand(COM1); //tbd
		if (n==ctrl.ret_mode[ACT_11])
		{			
	
			cpReset(COM1);		
			ctrl.try[ACT_11] = 0;		
			
			sprintf(LabelCmdMsg[1].Text,"<<"); 	//	9					
			LabelCmdMsg[1].Text[strlen(LabelCmdMsg[1].Text)] = 0;					
			Set_LABEL_Visibility(OSD_ZOOM_ID,1);

			if(GetDMCflag(DMC_BLACK_HOT))
			{
				strcpy(LabelCmdMsg[2].Text,"B-W");
				LabelCmdMsg[2].Text[strlen(LabelCmdMsg[2].Text)] = 0;										
				Set_LABEL_Visibility(OSD_BLACK_WHITE_HOT_ID,1);
			
			}
			else
			{
				strcpy(LabelCmdMsg[2].Text,"W-H");
				LabelCmdMsg[2].Text[strlen(LabelCmdMsg[2].Text)] = 0;										
				Set_LABEL_Visibility(OSD_BLACK_WHITE_HOT_ID,1);			
			}

			sprintf(LabelCmdMsg[OSD_VERSION_ID].Text,"%s","Paylod Bit"); 	//	9					
			LabelCmdMsg[OSD_VERSION_ID].Text[strlen(LabelCmdMsg[OSD_VERSION_ID].Text)] = 0;					
			Set_LABEL_Visibility(OSD_VERSION_ID,1);
			
			if (GetDMCflag(DMC_STOW))
			{
				strcpy(LabelCmdMsg[0].Text,"STOW");
				LabelCmdMsg[0].Text[strlen(LabelCmdMsg[0].Text)] = 0;										
				Set_LABEL_Visibility(OSD_MODE_ID,1);
			
			}
			else
			if (GetDMCflag(DMC_PILOT_WINDOW))
			{
				strcpy(LabelCmdMsg[0].Text,"PILOT");
				LabelCmdMsg[0].Text[strlen(LabelCmdMsg[0].Text)] = 0;										
				Set_LABEL_Visibility(OSD_MODE_ID,1);
			
			}
			else
			{
				strcpy(LabelCmdMsg[0].Text,"RATE");
				LabelCmdMsg[0].Text[strlen(LabelCmdMsg[0].Text)] = 0;										
				Set_LABEL_Visibility(OSD_MODE_ID,1);			
			}

			strcpy(LabelCmdMsg[i].Text,"Roll:");
			LabelCmdMsg[i].Text[strlen(LabelCmdMsg[i].Text)] = 0;										
			Set_LABEL_Visibility(i,1);
			i++;

			strcpy(LabelCmdMsg[i].Text,"Pitch:");
			LabelCmdMsg[i].Text[strlen(LabelCmdMsg[i].Text)] = 0;										
			Set_LABEL_Visibility(i,1);
			i++;

			strcpy(LabelCmdMsg[i].Text,"0.0");
			LabelCmdMsg[i].Text[strlen(LabelCmdMsg[i].Text)] = 0;										
			Set_LABEL_Visibility(i,1);
			i++;

			strcpy(LabelCmdMsg[i].Text,"0.0:");
			LabelCmdMsg[i].Text[strlen(LabelCmdMsg[i].Text)] = 0;										
			Set_LABEL_Visibility(i,1);
			i++;

			// i ==8 
			strcpy(LabelCmdMsg[i].Text,"ZOOM");
			LabelCmdMsg[i].Text[strlen(LabelCmdMsg[i].Text)] = 0;										
			Set_LABEL_Visibility(i,1);
			i++;

			strcpy(LabelCmdMsg[i].Text,"Zx0-|");
			LabelCmdMsg[i].Text[strlen(LabelCmdMsg[i].Text)] = 0;										
			Set_LABEL_Visibility(i,1);
			i++;

			strcpy(LabelCmdMsg[i].Text,"|");
			LabelCmdMsg[i].Text[strlen(LabelCmdMsg[i].Text)] = 0;										
			Set_LABEL_Visibility(i,1);
			i++;
			
			strcpy(LabelCmdMsg[i].Text,"|");
			LabelCmdMsg[i].Text[strlen(LabelCmdMsg[i].Text)] = 0;										
			Set_LABEL_Visibility(i,1);			
			i++;

			strcpy(LabelCmdMsg[i].Text,"|");
			LabelCmdMsg[i].Text[strlen(LabelCmdMsg[i].Text)] = 0;										
			Set_LABEL_Visibility(i,1);			
			i++;
			
			strcpy(LabelCmdMsg[i].Text,"|");
			LabelCmdMsg[i].Text[strlen(LabelCmdMsg[i].Text)] = 0;										
			Set_LABEL_Visibility(i,1);			
			i++;

			strcpy(LabelCmdMsg[i].Text,"|");
			LabelCmdMsg[i].Text[strlen(LabelCmdMsg[i].Text)] = 0;										
			Set_LABEL_Visibility(i,1);			
			i++;
			
			strcpy(LabelCmdMsg[i].Text,"|");
			LabelCmdMsg[i].Text[strlen(LabelCmdMsg[i].Text)] = 0;										
			Set_LABEL_Visibility(i,1);			
			i++;

			strcpy(LabelCmdMsg[i].Text,"Zx2-|");
			LabelCmdMsg[i].Text[strlen(LabelCmdMsg[i].Text)] = 0;										
			Set_LABEL_Visibility(i,1);			
			i++;			

			strcpy(LabelCmdMsg[i].Text,"|");
			LabelCmdMsg[i].Text[strlen(LabelCmdMsg[i].Text)] = 0;										
			Set_LABEL_Visibility(i,1);			
			i++;

			strcpy(LabelCmdMsg[i].Text,"|");
			LabelCmdMsg[i].Text[strlen(LabelCmdMsg[i].Text)] = 0;										
			Set_LABEL_Visibility(i,1);			
			i++;

			strcpy(LabelCmdMsg[i].Text,"|");
			LabelCmdMsg[i].Text[strlen(LabelCmdMsg[i].Text)] = 0;										
			Set_LABEL_Visibility(i,1);			
			i++;	
			
			strcpy(LabelCmdMsg[i].Text,"|");
			LabelCmdMsg[i].Text[strlen(LabelCmdMsg[i].Text)] = 0;										
			Set_LABEL_Visibility(i,1);			
			i++;

			strcpy(LabelCmdMsg[i].Text,"|");
			LabelCmdMsg[i].Text[strlen(LabelCmdMsg[i].Text)] = 0;										
			Set_LABEL_Visibility(i,1);				
			i++;

			strcpy(LabelCmdMsg[i].Text,"|");
			LabelCmdMsg[i].Text[strlen(LabelCmdMsg[i].Text)] = 0;										
			Set_LABEL_Visibility(i,1);				
			i++;

			strcpy(LabelCmdMsg[i].Text,"Zx4-|");
			LabelCmdMsg[i].Text[strlen(LabelCmdMsg[i].Text)] = 0;										
			Set_LABEL_Visibility(i,1);	
			i++;
			
			strcpy(LabelCmdMsg[i].Text,"0.0");
			LabelCmdMsg[i].Text[strlen(LabelCmdMsg[i].Text)] = 0;										
			Set_LABEL_Visibility(i,1);	
			i++;
			
			strcpy(LabelCmdMsg[i].Text,"1.1");
			LabelCmdMsg[i].Text[strlen(LabelCmdMsg[i].Text)] = 0;										
			Set_LABEL_Visibility(i,1);	
			i++;

			
			//ctrl.state[TEXT_LABEL1] = HIDE_SHOW_LABEL;

			//for(id=0;id<MAX_LABEL_LENGTH;id++)
				//Set_Text_Label(id,1);

			//ctrl.state[TEXT_LABEL1] = HIDE_SHOW_LABEL;

			//for(id=0;id<MAX_LABEL_LENGTH;id++)
				//Set_Text_Label(id,1);

			ctrl.state[TEXT_LABEL1] = HIDE_SHOW_LABEL;

			for(id=0;id<MAX_LABEL_LENGTH;id++)
			{
				if(Set_Text_Label(id,1) == 0)		
					id = 0;
			}
			
			cpReset(COM1);
			SetDMCflag(DMC_ENGINE_CONNECT,DMC_ENGINE_CONNECT);
			mode = ONLINE_STATE;
		
		}
		else
			cpReset(COM1);
		  
	}
	else
	if ((WORD)(GetSysTickCount() - ctrl.time[ACT_11]) >= (WORD)5000) //-- every 3 sec
	{			
		  cpPutMessage(UART1_BASE,0xF011,0,NULL);	
		  ctrl.time[ACT_11]	= g_ulTickCount;
		  
		  if(ctrl.try[ACT_11]++ > 3)
		  {		  
			  ctrl.time[ACT_11]	= g_ulTickCount;		  
			  ctrl.time[ONLINE] = g_ulTickCount;
			  ctrl.try[ACT_11] = 0;									  
			  cpReset(COM1);				 
			  mode = INITIALIZATION_UNIT;			  
		  }
	}					

	return mode;


}


WORD Upload_General_Parameters(WORD mode)
{
	
	ctrl.ret_mode[TRAN_16] = 0xF018;
	//ctrl.next_mode = ONLINE_STATE;		
	ctrl.next_mode[TRAN_16] = ONLINE_STATE;	
	cpPutMessage(UART1_BASE,0xF018,sizeof(m_ALG_Params),(unsigned char *)&m_ALG_Params);

	return ONLINE_STATE;

}

WORD Disabled_CIC(WORD mode)
{
	m_ALG_Params.ALG_CIC = 0;
	return UPLOAD_G_PARAM;

}

WORD Enabled_CIC(WORD mode)
{
	m_ALG_Params.ALG_CIC = 0x2;
	return UPLOAD_G_PARAM;

}

WORD Disabled_ARC(WORD mode)
{
	m_ARC_Params.Enable = 0x00;
	ctrl.ret_mode[TRAN_ARC] = 0xF020;
	cpPutMessage(UART1_BASE,0xF020,sizeof(m_ARC_Params),(unsigned char *)&m_ARC_Params);

	return ONLINE_STATE;

}

WORD Enabled_ARC(WORD mode)
{
	m_ARC_Params.Enable = 0x01;
	ctrl.ret_mode[TRAN_ARC] = 0xF020;
	
	cpPutMessage(UART1_BASE,0xF020,sizeof(m_ARC_Params),(unsigned char *)&m_ARC_Params);

	return ONLINE_STATE;

}


WORD Shutter_Read_Status(WORD mode)
{
	char arr[9] ={ 0x00, 0xA5, 0x90, 0x20, 0x20, 0x20,0x20, 0x20, 0x68};

	ctrl.ret_mode[TRAN_16] = 0xFD05;
	//ctrl.next_mode = SHUTTER_WRITE_STATUS;
	ctrl.next_mode[TRAN_16] = SHUTTER_WRITE_STATUS;
		
	cpPutMessage(UART1_BASE,0xFD05,sizeof(arr),(unsigned char *)&arr);

	return ONLINE_STATE;

}

WORD Shutter_Write_Status(WORD mode)
{

	WORD n;

	if (cpMessageReady(COM1))
	{
	   n = cpGetMessageCommand(COM1); //tbd
	   if (n==ctrl.ret_mode[TRAN_16])
	   {
	   	 if(*(cpGetMessageData(COM1,1)) == 0x01)
		  mode = ONLINE_STATE;
		 	
	   }
	}

	return mode;

}

WORD Upload_Black_Hot(WORD mode)
{

	//int i=0;
	WORD n;
	unsigned char Polarity = 0x00;

	/*
	 * Switch for state of tge message protocol processing
	 */
	
	switch (ctrl.state[TIME_BH])
	{
	
		  case BH_SENT_MESSAGE: // Locks for a FLAG character

			   memset(&LabelCmdMsg[2].Style_id_Color_Size_BackGro_text[1],0xFF,sizeof(unsigned short));
			   memset(&LabelCmdMsg[2].Style_id_Color_Size_BackGro_text[3],0x10,sizeof(unsigned char));
			   memset(&LabelCmdMsg[2].Style_id_Color_Size_BackGro_text[4],0,sizeof(unsigned short));

			   strcpy(LabelCmdMsg[2].Text,"B-H");
			   LabelCmdMsg[2].Text[strlen(LabelCmdMsg[2].Text)] = 0;									   
			   Set_LABEL_Visibility(OSD_BLACK_WHITE_HOT_ID,1);
			   
			   ctrl.ret_mode[TIME_BH] = 0xF201;			   
			   ctrl.next_mode[TIME_BH] = BH_HIDE_SHOW_LABEL;			   
			   ctrl.exit_mode[TIME_BH] = BH_SENT_MESSAGE;
			   ctrl.state[TIME_BH] = BH_WAIT_FOR_MESSAGE_READY;
		
			   #ifdef VERSION_1_3
			   cpPutMessage(UART1_BASE,0xF021,sizeof(m_DRC_Params),(unsigned char *)&m_DRC_Params);
			   #else	  
			   cpPutMessage(UART1_BASE,0xF201,1,(unsigned char *)&Polarity);
			   #endif
			   ctrl.time[TIME_BH] = g_ulTickCount;		   
		
		  break;	
 
			 case BH_HIDE_SHOW_LABEL: // Locks for a FLAG character
  
				  ctrl.ret_mode[TIME_BH] = 0xF050;			  
				  ctrl.next_mode[TIME_BH] = BH_X_LABEL_POSITION;			  
				  ctrl.exit_mode[TIME_BH] = BH_HIDE_SHOW_LABEL;
				  cpPutMessage(UART1_BASE,0xF050,0x2,&LabelCmdMsg[OSD_BLACK_WHITE_HOT_ID].Label_id_Visibility[0]);			  
				  ctrl.state[TIME_BH] = BH_WAIT_FOR_MESSAGE_READY;  
				  ctrl.time[TIME_BH] = g_ulTickCount;
  
			 break;
  
			 case BH_X_LABEL_POSITION: // Locks for a FLAG character
			 
				  ctrl.ret_mode[TIME_BH] = 0xF051;
				  ctrl.next_mode[TIME_BH] = BH_LEBEL_TEXT;
				  ctrl.exit_mode[TIME_BH] = BH_X_LABEL_POSITION;
				  cpPutMessage(UART1_BASE,0xF051,0x5,&LabelCmdMsg[OSD_BLACK_WHITE_HOT_ID].Position_id_X_Y[0]);			  
				  ctrl.state[TIME_BH] = BH_WAIT_FOR_MESSAGE_READY;
				  ctrl.time[TIME_BH] = g_ulTickCount;
				  
			 
			 break;
  
			 case BH_LEBEL_TEXT: // Locks for a FLAG character 			 
			 
				  ctrl.ret_mode[TIME_BH] = 0xF052;
				  ctrl.next_mode[TIME_BH] = BH_STYLE_FORMAT;
				  ctrl.exit_mode[TIME_BH] = BH_LEBEL_TEXT;
				  cpPutMessage(UART1_BASE,0xF052,strlen(LabelCmdMsg[OSD_BLACK_WHITE_HOT_ID].Text)+2,&LabelCmdMsg[OSD_BLACK_WHITE_HOT_ID].Text_id);
				  ctrl.state[TIME_BH] = BH_WAIT_FOR_MESSAGE_READY;
				  ctrl.time[TIME_BH] = g_ulTickCount;		  
				  
			 
			 break;
  
			 case BH_STYLE_FORMAT: // Locks for a FLAG character
			 
				 ctrl.ret_mode[TIME_BH] = 0xF053;
				 ctrl.next_mode[TIME_BH] = BH_END_LEBEL_CMD;
				 ctrl.exit_mode[TIME_BH] = BH_STYLE_FORMAT;
				 
				 cpPutMessage(UART1_BASE,0xF053,0x7,&LabelCmdMsg[OSD_BLACK_WHITE_HOT_ID].Style_id_Color_Size_BackGro_text[0]);
				 ctrl.state[TIME_BH] = BH_WAIT_FOR_MESSAGE_READY;
				 ctrl.time[TIME_BH] = g_ulTickCount;
				 
			 
			 break;
  
			 case BH_END_LEBEL_CMD: // Locks for a FLAG character
			 
				 ctrl.state[TIME_BH] = BH_HIDE_SHOW_LABEL;	  
				 ctrl.time[TIME_BH] = g_ulTickCount;
				 cpReset(1);  
				 SetDMCflag(DMC_EXE_MODE,DMC_EXE_MODE); 				 
				 mode = ONLINE_STATE;
			 
			 break;
		
		  case BH_WAIT_FOR_MESSAGE_READY:
		  
			  if (cpMessageReady(COM1))
			  {
				  n = cpGetMessageCommand(COM1); //tbd
				  if (n==ctrl.ret_mode[TIME_BH])
				  {
					   ctrl.state[TIME_BH] = ctrl.next_mode[TIME_BH]; 				   
					   ctrl.time[TIME_BH] = g_ulTickCount;					   
					   cpReset(1);
				  }
				  else
				  {
					  ctrl.state[TIME_BH] = BH_SENT_MESSAGE; 					  
					  ctrl.time[TIME_BH] = g_ulTickCount;					  					  
					  cpReset(1);
					  mode = ONLINE_STATE;
				  }
					
			  } 		 
			  else
			  if ((WORD)(g_ulTickCount - ctrl.time[TIME_BH]) >= (WORD)1000) //-- every 0.5 sec  
			  {
					ctrl.state[TIME_BH] = BH_SENT_MESSAGE;	 
					ctrl.time[TIME_BH] = g_ulTickCount;
					cpReset(1);  
					mode = ONLINE_STATE;					
			  }
		  
		  break;
	
	  default:

					  ctrl.state[TIME_BH] = BH_SENT_MESSAGE; 					  
					  ctrl.time[TIME_BH] = g_ulTickCount;					  					  
					  cpReset(1);
					  mode = ONLINE_STATE;
					  
		 break;
	}

	return mode;

}

WORD Upload_White_Hot(WORD mode)
{

	//int i=0;
	WORD n;	
	unsigned char Polarity = 0x01;

	/*
	 * Switch for state of tge message protocol processing
	 */

	switch (ctrl.state[TIME_WH])
	{
	
			  case WH_SENT_MESSAGE: // Locks for a FLAG character

			   memset(&LabelCmdMsg[2].Style_id_Color_Size_BackGro_text[1],0,sizeof(unsigned short));
			   memset(&LabelCmdMsg[2].Style_id_Color_Size_BackGro_text[3],0x10,sizeof(unsigned char));
			   memset(&LabelCmdMsg[2].Style_id_Color_Size_BackGro_text[4],0xFF,sizeof(unsigned short));

			   strcpy(LabelCmdMsg[2].Text,"W-H");
			   LabelCmdMsg[2].Text[strlen(LabelCmdMsg[2].Text)] = 0;									   
			   Set_LABEL_Visibility(OSD_BLACK_WHITE_HOT_ID,1);
			   
			   ctrl.ret_mode[TIME_WH] = 0xF201; 		   
			   ctrl.next_mode[TIME_WH] = WH_HIDE_SHOW_LABEL; 		   
			   ctrl.exit_mode[TIME_WH] = WH_SENT_MESSAGE;
			   ctrl.state[TIME_WH] = WH_WAIT_FOR_MESSAGE_READY;
			
			   #ifdef VERSION_1_3
				   cpPutMessage(UART1_BASE,0xF016,sizeof(m_DRC_Params),(unsigned char *)&m_DRC_Params);
			   #else	  
				   cpPutMessage(UART1_BASE,0xF201,1,(unsigned char *)&Polarity);
			   #endif
			   ctrl.time[TIME_WH] = g_ulTickCount;		   
				   	
			
			  break;
	
			 case WH_HIDE_SHOW_LABEL: // Locks for a FLAG character
  
				  ctrl.ret_mode[TIME_WH] = 0xF050;			  
				  ctrl.next_mode[TIME_WH] = WH_X_LABEL_POSITION;			  
				  ctrl.exit_mode[TIME_WH] = WH_HIDE_SHOW_LABEL;
				  cpPutMessage(UART1_BASE,0xF050,0x2,&LabelCmdMsg[OSD_BLACK_WHITE_HOT_ID].Label_id_Visibility[0]);			  
				  ctrl.state[TIME_WH] = WH_WAIT_FOR_MESSAGE_READY;
				  ctrl.time[TIME_WH] = g_ulTickCount;
  
  
			 break;
  
			 case WH_X_LABEL_POSITION: // Locks for a FLAG character
			 
				  ctrl.ret_mode[TIME_WH] = 0xF051;
				  ctrl.next_mode[TIME_WH] = WH_LEBEL_TEXT;
				  ctrl.exit_mode[TIME_WH] = WH_X_LABEL_POSITION;
				  cpPutMessage(UART1_BASE,0xF051,0x5,&LabelCmdMsg[OSD_BLACK_WHITE_HOT_ID].Position_id_X_Y[0]);			  
				  ctrl.state[TIME_WH] = WH_WAIT_FOR_MESSAGE_READY;
				  ctrl.time[TIME_WH] = g_ulTickCount;
				  
			 
			 break;
  
			 case WH_LEBEL_TEXT: // Locks for a FLAG character 			 
			 
				  ctrl.ret_mode[TIME_WH] = 0xF052;
				  ctrl.next_mode[TIME_WH] = WH_STYLE_FORMAT;
				  ctrl.exit_mode[TIME_WH] = WH_LEBEL_TEXT;
				  cpPutMessage(UART1_BASE,0xF052,strlen(LabelCmdMsg[OSD_BLACK_WHITE_HOT_ID].Text)+2,&LabelCmdMsg[OSD_BLACK_WHITE_HOT_ID].Text_id);
				  ctrl.state[TIME_WH] = WH_WAIT_FOR_MESSAGE_READY;
				  ctrl.time[TIME_WH] = g_ulTickCount;		  
				  
			 
			 break;
  
			 case WH_STYLE_FORMAT: // Locks for a FLAG character
			 
				 ctrl.ret_mode[TIME_WH] = 0xF053;
				 ctrl.next_mode[TIME_WH] = WH_END_LEBEL_CMD;
				 ctrl.exit_mode[TIME_WH] = WH_STYLE_FORMAT;
				 
				 cpPutMessage(UART1_BASE,0xF053,0x7,&LabelCmdMsg[OSD_BLACK_WHITE_HOT_ID].Style_id_Color_Size_BackGro_text[0]);
				 ctrl.state[TIME_WH] = WH_WAIT_FOR_MESSAGE_READY;
				 ctrl.time[TIME_WH] = g_ulTickCount;
				 
			 
			 break;
  
			 case WH_END_LEBEL_CMD: // Locks for a FLAG character
			 
				 ctrl.state[TIME_WH] = WH_HIDE_SHOW_LABEL;	  
				 ctrl.time[TIME_WH] = g_ulTickCount;
				 cpReset(1);  
				 SetDMCflag(DMC_EXE_MODE,DMC_EXE_MODE); 				 
				 mode = ONLINE_STATE;
			 
			 break;	
			
			  case WH_WAIT_FOR_MESSAGE_READY:
			  
				  if (cpMessageReady(COM1))
				  {
					  n = cpGetMessageCommand(COM1); //tbd
					  if (n==ctrl.ret_mode[TIME_WH])
					  {
						   ctrl.state[TIME_WH] = ctrl.next_mode[TIME_WH]; 							   
						   ctrl.time[TIME_WH] = g_ulTickCount;						   						   
						   cpReset(1);
					  }
					  else
					  {
						  ctrl.state[TIME_WH] = WH_SENT_MESSAGE;							  
						  ctrl.time[TIME_WH] = g_ulTickCount;						  
						  cpReset(1);
						  mode = ONLINE_STATE;
					  }
						
				  } 		 
				  else
				  if ((WORD)(g_ulTickCount - ctrl.time[TIME_WH]) >= (WORD)1000) //-- every 0.5 sec  
				  {
						ctrl.state[TIME_WH] = WH_SENT_MESSAGE;						
						ctrl.time[TIME_WH] = g_ulTickCount;
						cpReset(1);  
						mode = ONLINE_STATE;
						
				  }
			  
			  break;
			
	  		default:

						  ctrl.state[TIME_WH] = WH_SENT_MESSAGE;							  
						  ctrl.time[TIME_WH] = g_ulTickCount;						  
						  cpReset(1);
						  mode = ONLINE_STATE;				
			 break;
	}


	return mode;

}


/*
 * FUNCTION:  Freeze_Unfreeze_Video - Main Com Proc message processing loop
 * RETURN: Non-zero of message detected  
 */
WORD Upload_V_Flip_Enable(WORD mode)
{

	WORD n; 
	
	m_Zoom_Flip.byte2 |= 0x10;

	if (cpMessageReady(COM1))
	{
		n = cpGetMessageCommand(COM1); //tbd
		if (n==0xA020)
			mode = ONLINE_STATE;
	}
	else
		cpPutMessage(UART1_BASE,0xA020,sizeof(m_Zoom_Flip),(unsigned char *)&m_Zoom_Flip);

return mode;



}	

/*
 * FUNCTION:  Freeze_Unfreeze_Video - Main Com Proc message processing loop
 * RETURN: Non-zero of message detected  
 */
WORD Upload_V_Flip_Disable(WORD mode)
{

	WORD n; 
	
	m_Zoom_Flip.byte2 &= 0xEF;

	if (cpMessageReady(COM1))
	{
		n = cpGetMessageCommand(COM1); //tbd
		if (n==0xA020)
			mode = ONLINE_STATE;
	}
	else
		cpPutMessage(UART1_BASE,0xA020,sizeof(m_Zoom_Flip),(unsigned char *)&m_Zoom_Flip);

return mode;



}	





/*
 * FUNCTION:  Freeze_Unfreeze_Video - Main Com Proc message processing loop
 * RETURN: Non-zero of message detected  
 */
WORD Upload_H_Flip_Enable(WORD mode)

{

	WORD n; 
	
	m_Zoom_Flip.byte2 |= 0x01;
	
	if (cpMessageReady(COM1))
	{
		n = cpGetMessageCommand(COM1); //tbd
		if (n==0xA020)
			mode = ONLINE_STATE;
	}
	else
		cpPutMessage(UART1_BASE,0xA020,sizeof(m_Zoom_Flip),(unsigned char *)&m_Zoom_Flip);

return mode;

}	

/*
 * FUNCTION:  Freeze_Unfreeze_Video - Main Com Proc message processing loop
 * RETURN: Non-zero of message detected  
 */
WORD Upload_H_Flip_Disable(WORD mode)

{

	WORD n; 
	
	m_Zoom_Flip.byte2 &= 0xFE;
	
	if (cpMessageReady(COM1))
	{
		n = cpGetMessageCommand(COM1); //tbd
		if (n==0xA020)
			mode = ONLINE_STATE;
	}
	else
		cpPutMessage(UART1_BASE,0xA020,sizeof(m_Zoom_Flip),(unsigned char *)&m_Zoom_Flip);

return mode;

}	


/*
 * FUNCTION:  Freeze_Unfreeze_Video - Main Com Proc message processing loop
 * RETURN: Non-zero of message detected  
 */
WORD Open_Close_Shutter(WORD mode)

{

	WORD n; 

     /*
      * Switch for state of tge message protocol processing
      */
      
      switch (ctrl.state[ACT_4])
      {	

		  case SET_TEMPERATURE:

				  ctrl.ret_mode[ACT_4] = 0xFD06;	  
				  ctrl.next_mode[ACT_4] = READ_SHUTTER_STATUS;
				  ctrl.exit_mode[ACT_4] = SET_TEMPERATURE;
				  ctrl.time[ACT_4] = GetSysTickCount();		  			  
				  ctrl.try[ACT_4] = 0;			  
				  cpPutMessage(UART1_BASE,0xFD06,sizeof(set_temperature),(unsigned char *)&set_temperature);
				  
				  ctrl.state[ACT_4] = WAIT_FOR_MESSAGE;
			  
			  break;

		  
		  case READ_SHUTTER_STATUS:

				  ctrl.ret_mode[ACT_4] = 0xFD05;
				  ctrl.next_mode[ACT_4] = WRITE_SHUTTER_STATUS;				  				  
				  ctrl.exit_mode[ACT_4] = READ_SHUTTER_STATUS;
				  ctrl.time[ACT_4] = GetSysTickCount();		
  				  ctrl.try[ACT_4] = 0;
				  cpPutMessage(UART1_BASE,0xFD05,sizeof(status_shutter),(unsigned char *)&status_shutter);
				  
				  ctrl.state[ACT_4] = WAIT_FOR_MESSAGE;
			  
			  break;


		  case WRITE_SHUTTER_STATUS:

				  if(*(cpGetMessageData(COM1,1)) == 0x01)
				  {
					  ctrl.state[ACT_4] = SET_CLOSE_OPEN_SHUTTER;
					  cpReset(COM1);		   
				  }
				  else
					  ctrl.state[ACT_4] = ctrl.exit_mode[ACT_4];
				  
				  break;

		 case SET_CLOSE_OPEN_SHUTTER:
		 
				 ctrl.ret_mode[ACT_4] = 0xFD06;	 
				 ctrl.next_mode[ACT_4] = X_WRITE_SHUTTER_STATUS;
				 ctrl.exit_mode[ACT_4] = SET_CLOSE_OPEN_SHUTTER;
				 ctrl.time[ACT_4] = GetSysTickCount(); 						 
				 ctrl.try[ACT_4] = 0;
				 
				 if(ctrl.action == 0 )
					 cpPutMessage(UART1_BASE,0xFD06,sizeof(close_shutter),(unsigned char *)&close_shutter);
				 else
					 cpPutMessage(UART1_BASE,0xFD06,sizeof(open_shutter),(unsigned char *)&open_shutter);
				 
				 ctrl.state[ACT_4] = WAIT_FOR_MESSAGE;
			 
			 break;
		 
		 
		 case X_WRITE_SHUTTER_STATUS: // 4
		 
				 ctrl.ret_mode[ACT_4] = 0xFD05;
				 ctrl.next_mode[ACT_4] = X_READ_SHUTTER_STATUS;
				 ctrl.exit_mode[ACT_4] = X_WRITE_SHUTTER_STATUS;
				 ctrl.time[ACT_4] = GetSysTickCount(); 	   
				 ctrl.try[ACT_4] = 0;
				 cpPutMessage(UART1_BASE,0xFD05,sizeof(status_shutter),(unsigned char *)&status_shutter);
				 
				 ctrl.state[ACT_4] = WAIT_FOR_MESSAGE;
			 
			 break; 			 
		 
		 
		 case X_READ_SHUTTER_STATUS: // 5

			 if(*(cpGetMessageData(COM1,1)) == 0x01)
			 {
				 ctrl.state[ACT_4] = SET_TEMPERATURE;	
				 ctrl.try[ACT_4] = 0;
				 
				 
				 if(GetDMCflag(DMC_ZOOM_X2))
					 mode = UPDATE_ZOOMX2;
				 else				 
				 if(GetDMCflag(DMC_ZOOM_X4))
					 mode = UPDATE_ZOOMX4;
				 else
					 mode = ONLINE_STATE;

				 cpReset(COM1);				 
			 }			
			 else
			 if ((WORD)(GetSysTickCount() - ctrl.time[ACT_4]) >= (WORD)100) //-- every 3 sec	
			 //if(ctrl.try[ACT_4]++ > 5000)
			 {
				 ctrl.time[ACT_4] = GetSysTickCount(); 
				 ctrl.next_mode[ACT_4] = ctrl.exit_mode[ACT_4];
			 }
			
			 break;			 

			 case WAIT_FOR_MESSAGE: // 6
		 
			 if (cpMessageReady(COM1))
			 {
				 n = cpGetMessageCommand(COM1); //tbd
				 if (n==ctrl.ret_mode[ACT_4])
				 {
					  ctrl.state[ACT_4] = ctrl.next_mode[ACT_4];								  
				 }
			 }			
			 else
			 if ((WORD)(GetSysTickCount() - ctrl.time[ACT_4]) >= (WORD)100) //-- every 3 sec
			 {
			 
				   if(ctrl.try[ACT_4]++ > 3)
				   {
					   ctrl.state[ACT_4] = SET_TEMPERATURE;   
					   ctrl.try[ACT_4] = 0;
					   					   
					   mode = ONLINE_STATE;					   
					   cpReset(COM1);					  
				   }
				   //else
					  //ctrl.state[ACT_4] = ctrl.exit_mode[ACT_4]; 					   
			 }
		 
		 break;		 

		 default:
			break;
	  }
				  

return mode;



}	



/*
 * FUNCTION:  Upload_ZoomX4 - Main Com Proc message processing loop
 * RETURN: Non-zero of message detected  
 */

WORD RefreshNUCDataCommand(WORD mode)
{

	char data_bock = 0x01;
	WORD n;

	switch (ctrl.state[TIME_NUC])
	{
	
			  case REFRESH_NUC_CMD: // Locks for a FLAG character

			   //ctrl.ret_mode[TIME_NUC] = 0xF202; 		   
			   //ctrl.next_mode[TIME_NUC] = WAIT_FOR_NUC_MESSAGE_READY; 		   
			   //ctrl.exit_mode[TIME_NUC] = REFRESH_NUC_CMD;
			   //ctrl.state[TIME_NUC] = WAIT_FOR_NUC_MESSAGE_READY;
			   
			   cpPutMessage(UART1_BASE,0xF202,1,(unsigned char *)&data_bock);
			   ctrl.state[TIME_NUC] = REFRESH_NUC_CMD;
			   mode = ONLINE_STATE;

			  break;
	

			case WAIT_FOR_NUC_MESSAGE_READY:
			
				if (cpMessageReady(COM1))
				{
					n = cpGetMessageCommand(COM1); //tbd
					if (n==ctrl.ret_mode[TIME_NUC])
					{
						 ctrl.state[TIME_NUC] = REFRESH_NUC_CMD; 
				         mode = ONLINE_STATE;
			   
					}
					else
						ctrl.state[TIME_NUC] = REFRESH_NUC_CMD; 
					  
				}		   
				else
				if ((WORD)(GetSysTickCount() - ctrl.time[TIME_NUC]) >= (WORD)50) //-- every 0.5 sec	 
				{
					  ctrl.time[TIME_NUC] = GetSysTickCount();							 
					  ctrl.state[TIME_NUC] = REFRESH_NUC_CMD; 
				}
			
			break;

	  		default:

			  cpReset(COM1);  
			  mode = ONLINE_STATE;
						  
			 break;
	}
		
	return mode;
}



/*
 * FUNCTION:  Upload_ZoomX4 - Main Com Proc message processing loop
 * RETURN: Non-zero of message detected  
 */
WORD Upload_ZoomX2(WORD mode)
{

	WORD n; 
	
	m_Zoom_Flip.byte1 = 0x01; //  Zoom X2
	
	m_Zoom_Flip.byte2 = 0x11; // H-Flip V-Flip enable	
	m_Zoom_Flip.center_x = 200;
	m_Zoom_Flip.center_y = 180;

	switch (ctrl.state[TIME_ZOOM2])
	{
	
			  case Z2_SENT_MESSAGE: // Locks for a FLAG character

			   ctrl.ret_mode[TIME_ZOOM2] = 0xA020; 		   
			   ctrl.next_mode[TIME_ZOOM2] = Z2_HIDE_SHOW_LABEL; 		   
			   ctrl.exit_mode[TIME_ZOOM2] = Z2_SENT_MESSAGE;
			   ctrl.state[TIME_ZOOM2] = Z2_WAIT_FOR_MESSAGE_READY;

			   LabelCmdMsg[OSD_ZOOM_ID].Position_id_X_Y[1]	   = 27;   // X
			   LabelCmdMsg[OSD_ZOOM_ID].Position_id_X_Y[3]	   = 170;  // Y

			   strcpy(LabelCmdMsg[1].Text,"<<");
			   LabelCmdMsg[1].Text[strlen(LabelCmdMsg[1].Text)] = 0;									   
			   Set_LABEL_Visibility(OSD_ZOOM_ID,1);
			
			   cpPutMessage(UART1_BASE,0xA020,sizeof(m_Zoom_Flip),(unsigned char *)&m_Zoom_Flip);
			   ctrl.time[TIME_ZOOM2] = g_ulTickCount;				   	
			
			  break;
	
			 case Z2_HIDE_SHOW_LABEL: // Locks for a FLAG character

  
				  ctrl.ret_mode[TIME_ZOOM2] = 0xF050;			  
				  ctrl.next_mode[TIME_ZOOM2] = Z2_X_LABEL_POSITION;			  
				  ctrl.exit_mode[TIME_ZOOM2] = Z2_HIDE_SHOW_LABEL;
				  cpPutMessage(UART1_BASE,0xF050,0x2,&LabelCmdMsg[OSD_ZOOM_ID].Label_id_Visibility[0]);			  
				  ctrl.state[TIME_ZOOM2] = Z2_WAIT_FOR_MESSAGE_READY;
				  ctrl.time[TIME_ZOOM2] = g_ulTickCount;
  
  
			 break;
  
			 case Z2_X_LABEL_POSITION: // Locks for a FLAG character
			 
				  ctrl.ret_mode[TIME_ZOOM2] = 0xF051;
				  ctrl.next_mode[TIME_ZOOM2] = Z2_LEBEL_TEXT;
				  ctrl.exit_mode[TIME_ZOOM2] = Z2_X_LABEL_POSITION;
				  cpPutMessage(UART1_BASE,0xF051,0x5,&LabelCmdMsg[OSD_ZOOM_ID].Position_id_X_Y[0]);			  
				  ctrl.state[TIME_ZOOM2] = Z2_WAIT_FOR_MESSAGE_READY;
				  ctrl.time[TIME_ZOOM2] = g_ulTickCount;
				  
			 
			 break;
  
			 case Z2_LEBEL_TEXT: // Locks for a FLAG character 			 
			 
				  ctrl.ret_mode[TIME_ZOOM2] = 0xF052;
				  ctrl.next_mode[TIME_ZOOM2] = Z2_STYLE_FORMAT;
				  ctrl.exit_mode[TIME_ZOOM2] = Z2_LEBEL_TEXT;
				  cpPutMessage(UART1_BASE,0xF052,strlen(LabelCmdMsg[OSD_ZOOM_ID].Text)+2,&LabelCmdMsg[OSD_ZOOM_ID].Text_id);
				  ctrl.state[TIME_ZOOM2] = Z2_WAIT_FOR_MESSAGE_READY;
				  ctrl.time[TIME_ZOOM2] = g_ulTickCount;		  
				  
			 
			 break;
  
			 case Z2_STYLE_FORMAT: // Locks for a FLAG character
			 
				 ctrl.ret_mode[TIME_ZOOM2] = 0xF053;
				 ctrl.next_mode[TIME_ZOOM2] = Z2_END_LEBEL_CMD;
				 ctrl.exit_mode[TIME_ZOOM2] = Z2_STYLE_FORMAT;
				 
				 cpPutMessage(UART1_BASE,0xF053,0x7,&LabelCmdMsg[OSD_ZOOM_ID].Style_id_Color_Size_BackGro_text[0]);
				 ctrl.state[TIME_ZOOM2] = Z2_WAIT_FOR_MESSAGE_READY;
				 ctrl.time[TIME_ZOOM2] = g_ulTickCount;
				 
			 
			 break;
  
			 case Z2_END_LEBEL_CMD: // Locks for a FLAG character

				 SetDMCflag(DMC_ZOOM_X2,DMC_ZOOM_X2);	 // signal for control center					 
				 SetDMCflag(DMC_ZOOM_X4,0);
				 SetDMCflag(DMC_DISABLE_ZOOM,0);	 
			 
				 ctrl.state[TIME_ZOOM2] = Z2_SENT_MESSAGE;	  
				 ctrl.time[TIME_ZOOM2] = g_ulTickCount;
				 cpReset(1);  
				 mode = ONLINE_STATE;
			 
			 break;	
			
			  case Z2_WAIT_FOR_MESSAGE_READY:
			  
				  if (cpMessageReady(COM1))
				  {
					  n = cpGetMessageCommand(COM1); //tbd
					  if (n==ctrl.ret_mode[TIME_ZOOM2])	  
					  {					  
						  ctrl.state[TIME_ZOOM2] = ctrl.next_mode[TIME_ZOOM2];				  					  
						  ctrl.try[TIME_ZOOM2] = 0;
						  cpReset(COM1);  
						  ctrl.time[TIME_ZOOM2]   = g_ulTickCount;						  
					  }
					  else
					  {
						  ctrl.state[TIME_ZOOM2] = Z2_SENT_MESSAGE;				  					  
						  ctrl.time[TIME_ZOOM2]   = g_ulTickCount;						  						  
						  ctrl.try[TIME_ZOOM2] = 0;
						  cpReset(COM1);  
						  mode = ONLINE_STATE;
					  }
				  }
				  else
				  if ((WORD)(GetSysTickCount() - ctrl.time[TIME_ZOOM2]) >= (WORD)1000) //-- every 3 sec
				  { 	  
							ctrl.time[TIME_ZOOM2] = g_ulTickCount;
							if(ctrl.try[TIME_ZOOM2]++ > 1)
							{
								ctrl.try[TIME_ZOOM2] = 0;									
							    ctrl.state[TIME_ZOOM2] = Z2_SENT_MESSAGE;							
								mode = ONLINE_STATE;		  
								cpReset(COM1);				   
							}
				  }
				  
				  break;
			
	  		default:

						  ctrl.state[TIME_ZOOM2] = Z2_SENT_MESSAGE;				  					  
						  ctrl.time[TIME_ZOOM2]   = g_ulTickCount;						  						  
						  ctrl.try[TIME_ZOOM2] = 0;
						  cpReset(COM1);  
						  mode = ONLINE_STATE;
						  
			 break;
	}
		

	return mode;
}
	
/*
 * FUNCTION:  Upload_ZoomX2 - Main Com Proc message processing loop
 * RETURN: Non-zero of message detected  
 */
WORD Upload_ZoomX4(WORD mode)
{
	
		WORD n; 
		m_Zoom_Flip.byte1 = 0x11; //  Zoom X4		
		m_Zoom_Flip.byte2 = 0x11; // H-Flip V-Flip enable		
		m_Zoom_Flip.center_x = 200;
		m_Zoom_Flip.center_y = 160;

		switch (ctrl.state[TIME_ZOOM4])
		{
		
				  case Z4_SENT_MESSAGE: // Locks for a FLAG character

				   ctrl.ret_mode[TIME_ZOOM4] = 0xA020; 		   
				   ctrl.next_mode[TIME_ZOOM4] = Z4_HIDE_SHOW_LABEL; 		   
				   ctrl.exit_mode[TIME_ZOOM4] = Z4_SENT_MESSAGE;
				   ctrl.state[TIME_ZOOM4] = Z4_WAIT_FOR_MESSAGE_READY;
				   

			   	   LabelCmdMsg[OSD_ZOOM_ID].Position_id_X_Y[1]	   = 27;   // X
			   	   LabelCmdMsg[OSD_ZOOM_ID].Position_id_X_Y[3]	   = 240;  // Y
			   
				   strcpy(LabelCmdMsg[1].Text,"<<");
				   LabelCmdMsg[1].Text[strlen(LabelCmdMsg[1].Text)] = 0;									   
				   Set_LABEL_Visibility(OSD_ZOOM_ID,1);
				
				   cpPutMessage(UART1_BASE,0xA020,sizeof(m_Zoom_Flip),(unsigned char *)&m_Zoom_Flip);
				   ctrl.time[TIME_ZOOM4] = g_ulTickCount;				   	
				
				  break;
		
				 case Z4_HIDE_SHOW_LABEL: // Locks for a FLAG character

	  
					  ctrl.ret_mode[TIME_ZOOM4] = 0xF050;			  
					  ctrl.next_mode[TIME_ZOOM4] = Z4_X_LABEL_POSITION;			  
					  ctrl.exit_mode[TIME_ZOOM4] = Z4_HIDE_SHOW_LABEL;
					  cpPutMessage(UART1_BASE,0xF050,0x2,&LabelCmdMsg[OSD_ZOOM_ID].Label_id_Visibility[0]);			  
					  ctrl.state[TIME_ZOOM4] = Z4_WAIT_FOR_MESSAGE_READY;
					  ctrl.time[TIME_ZOOM4] = g_ulTickCount;
	  
	  
				 break;
	  
				 case Z4_X_LABEL_POSITION: // Locks for a FLAG character
				 
					  ctrl.ret_mode[TIME_ZOOM4] = 0xF051;
					  ctrl.next_mode[TIME_ZOOM4] = Z4_LEBEL_TEXT;
					  ctrl.exit_mode[TIME_ZOOM4] = Z4_X_LABEL_POSITION;
					  cpPutMessage(UART1_BASE,0xF051,0x5,&LabelCmdMsg[OSD_ZOOM_ID].Position_id_X_Y[0]);			  
					  ctrl.state[TIME_ZOOM4] = Z4_WAIT_FOR_MESSAGE_READY;
					  ctrl.time[TIME_ZOOM4] = g_ulTickCount;
					  
				 
				 break;
	  
				 case Z4_LEBEL_TEXT: // Locks for a FLAG character 			 
				 
					  ctrl.ret_mode[TIME_ZOOM4] = 0xF052;
					  ctrl.next_mode[TIME_ZOOM4] = Z4_STYLE_FORMAT;
					  ctrl.exit_mode[TIME_ZOOM4] = Z4_LEBEL_TEXT;
					  cpPutMessage(UART1_BASE,0xF052,strlen(LabelCmdMsg[OSD_ZOOM_ID].Text)+2,&LabelCmdMsg[OSD_ZOOM_ID].Text_id);
					  ctrl.state[TIME_ZOOM4] = Z4_WAIT_FOR_MESSAGE_READY;
					  ctrl.time[TIME_ZOOM4] = g_ulTickCount;		  
					  
				 
				 break;
	  
				 case Z4_STYLE_FORMAT: // Locks for a FLAG character
				 
					 ctrl.ret_mode[TIME_ZOOM4] = 0xF053;
					 ctrl.next_mode[TIME_ZOOM4] = Z4_END_LEBEL_CMD;
					 ctrl.exit_mode[TIME_ZOOM4] = Z4_STYLE_FORMAT;
					 
					 cpPutMessage(UART1_BASE,0xF053,0x7,&LabelCmdMsg[OSD_ZOOM_ID].Style_id_Color_Size_BackGro_text[0]);
					 ctrl.state[TIME_ZOOM4] = Z4_WAIT_FOR_MESSAGE_READY;
					 ctrl.time[TIME_ZOOM4] = g_ulTickCount;
					 
				 
				 break;
	  
				 case Z4_END_LEBEL_CMD: // Locks for a FLAG character

					 SetDMCflag(DMC_ZOOM_X4,DMC_ZOOM_X4);	 // signal for control center
					 SetDMCflag(DMC_ZOOM_X2,0);
					 SetDMCflag(DMC_DISABLE_ZOOM,0);
				 
					 ctrl.state[TIME_ZOOM4] = Z4_SENT_MESSAGE;	  
					 ctrl.time[TIME_ZOOM4] = g_ulTickCount;
					 cpReset(1);  
					 mode = ONLINE_STATE;
				 
				 break;	
				
				  case Z4_WAIT_FOR_MESSAGE_READY:
				  
					  if (cpMessageReady(COM1))
					  {
						  n = cpGetMessageCommand(COM1); //tbd
						  if (n==ctrl.ret_mode[TIME_ZOOM4])	  
						  {					  
							  ctrl.state[TIME_ZOOM4] = ctrl.next_mode[TIME_ZOOM4];				  					  
							  ctrl.try[TIME_ZOOM4] = 0;
							  cpReset(COM1);  
							  ctrl.time[TIME_ZOOM4]   = g_ulTickCount;						  
						  }
						  else
						  {
							  ctrl.state[TIME_ZOOM4] = Z4_SENT_MESSAGE;				  					  
							  ctrl.try[TIME_ZOOM4] = 0;
							  cpReset(COM1);  
							  ctrl.time[TIME_ZOOM4]   = g_ulTickCount;	
							  mode = ONLINE_STATE;
					  	  }
					  }
					  else
					  if ((WORD)(GetSysTickCount() - ctrl.time[TIME_ZOOM4]) >= (WORD)1000) //-- every 3 sec
					  { 	  
								ctrl.time[TIME_ZOOM4] = g_ulTickCount;
								if(ctrl.try[TIME_ZOOM4]++ > 1)
								{
									ctrl.state[TIME_ZOOM4] = Z4_SENT_MESSAGE;								
									ctrl.try[TIME_ZOOM4] = 0;									
									mode = ONLINE_STATE;		  
									cpReset(COM1);			
									mode = ONLINE_STATE;
								}
					  }
					  
					  break;
				
		  		default:

							  ctrl.state[TIME_ZOOM4] = Z4_SENT_MESSAGE;				  					  
							  ctrl.try[TIME_ZOOM4] = 0;
							  cpReset(COM1);  
							  ctrl.time[TIME_ZOOM4]   = g_ulTickCount;	
							  mode = ONLINE_STATE;					
				 break;
		}
		
	return mode;
}

/*
 * FUNCTION:  Upload_ZoomX2 - Main Com Proc message processing loop
 * RETURN: Non-zero of message detected  
 */
WORD Disable_Zoom(WORD mode)
{
	
		WORD n; 
	
		m_Zoom_Flip.byte1 = 0x0; // Disable Zoom		
		m_Zoom_Flip.byte2 = 0x11; // H-Flip V-Flip enable
		m_Zoom_Flip.center_x = 396;
		m_Zoom_Flip.center_y = 288;


		switch (ctrl.state[TIME_DIS])
		{


				  case DIS_SENT_MESSAGE: // Locks for a FLAG character

				   ctrl.ret_mode[TIME_DIS] = 0xA020; 		   
				   ctrl.next_mode[TIME_DIS] = DIS_HIDE_SHOW_LABEL; 		   
				   ctrl.exit_mode[TIME_DIS] = DIS_SENT_MESSAGE;
				   ctrl.state[TIME_DIS] = DIS_WAIT_FOR_MESSAGE_READY;	
				   

			       LabelCmdMsg[OSD_ZOOM_ID].Position_id_X_Y[1]	   = 27;   // X
			   	   LabelCmdMsg[OSD_ZOOM_ID].Position_id_X_Y[3]	   = 100;  // Y

			       strcpy(LabelCmdMsg[1].Text,"<<");
			       LabelCmdMsg[1].Text[strlen(LabelCmdMsg[1].Text)] = 0;									   
			       Set_LABEL_Visibility(OSD_ZOOM_ID,1);
			   
				   cpPutMessage(UART1_BASE,0xA020,0x6,(unsigned char *)&m_Zoom_Flip);
				   ctrl.time[TIME_DIS] = g_ulTickCount;				   	
				
				  break;

		
				 case DIS_HIDE_SHOW_LABEL: // Locks for a FLAG character

	  
					  ctrl.ret_mode[TIME_DIS] = 0xF050;			  
					  ctrl.next_mode[TIME_DIS] = DIS_X_LABEL_POSITION;			  
					  ctrl.exit_mode[TIME_DIS] = DIS_HIDE_SHOW_LABEL;
					  ctrl.state[TIME_DIS] = DIS_WAIT_FOR_MESSAGE_READY;					  
					  cpPutMessage(UART1_BASE,0xF050,0x2,&LabelCmdMsg[OSD_ZOOM_ID].Label_id_Visibility[0]);			  
					  ctrl.time[TIME_DIS] = g_ulTickCount;
	  
	  
				 break;
	  
				 case DIS_X_LABEL_POSITION: // Locks for a FLAG character
				 
					  ctrl.ret_mode[TIME_DIS] = 0xF051;
					  ctrl.next_mode[TIME_DIS] = DIS_LEBEL_TEXT;
					  ctrl.exit_mode[TIME_DIS] = DIS_X_LABEL_POSITION;
					  ctrl.state[TIME_DIS] = DIS_WAIT_FOR_MESSAGE_READY;					  
					  cpPutMessage(UART1_BASE,0xF051,0x5,&LabelCmdMsg[OSD_ZOOM_ID].Position_id_X_Y[0]);			  
					  ctrl.time[TIME_DIS] = g_ulTickCount;
					  
				 
				 break;
	  
				 case DIS_LEBEL_TEXT: // Locks for a FLAG character 			 
				 
					  ctrl.ret_mode[TIME_DIS] = 0xF052;
					  ctrl.next_mode[TIME_DIS] = DIS_STYLE_FORMAT;
					  ctrl.exit_mode[TIME_DIS] = DIS_LEBEL_TEXT;
					  ctrl.state[TIME_DIS] = DIS_WAIT_FOR_MESSAGE_READY;					  
					  cpPutMessage(UART1_BASE,0xF052,strlen(LabelCmdMsg[OSD_ZOOM_ID].Text)+2,&LabelCmdMsg[OSD_ZOOM_ID].Text_id);
					  ctrl.time[TIME_DIS] = g_ulTickCount;		  
					  
				 
				 break;
	  
				 case DIS_STYLE_FORMAT: // Locks for a FLAG character
				 
					 ctrl.ret_mode[TIME_DIS] = 0xF053;
					 ctrl.next_mode[TIME_DIS] = DIS_END_LEBEL_CMD;
					 ctrl.exit_mode[TIME_DIS] = DIS_STYLE_FORMAT;
					 ctrl.state[TIME_DIS] = DIS_WAIT_FOR_MESSAGE_READY;					 
					 cpPutMessage(UART1_BASE,0xF053,0x7,&LabelCmdMsg[OSD_ZOOM_ID].Style_id_Color_Size_BackGro_text[0]);
					 ctrl.time[TIME_DIS] = g_ulTickCount;
					 
				 
				 break;
	  
				 case DIS_END_LEBEL_CMD: // Locks for a FLAG character

					 SetDMCflag(DMC_DISABLE_ZOOM,DMC_DISABLE_ZOOM);  // signal for control center				 
					 SetDMCflag(DMC_ZOOM_X2,0);
					 SetDMCflag(DMC_ZOOM_X4,0); 					 
					 ctrl.state[TIME_DIS] = DIS_SENT_MESSAGE;	  
					 ctrl.time[TIME_DIS] = g_ulTickCount;
					 cpReset(1);  
					 mode = ONLINE_STATE;
				 
				 break;	
				
				  case DIS_WAIT_FOR_MESSAGE_READY:
				  
					  if (cpMessageReady(COM1))
					  {
						  n = cpGetMessageCommand(COM1); //tbd
						  if (n==ctrl.ret_mode[TIME_DIS])	  
						  {					  
							  ctrl.state[TIME_DIS] = ctrl.next_mode[TIME_DIS];				  					  
							  ctrl.try[TIME_DIS] = 0;
							  cpReset(COM1);  
							  ctrl.time[TIME_DIS]   = g_ulTickCount;						  
						  }
						  else
						  {
							  ctrl.state[TIME_DIS] = DIS_SENT_MESSAGE;				  					  
							  ctrl.try[TIME_DIS] = 0;
							  cpReset(COM1);  
							  ctrl.time[TIME_DIS]   = g_ulTickCount;
							  mode = ONLINE_STATE;
						  }
					  }
					  else
					  if ((WORD)(GetSysTickCount() - ctrl.time[TIME_DIS]) >= (WORD)1000) //-- every 3 sec
					  { 	  
								ctrl.time[TIME_DIS] = g_ulTickCount;
								if(ctrl.try[TIME_DIS]++ > 1)
								{
									ctrl.state[TIME_DIS] = DIS_SENT_MESSAGE;								
									ctrl.try[TIME_DIS] = 0;									
									mode = ONLINE_STATE;		  
									cpReset(COM1);				   
								}
					  }
					  
					  break;
				
		  		default:
					
					ctrl.state[TIME_DIS] = DIS_SENT_MESSAGE;									
					ctrl.try[TIME_DIS] = 0;
					cpReset(COM1);	
					ctrl.time[TIME_DIS]   = g_ulTickCount;
					mode = ONLINE_STATE;
					
				 break;
		}
			
	return mode;
}

		
/*
 * FUNCTION:  Upload_ZoomX2 - Main Com Proc message processing loop
 * RETURN: Non-zero of message detected  
 */
WORD Upload_Zoom_Position(WORD mode)
{
		
			WORD n; 
			m_Zoom_Flip.byte2 = 0x11; // H-Flip V-Flip enable
		
			if (cpMessageReady(COM1))
			{
				n = cpGetMessageCommand(COM1); //tbd
				if (n==0xA020)
				{
					mode = ONLINE_STATE;
				}
			}
			else
				cpPutMessage(UART1_BASE,0xA020,sizeof(m_Zoom_Flip),(unsigned char *)&m_Zoom_Flip);
		
		return mode;
}

/*
 * FUNCTION:  Freeze_Unfreeze_Video - Main Com Proc message processing loop
 * RETURN: Non-zero of message detected  
 */
WORD Freeze_Unfreeze_Video(WORD mode)
{

	WORD n;	

	if (cpMessageReady(COM1))
	{
		n = cpGetMessageCommand(COM1); //tbd
		if (n==0xA040)
			mode = ONLINE_STATE;
	}
	else
	{
		Freeze_Unfreeze = !Freeze_Unfreeze;
		cpPutMessage(UART1_BASE,0xA040,1,(unsigned char *)&Freeze_Unfreeze);
	}

	return mode;

}	

/*
 * FUNCTION:  Freeze_Unfreeze_Video - Main Com Proc message processing loop
 * RETURN: Non-zero of message detected  
 */
WORD Tec_Read_Handler(WORD mode)
{

	WORD n;	
	WORD   len = 0;	
	unsigned char msg[20];
	double	Case_temp;

	
	if (cpMessageReady(COM1))
	{
		n = cpGetMessageCommand(COM1); //tbd
		if (n==0xF00F)
		{
			len = cpGetMessageDataLen(COM1);
			//memcpy(&m_TEC_Params, cpGetMessageData(COM1,0), len);
			memcpy(&msg,cpGetMessageData(COM1,0),len);


			// Calculate case and substrate values from TEC_Read_Buf
			Case_temp = (double)(((int)(msg[3])<<24) + (((int)(msg[4])) << 16) + (((int)(msg[5] << 8))));
			// Performing arithmetic shift right 8 bit while keeping the sign
			Case_temp /= 256.0;			
			m_TEC_Params.Case_tem = ((((((double)Case_temp)*10)/pow(2.0, 24)) -  0.106)/0.000360) + 25.0;
						  
			mode = ONLINE_STATE;			
			cpReset(COM1);		   
		
		}
	}
	else
	{
		cpPutMessage(UART1_BASE,0xF00F,0,NULL);
	}

	return mode;

}	


/*
 * FUNCTION:  Update_2_P - Main Com Proc message processing loop
 * RETURN: Non-zero of message detected  
 */
WORD Update_2_P(WORD mode)
{
   //char ch;
   //int i = 0;	
   WORD n;
   char status_shutter[9] ={ 0x00, 0xA5, 0x90, 0x20, 0x20, 0x20,0x20, 0x20, 0x68};
   char algorithm[10] = {0,0,6,3,5,4,0,0,0,0};
   char algorithm1[10] = {0,0,2,6,3,5,4,0,0,0};
   
     /*
      * Switch for state of tge message protocol processing
      */
      switch (ctrl.state[UPDATE_2_P])
      {
	 
          case DISABLED_CIC_2_P: // Locks for a FLAG character

				  m_ALG_Params.ALG_CIC = 0;					
				  ctrl.ret_mode[UPDATE_2_P] = 0xF018;
				  ctrl.next_mode[UPDATE_2_P] = DISABLED_ARC_2_P;
				  ctrl.exit_mode[UPDATE_2_P] = DISABLED_CIC_2_P;
				  ctrl.time[UPDATE_2_P] = GetSysTickCount();
 				  ctrl.try[UPDATE_2_P] = 0;				
				  //cpPutMessage(UART1_BASE,0xF018,sizeof(ALG_Params),&ALG_Params);
				  cpPutMessage(UART1_BASE,0xF018,sizeof(algorithm),(unsigned char *)&algorithm);
				  
				  ctrl.state[UPDATE_2_P] = WAIT_FOR_MESSAGE_READY;
				
             break;

          case DISABLED_ARC_2_P: // Disabel ARC command

				  m_ARC_Params.Enable = 0x00;
				  ctrl.ret_mode[UPDATE_2_P] = 0xF020;
				  ctrl.next_mode[UPDATE_2_P] = CLOSE_SHUTTER_2_P;
				  ctrl.exit_mode[UPDATE_2_P] = DISABLED_ARC_2_P;
				  ctrl.time[UPDATE_2_P] = GetSysTickCount();				
				  ctrl.try[UPDATE_2_P] = 0;				
				  cpPutMessage(UART1_BASE,0xF020,sizeof(m_ARC_Params),(unsigned char *)&m_ARC_Params);
				  
				  ctrl.state[UPDATE_2_P] = WAIT_FOR_MESSAGE_READY;


             break;

		  case CLOSE_SHUTTER_2_P:

				  ctrl.ret_mode[UPDATE_2_P] = 0xFD06;	  
				  ctrl.next_mode[UPDATE_2_P] = READ_SHUTTER_STATUS_2_P;
				  ctrl.exit_mode[UPDATE_2_P] = CLOSE_SHUTTER_2_P;
				  ctrl.time[UPDATE_2_P] = GetSysTickCount();		  			  
				  ctrl.try[UPDATE_2_P] = 0;			  
				  cpPutMessage(UART1_BASE,0xFD06,sizeof(close_shutter),(unsigned char *)&close_shutter);
				  
				  ctrl.state[UPDATE_2_P] = WAIT_FOR_MESSAGE_READY;
			  
			  break;


		  case READ_SHUTTER_STATUS_2_P:

				  ctrl.ret_mode[UPDATE_2_P] = 0xFD05;
				  ctrl.next_mode[UPDATE_2_P] = WRITE_SHUTTER_STATUS_2_P;
				  ctrl.exit_mode[UPDATE_2_P] = READ_SHUTTER_STATUS_2_P;
				  ctrl.time[UPDATE_2_P] = GetSysTickCount();		
  				  ctrl.try[UPDATE_2_P] = 0;
				  cpPutMessage(UART1_BASE,0xFD05,sizeof(status_shutter),(unsigned char *)&status_shutter);
				  
				  ctrl.state[UPDATE_2_P] = WAIT_FOR_MESSAGE_READY;
			  
			  break;


		  case WRITE_SHUTTER_STATUS_2_P:

				  if(*(cpGetMessageData(COM1,1)) == 0x01)
					  ctrl.state[UPDATE_2_P] = F_AVG_SET_PARAM;
				  break;


		  case F_AVG_SET_PARAM:

				  ctrl.ret_mode[UPDATE_2_P] = 0xF019;
				  ctrl.next_mode[UPDATE_2_P] = POLL_COMPILETION_FRAME_AVERAGING;
				  ctrl.exit_mode[UPDATE_2_P] = F_AVG_SET_PARAM;
				  ctrl.time[UPDATE_2_P] = GetSysTickCount();						  				    
				  ctrl.try[UPDATE_2_P] = 0;
				  cpPutMessage(UART1_BASE,0xF019,sizeof(m_Frm_Avg_Params),(unsigned char *)&m_Frm_Avg_Params);
				  
				  ctrl.state[UPDATE_2_P] = WAIT_FOR_MESSAGE_READY;
			 
			  break;

			  
		  case POLL_COMPILETION_FRAME_AVERAGING:

				  ctrl.ret_mode[UPDATE_2_P] = 0xF022;
				  ctrl.next_mode[UPDATE_2_P] = COMPLETED_FRAME_AVERAGING;
				  ctrl.exit_mode[UPDATE_2_P] = POLL_COMPILETION_FRAME_AVERAGING;
				  ctrl.time[UPDATE_2_P] = GetSysTickCount();						  				    
				  ctrl.try[UPDATE_2_P] = 0;
				  cpPutMessage(UART1_BASE,0xF022,0,NULL);
				  
				  ctrl.state[UPDATE_2_P] = WAIT_FOR_MESSAGE_READY;
				 
			  break;


		  case COMPLETED_FRAME_AVERAGING:


			  if(*(cpGetMessageData(COM1,0)) == 0x00)
				  ctrl.state[UPDATE_2_P] = ENABLED_CIC_2_P;
			  else
				  ctrl.state[UPDATE_2_P] = ctrl.exit_mode[UPDATE_2_P];
			  	
			  break;


          case ENABLED_CIC_2_P: // Locks for a FLAG character

			  	  m_ALG_Params.ALG_CIC = 0x2;
				  
				  ctrl.ret_mode[UPDATE_2_P] = 0xF018;
				  ctrl.next_mode[UPDATE_2_P] = SET_NEW_CIC_REFERENCE;
				  ctrl.exit_mode[UPDATE_2_P] = ENABLED_CIC_2_P;
				  ctrl.time[UPDATE_2_P] = GetSysTickCount();
 				  ctrl.try[UPDATE_2_P] = 0;				
				  //cpPutMessage(UART1_BASE,0xF018,sizeof(ALG_Params),&ALG_Params);
				  cpPutMessage(UART1_BASE,0xF018,sizeof(algorithm1),(unsigned char *)&algorithm1);
				  
				  ctrl.state[UPDATE_2_P] = WAIT_FOR_MESSAGE_READY;
				
             break;


		  case SET_NEW_CIC_REFERENCE:

				  m_CIC_Params.Op_Type = 0x01;

				  ctrl.ret_mode[UPDATE_2_P] = 0xF013;
				  ctrl.next_mode[UPDATE_2_P] = SET_NEW_NUC_REFERENCE;
				  ctrl.exit_mode[UPDATE_2_P] = SET_NEW_CIC_REFERENCE;
				  ctrl.time[UPDATE_2_P] = GetSysTickCount();
				  ctrl.try[UPDATE_2_P] = 0; 			
				  cpPutMessage(UART1_BASE,0xF013,sizeof(m_CIC_Params),(unsigned char *)&m_CIC_Params);
				  
				  ctrl.state[UPDATE_2_P] = WAIT_FOR_MESSAGE_READY;
				 
			  break;


		  case SET_NEW_NUC_REFERENCE:

				  m_NUC_Params.Op_Type = 0x2;

				  ctrl.ret_mode[UPDATE_2_P] = 0xF014;
				  ctrl.next_mode[UPDATE_2_P] = OPEN_SHUTTER_2_P;
				  ctrl.exit_mode[UPDATE_2_P] = SET_NEW_NUC_REFERENCE;
				  ctrl.time[UPDATE_2_P] = GetSysTickCount();
				  ctrl.try[UPDATE_2_P] = 0; 			
				  cpPutMessage(UART1_BASE,0xF014,sizeof(m_NUC_Params),(unsigned char *)&m_NUC_Params);
				  
				  ctrl.state[UPDATE_2_P] = WAIT_FOR_MESSAGE_READY;
				 
			  break;


		  case OPEN_SHUTTER_2_P:

				  ctrl.ret_mode[UPDATE_2_P] = 0xFD06;	  
				  ctrl.next_mode[UPDATE_2_P] = READ_SHUTTER_STATUS_2_P_2;
				  ctrl.exit_mode[UPDATE_2_P] = OPEN_SHUTTER_2_P;
				  ctrl.time[UPDATE_2_P] = GetSysTickCount();			  			  
				  ctrl.try[UPDATE_2_P] = 0;			  
				  cpPutMessage(UART1_BASE,0xFD06,sizeof(open_shutter),(unsigned char *)&open_shutter);
				  
				  ctrl.state[UPDATE_2_P] = WAIT_FOR_MESSAGE_READY;
			  
			  break;


		  case READ_SHUTTER_STATUS_2_P_2:

				  ctrl.ret_mode[UPDATE_2_P] = 0xFD05;
				  ctrl.next_mode[UPDATE_2_P] = ENABLED_ARC_2_P;
				  ctrl.exit_mode[UPDATE_2_P] = READ_SHUTTER_STATUS_2_P_2;
				  ctrl.time[UPDATE_2_P] = GetSysTickCount();		
  				  ctrl.try[UPDATE_2_P] = 0;
				  cpPutMessage(UART1_BASE,0xFD05,sizeof(status_shutter),(unsigned char *)&status_shutter);
				  
				  ctrl.state[UPDATE_2_P] = WAIT_FOR_MESSAGE_READY;
			  
			  break;			  


		  case WRITE_SHUTTER_STATUS_2_P_2:

				  if(*(cpGetMessageData(COM1,1)) == 0x01)
					  ctrl.state[UPDATE_2_P] = ENABLED_ARC_2_P;
				  
			 
			  break;


          case ENABLED_ARC_2_P: // Disabel ARC command

				  m_ARC_Params.Enable = 0x01;
				  ctrl.ret_mode[UPDATE_2_P] = 0xF020;
				  ctrl.next_mode[UPDATE_2_P] = END_UPDATE_2_P;
				  ctrl.exit_mode[UPDATE_2_P] = ENABLED_ARC_2_P;
				  ctrl.time[UPDATE_2_P] = GetSysTickCount();				
				  ctrl.try[UPDATE_2_P] = 0;				
				  cpPutMessage(UART1_BASE,0xF020,sizeof(m_ARC_Params),(unsigned char *)&m_ARC_Params);
				  ctrl.state[UPDATE_2_P] = WAIT_FOR_MESSAGE_READY;


             break;

			  case END_UPDATE_2_P: // Disabel ARC command
			  
					  mode = ONLINE_STATE;
					  ctrl.state[UPDATE_2_P] = DISABLED_CIC_2_P;
					  cpReset(1);
			  		  	
				 break;


			  case WAIT_FOR_MESSAGE_READY:

			  if (cpMessageReady(COM1))
			  {
				  n = cpGetMessageCommand(COM1); //tbd
				  if (n==ctrl.ret_mode[UPDATE_2_P])
				  {
					   ctrl.state[UPDATE_2_P] = ctrl.next_mode[UPDATE_2_P]; 
				  }
				  	
			  }			 
			  else
		  	  if ((WORD)(GetSysTickCount() - ctrl.time[UPDATE_2_P]) >= (WORD)3000) //-- every 3 sec
			  //if(ctrl.try[UPDATE_2_P]++ > 5000)
		  	  {
						cpReset(1);		  	  
						ctrl.state[UPDATE_2_P] = ctrl.exit_mode[UPDATE_2_P]; 
						ctrl.time[UPDATE_2_P] = GetSysTickCount();
		  	  }

		  break;

          default:
             break;
       }
	  
   return mode;
}

  /*
   * FUNCTION:	Set_Label_Position 
   * RETURN: Non-zero of message detected  
   */
  WORD Set_Text_Label_Command(WORD mode)
  {
  
	  //WORD	 id = 0;  
	  WORD n; 

		  /*
		   * Switch for state of tge message protocol processing
		   */
		   switch (ctrl.state[TEXT_LABEL])
		   {
		  
			 case HIDE_SHOW_LABEL: // Locks for a FLAG character
  
				  ctrl.ret_mode[TEXT_LABEL] = 0xF050;			  
				  ctrl.next_mode[TEXT_LABEL] = X_LABEL_POSITION;			  
				  ctrl.exit_mode[TEXT_LABEL] = HIDE_SHOW_LABEL;
				  ctrl.time[TEXT_LABEL] = g_ulTickCount;
				  cpPutMessage(UART1_BASE,0xF050,0x2,&LabelCmdMsg[OSD_MODE_ID].Label_id_Visibility[0]);			  
				  ctrl.state[TEXT_LABEL] = WAIT_FOR_MESSAGE_READY;
  
  
			 break;
  
			 case X_LABEL_POSITION: // Locks for a FLAG character
			 
				  ctrl.ret_mode[TEXT_LABEL] = 0xF051;
				  ctrl.next_mode[TEXT_LABEL] = LEBEL_TEXT;
				  ctrl.exit_mode[TEXT_LABEL] = X_LABEL_POSITION;
				  cpPutMessage(UART1_BASE,0xF051,0x5,&LabelCmdMsg[OSD_MODE_ID].Position_id_X_Y[0]);			  
				  ctrl.state[TEXT_LABEL] = WAIT_FOR_MESSAGE_READY;
				  ctrl.time[TEXT_LABEL] = g_ulTickCount;
				  
			 
			 break;
  
			 case LEBEL_TEXT: // Locks for a FLAG character 			 
			 
				  ctrl.ret_mode[TEXT_LABEL] = 0xF052;
				  ctrl.next_mode[TEXT_LABEL] = STYLE_FORMAT;
				  ctrl.exit_mode[TEXT_LABEL] = LEBEL_TEXT;
				  cpPutMessage(UART1_BASE,0xF052,strlen(LabelCmdMsg[OSD_MODE_ID].Text)+2,&LabelCmdMsg[OSD_MODE_ID].Text_id);
				  ctrl.state[TEXT_LABEL] = WAIT_FOR_MESSAGE_READY;
				  ctrl.time[TEXT_LABEL] = g_ulTickCount;		  
				  
			 
			 break;
  
			 case STYLE_FORMAT: // Locks for a FLAG character
			 
				 ctrl.ret_mode[TEXT_LABEL] = 0xF053;
				 ctrl.next_mode[TEXT_LABEL] = END_LEBEL_CMD;
				 ctrl.exit_mode[TEXT_LABEL] = STYLE_FORMAT;
				 
				 cpPutMessage(UART1_BASE,0xF053,0x7,&LabelCmdMsg[OSD_MODE_ID].Style_id_Color_Size_BackGro_text[0]);
				 ctrl.state[TEXT_LABEL] = WAIT_FOR_MESSAGE_READY;
				 ctrl.time[TEXT_LABEL] = g_ulTickCount;
				 
			 
			 break;
  
			 case END_LEBEL_CMD: // Locks for a FLAG character
			 
				 ctrl.state[TEXT_LABEL] = HIDE_SHOW_LABEL;	  
				 ctrl.time[TEXT_LABEL] = g_ulTickCount;
				 cpReset(1);  
				 SetDMCflag(DMC_EXE_MODE,DMC_EXE_MODE);  
				 mode = ONLINE_STATE;
			 
			 break;
  
			 case WAIT_FOR_MESSAGE_READY:
			 
				 if (cpMessageReady(COM1))
				 {
					 n = cpGetMessageCommand(COM1); //tbd
					 if (n==ctrl.ret_mode[TEXT_LABEL])
					 {
						  ctrl.state[TEXT_LABEL] = ctrl.next_mode[TEXT_LABEL]; 
						  ctrl.time[TEXT_LABEL] = g_ulTickCount;
						  cpReset(1);
					 }
					 else
					 {
						 ctrl.state[TEXT_LABEL] = END_LEBEL_CMD; 
						 cpReset(1);	
						 ctrl.time[TEXT_LABEL] = g_ulTickCount;
						 mode = ONLINE_STATE;
					 }
					   
				 }			
				 else
				 if ((WORD)(g_ulTickCount - ctrl.time[TEXT_LABEL]) >= (WORD)1000) //-- every 0.5 sec  
				 {
					   ctrl.state[TEXT_LABEL] = HIDE_SHOW_LABEL;	
					   ctrl.time[TEXT_LABEL] = g_ulTickCount;
					   cpReset(1);	
					   mode = ONLINE_STATE;
					   
				 }
			 
			 break;
		  
			 default:
			 	
				 ctrl.state[TEXT_LABEL] = END_LEBEL_CMD; 
				 cpReset(1);	
				 ctrl.time[TEXT_LABEL] = g_ulTickCount;
				 mode = ONLINE_STATE;

				break;
		   }
		   	  		  
	  return mode;
  
}   

 
  /*
  * FUNCTION:  Set_Label_Position 
  * RETURN: Non-zero of message detected  
  */
 void Put_Text_Label(BYTE id,BYTE Visib)
 {
 
	 WORD n; 
	 int endofloop=1;
		 
	 while(endofloop != 0)
	 {
		 /*
		  * Switch for state of tge message protocol processing
		  */
		  switch (ctrl.state[TEXT_LABEL1])
		  {
		 
 
			case LEBEL_TEXT: // Locks for a FLAG character				
			
				 ctrl.ret_mode[TEXT_LABEL1] = 0xF052;
				 ctrl.next_mode[TEXT_LABEL1] = END_LEBEL_CMD;
				 ctrl.exit_mode[TEXT_LABEL1] = LEBEL_TEXT;
				 cpPutMessage(UART1_BASE,0xF052,strlen(LabelCmdMsg[id].Text)+2,&LabelCmdMsg[id].Text_id);
				 ctrl.state[TEXT_LABEL1] = WAIT_FOR_MESSAGE_READY;
				 ctrl.time[TEXT_LABEL1] = GetSysTickCount();		 
				 
			
			break;
 
			case END_LEBEL_CMD: // Locks for a FLAG character
			
				ctrl.state[TEXT_LABEL1] = HIDE_SHOW_LABEL;	 
				ctrl.time[TEXT_LABEL1] = GetSysTickCount();
				cpReset(1);  
				ctrl.try[TEXT_LABEL1] = 0;
				endofloop = 0;
			
			break;
 
			case WAIT_FOR_MESSAGE_READY:
			
				if (cpMessageReady(COM1))
				{
					n = cpGetMessageCommand(COM1); //tbd
					if (n==ctrl.ret_mode[TEXT_LABEL1])
						 ctrl.state[TEXT_LABEL1] = ctrl.next_mode[TEXT_LABEL1]; 
					else
						ctrl.state[TEXT_LABEL1] = END_LEBEL_CMD; 
					  
				}		   
				else
				if ((WORD)(GetSysTickCount() - ctrl.time[TEXT_LABEL1]) >= (WORD)50) //-- every 0.5 sec	 
				{
					  ctrl.time[TEXT_LABEL1] = GetSysTickCount();							 
					  ctrl.state[TEXT_LABEL1] = END_LEBEL_CMD; 
				}
			
			break;
		 
			default:
				endofloop = 0;
			   break;
		  }
		 }
	 
 }	 


 /*
 * FUNCTION:  Set_Label_Position 
 * RETURN: Non-zero of message detected  
 */
int Set_Text_Label(BYTE id,BYTE Visib)
{

	WORD n;	
	int endofloop=1;
		
	while(endofloop != 0)
	{
		/*
		 * Switch for state of tge message protocol processing
		 */
		 switch (ctrl.state[TEXT_LABEL1])
		 {
		
		   case HIDE_SHOW_LABEL: // Locks for a FLAG character

				ctrl.ret_mode[TEXT_LABEL1] = 0xF050;				
				ctrl.next_mode[TEXT_LABEL1] = X_LABEL_POSITION;				
				ctrl.exit_mode[TEXT_LABEL1] = HIDE_SHOW_LABEL;
				ctrl.time[TEXT_LABEL1] = GetSysTickCount();
				cpPutMessage(UART1_BASE,0xF050,0x2,&LabelCmdMsg[id].Label_id_Visibility[0]);			
				ctrl.state[TEXT_LABEL1] = WAIT_FOR_MESSAGE_READY;


		   break;

		   case X_LABEL_POSITION: // Locks for a FLAG character
		   
		   		ctrl.ret_mode[TEXT_LABEL1] = 0xF051;
				ctrl.next_mode[TEXT_LABEL1] = LEBEL_TEXT;
				ctrl.exit_mode[TEXT_LABEL1] = X_LABEL_POSITION;
				cpPutMessage(UART1_BASE,0xF051,0x5,&LabelCmdMsg[id].Position_id_X_Y[0]);			
				ctrl.state[TEXT_LABEL1] = WAIT_FOR_MESSAGE_READY;
				ctrl.time[TEXT_LABEL1] = GetSysTickCount();
				
		   
		   break;

		   case LEBEL_TEXT: // Locks for a FLAG character	  		   
		   
		   		ctrl.ret_mode[TEXT_LABEL1] = 0xF052;
				ctrl.next_mode[TEXT_LABEL1] = STYLE_FORMAT;
				ctrl.exit_mode[TEXT_LABEL1] = LEBEL_TEXT;
				cpPutMessage(UART1_BASE,0xF052,strlen(LabelCmdMsg[id].Text)+2,&LabelCmdMsg[id].Text_id);
				ctrl.state[TEXT_LABEL1] = WAIT_FOR_MESSAGE_READY;
				ctrl.time[TEXT_LABEL1] = GetSysTickCount();			
				
		   
		   break;

		   case STYLE_FORMAT: // Locks for a FLAG character
		   
			   ctrl.ret_mode[TEXT_LABEL1] = 0xF053;
			   ctrl.next_mode[TEXT_LABEL1] = END_LEBEL_CMD;
			   ctrl.exit_mode[TEXT_LABEL1] = STYLE_FORMAT;
			   
			   cpPutMessage(UART1_BASE,0xF053,0x7,&LabelCmdMsg[id].Style_id_Color_Size_BackGro_text[0]);
			   ctrl.state[TEXT_LABEL1] = WAIT_FOR_MESSAGE_READY;
			   ctrl.time[TEXT_LABEL1] = GetSysTickCount();
			   
		   
		   break;

		   case END_LEBEL_CMD: // Locks for a FLAG character
		   
			   ctrl.state[TEXT_LABEL1] = HIDE_SHOW_LABEL;	
			   ctrl.time[TEXT_LABEL1] = GetSysTickCount();
			   cpReset(1);	
			   ctrl.try[TEXT_LABEL1] = 0;
			   endofloop = 0;
			   return 1;
		   
		   break;

		   case WAIT_FOR_MESSAGE_READY:
		   
			   if (cpMessageReady(COM1))
			   {
				   n = cpGetMessageCommand(COM1); //tbd
				   if (n==ctrl.ret_mode[TEXT_LABEL1])
						ctrl.state[TEXT_LABEL1] = ctrl.next_mode[TEXT_LABEL1]; 
				   else
					   ctrl.state[TEXT_LABEL1] = END_LEBEL_CMD; 
					 
			   }		  
			   else
			   if ((WORD)(GetSysTickCount() - ctrl.time[TEXT_LABEL1]) >= (WORD)50) //-- every 0.5 sec	
			   {
				   ctrl.state[TEXT_LABEL1] = HIDE_SHOW_LABEL;	
				   ctrl.time[TEXT_LABEL1] = GetSysTickCount();
				   cpReset(1);	
				   ctrl.try[TEXT_LABEL1] = 0;
				   endofloop = 0; 
				   return 0;
			   }
		   
		   break;
		
		   default:
			  break;
		 }
		}
	
}	
 
void Set_LABEL_Visibility(BYTE idx,int Visib)
{
	LabelCmdMsg[idx].Label_id_Visibility[1]	=Visib;		
}


WORD On_Off_Stabilize(WORD mode)
{
	
	cpPutMessage(UART1_BASE,0xF032,1,&Stabilize_status);
	
	return ONLINE_STATE;

}

