
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
#include "driverlib/qei.h"
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
#include "filter3.h"
#include "Stabilize.h"
#include "encoder.h"


/* Portable bit-set and bit-clear macros. */
#define BIT_SET(sfr,bitmask) sfr |= (bitmask)
#define BIT_CLR(sfr,bitmask) sfr &=~ (bitmask)

struct sTm {
  union birdbit {
    BYTE raw;
    struct birdS {
    unsigned char bit0          : 1;
    unsigned char bit1          : 1;
    unsigned char bit2          : 1;
    unsigned char bit3          : 1;
    unsigned char bit4          : 1;
    unsigned char bit5          : 1;
    unsigned char bit6          : 1;
    unsigned char bit7          : 1;	  	  
    } birdS;
  } birdbit;
} BirdStatus;


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
unsigned short DMCmode = BIT_MODE;

char status_shutter[9] ={ 0x00, 0xA5, 0x90, 0x20, 0x20, 0x20,0x20, 0x20, 0x68};
char set_temperature[8] = { 0xA4, 0x90, 0x2C, 0x10, 0x17, 0x10,  0x00, 0x50};		
char close_shutter[8] = { 0xA4, 0x90, 0x17, 0x10, 0x00, 0x10,  0x00, 0x50}; 		
char open_shutter[8] =	{ 0xA4, 0x90, 0x17, 0x10, 0x01, 0x10,  0x00, 0x50}; 

unsigned long lasttm;

unsigned char g_ucid_lebel = 0;
unsigned char g_ucvisible_lebel = 0;
static BYTE Freeze_Unfreeze = 0x0;
signed short servo27 = 0;
signed short servo28 = 0;

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
extern char g_curentmode;
extern float  vertical,horizontal;
extern float e_rate_roll,u_rate_roll;

Frm_Avg_Params	m_Frm_Avg_Params;
NUC_Params 		m_NUC_Params;
ARC_Params 		m_ARC_Params;
ALG_Params 		m_ALG_Params;
CIC_Params 		m_CIC_Params;
DRC_Params 		m_DRC_Params;
Gain_Params		m_Gain_Params;
Zoom_Flip 		m_Zoom_Flip;
TEC_Params 		m_TEC_Params;
Cont_Brit_Fiels	m_Contrast_Set;





unsigned char message[2][COM_PROC_BUF_LEN]; // receiver message buffer
unsigned char BbTxBuffer[TX_MSG_SIZE];
LabelCmd LabelCmdMsg[MAX_LABEL_LENGTH];
static long UserTelemetryData[20] = {0};
char input_data_com1[100];

typedef WORD (* ModeType)(WORD);

const ModeType ModeVec[]  =	
{
    NullMode,    				// 				 		0x00
	UploadEngineVersion,		// DOWNLOAD_VERSION 		0x01       
    OnlineState,		  		// ONLINE_STATE (stop)		0x02  
    WhiteBlackHot,      		//  BLACK_HOT           		0x03   
    NullMode,      				//  			          		0x04
    UploadGeneralParameters,	// UPLOAD_G_PARAM    	 	0x05	
    NullMode,					// 			         		0x06 
    OpenCloseShutter,			// OPEN_CLODE_SHUTTER		0x07
    ShutterReadStatus,			// SHUTTER_READ_STATUS 	0x08
    ShutterWriteStatus,			// SHUTTER_WRITE_STATUS 	0x09
    Enabled_ARC,          		// ENABLED_ARC 			0x0A   
    Disabled_ARC,          		// DISABLED_ARC    		0x0B
    Enabled_CIC,          		// ENABLED_CIC  			0x0C
    Disabled_CIC,     			// DISABLED_CIC			0x0D
    Update2P,					// UPDATE2_P				0x0E   
    NullMode,     				//                        			0x0F  
    OfflineState,	  			// OFFLINE_STATE (stop)		0x10
    FreezeUnfreezeVideo,		// FREEZE_UNFREEZE		0x11
    UploadVFlipEnable,			// UPDATE_V_FLIP			0x12
    UploadVFlipDisable,			// UPDATE_V_FLIP_D		0x13    
    UploadHFlipEnable,			// UPDATE_H_FLIP			0x14
    UploadHFlipDisable,			// UPDATE_H_FLIP_D		0x15  
    NullMode,					// 						0x16
    NullMode,					// 						0x17 
    ZoomFlippingCommand,		// UPDATE_DISABLE_ZOOM	0x18
    UploadZoomPosition,			// UPDATE_ZOOM_FLIP		0x19    
    Tec_Read_Handler,			// UPDATE_ZOOM_FLIP		0x1A    
	NullMode,					// 						0x1B
	NullMode,					// 						0x1C
	NullMode,					// 						0x1D	
	NullMode,					// 						0x1E
	SetTextLabelCommand,		// SET_LABEL_COMMAND		0x1F
	OnOffStabilize,				// ON_OFF_STABILIZE		0x20
	RefreshNUCDataCommand,		// REFRESH_NUC			0x21
	GraphicsOnOff,				// GRAPHICS_ON_OFF		0x22
	SetContrastBrithness,		// SET_CONTRAST_BRITHNESS  0x23	
	UpdateGainLevel,			// UPDATE_GAIN_LEVEL		0x24		
	SetBitMode,					// BIT_MODE				0x25
	BirdStatusResponse,			// BIRD_STATUS_RESPONSE	0x26	
    GetEmbeddedUIInitStatus		// GET_EMBEDDED_INIT_STATUS			0x27
};

#define MODE_NUM (sizeof(ModeVec) / sizeof(ModeType))


void init_ctrl(void)
{

  memset(&BirdStatus,0,sizeof(BirdStatus));

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
  ctrl.time[CON_BRIT] = g_ulTickCount;	
  ctrl.time[UPDATE_GAIN] = g_ulTickCount;	  

  

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
  ctrl.state[CON_BRIT] = SET_CONTRAST;	 
  ctrl.state[UPDATE_GAIN] = SET_GAIN;	
  ctrl.state[TIME_BIT] = SET_ENCODER;
  ctrl.state[TIME_BIRD_RES] = BIRD_STATUS;
  ctrl.state[ACT_11] = SEND_DOWNLOAD_MESSAGE;  
  
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

void Init_DRC_Gain_Params(void)
{

	m_Gain_Params.gain_minimum_input = 0;	//	2 Byte
	m_Gain_Params.gain_maximum_input = 0x3FFF; //	2 Byte
	m_Gain_Params.gain_minimum_output = 0x0; //	1 Byte
	m_Gain_Params.gain_maximum_output= 0xFF;	//	1 Byte

}

void Init_m_Zoom_Flip(void)
{

	/* see definitions above */
	m_Zoom_Flip.byte1 = 0x0;
	m_Zoom_Flip.byte2 = 0x10;
	m_Zoom_Flip.columns = IMAGE_HIGHT;
	m_Zoom_Flip.rows = IMAGE_WIDTH;
	m_Zoom_Flip.center_x = 320;
	m_Zoom_Flip.center_y = 200;

	m_Contrast_Set.brightness = 10;
	m_Contrast_Set.contrast = 50;	
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

void Init_LABEL_Roll_Pitch(BYTE Visibility)
{
	
		unsigned int l_uiXposition = 0;
		unsigned int l_uiYposition = 0;
		
		memset(&LabelCmdMsg,0,sizeof(LabelCmdMsg));

		LabelCmdMsg[OSD_MODE_ID].Label_id_Visibility[0]		= OSD_MODE_ID; 	// 0
		LabelCmdMsg[OSD_MODE_ID].Label_id_Visibility[1]		= Visibility;

		l_uiXposition = 160;
		l_uiYposition = 30;
		LabelCmdMsg[OSD_MODE_ID].Position_id_X_Y[0]		= OSD_MODE_ID;
		memcpy(&LabelCmdMsg[OSD_MODE_ID].Position_id_X_Y[1],(unsigned char *)&l_uiXposition,sizeof(unsigned int));
		memcpy(&LabelCmdMsg[OSD_MODE_ID].Position_id_X_Y[3],(unsigned char *)&l_uiYposition,sizeof(unsigned int));	

		LabelCmdMsg[OSD_MODE_ID].Text_id			= OSD_MODE_ID;
		strcpy(LabelCmdMsg[OSD_MODE_ID].Text,"RATE");
		LabelCmdMsg[OSD_MODE_ID].Text[strlen(LabelCmdMsg[OSD_MODE_ID].Text)] = 0;	
		
		LabelCmdMsg[OSD_MODE_ID].Style_id_Color_Size_BackGro_text[0] = OSD_MODE_ID;
		memset(&LabelCmdMsg[OSD_MODE_ID].Style_id_Color_Size_BackGro_text[1],0xFF,sizeof(unsigned short));		
		memset(&LabelCmdMsg[OSD_MODE_ID].Style_id_Color_Size_BackGro_text[3],10,sizeof(unsigned char));		
		memset(&LabelCmdMsg[OSD_MODE_ID].Style_id_Color_Size_BackGro_text[4],0,sizeof(unsigned short));		
		memset(&LabelCmdMsg[OSD_MODE_ID].Style_id_Color_Size_BackGro_text[6],1,sizeof(unsigned char));
		
		/*-------------------------------------------------------------------------------------------------*/
	

		LabelCmdMsg[OSD_ZOOM_ID].Label_id_Visibility[0]		= OSD_ZOOM_ID; 	// 0
		LabelCmdMsg[OSD_ZOOM_ID].Label_id_Visibility[1]		= Visibility;

		l_uiXposition = 30;
		l_uiYposition = 90;
		LabelCmdMsg[OSD_ZOOM_ID].Position_id_X_Y[0]		= OSD_ZOOM_ID;
		memcpy(&LabelCmdMsg[OSD_ZOOM_ID].Position_id_X_Y[1],(unsigned char *)&l_uiXposition,sizeof(unsigned int));
		memcpy(&LabelCmdMsg[OSD_ZOOM_ID].Position_id_X_Y[3],(unsigned char *)&l_uiYposition,sizeof(unsigned int));	

		LabelCmdMsg[OSD_ZOOM_ID].Text_id			= OSD_ZOOM_ID;
		strcpy(LabelCmdMsg[OSD_ZOOM_ID].Text,"ZxO");
		LabelCmdMsg[OSD_ZOOM_ID].Text[strlen(LabelCmdMsg[OSD_ZOOM_ID].Text)] = 0;	
		
		LabelCmdMsg[OSD_ZOOM_ID].Style_id_Color_Size_BackGro_text[0] = OSD_ZOOM_ID;
		memset(&LabelCmdMsg[OSD_ZOOM_ID].Style_id_Color_Size_BackGro_text[1],0xFF,sizeof(unsigned short));		
		memset(&LabelCmdMsg[OSD_ZOOM_ID].Style_id_Color_Size_BackGro_text[3],10,sizeof(unsigned char));		
		memset(&LabelCmdMsg[OSD_ZOOM_ID].Style_id_Color_Size_BackGro_text[4],0,sizeof(unsigned short));		
		memset(&LabelCmdMsg[OSD_ZOOM_ID].Style_id_Color_Size_BackGro_text[6],1,sizeof(unsigned char));


		/*-------------------------------------------------------------------------------------------------*/


		LabelCmdMsg[OSD_BH_WH_ID].Label_id_Visibility[0]		= OSD_BH_WH_ID; 	// 0
		LabelCmdMsg[OSD_BH_WH_ID].Label_id_Visibility[1]		= Visibility;

		l_uiXposition = 100;
		l_uiYposition = 30;
		LabelCmdMsg[OSD_BH_WH_ID].Position_id_X_Y[0]		= OSD_BH_WH_ID;
		memcpy(&LabelCmdMsg[OSD_BH_WH_ID].Position_id_X_Y[1],(unsigned char *)&l_uiXposition,sizeof(unsigned int));
		memcpy(&LabelCmdMsg[OSD_BH_WH_ID].Position_id_X_Y[3],(unsigned char *)&l_uiYposition,sizeof(unsigned int));	

		LabelCmdMsg[OSD_BH_WH_ID].Text_id			= OSD_BH_WH_ID;
		strcpy(LabelCmdMsg[OSD_BH_WH_ID].Text,"BW");
		LabelCmdMsg[OSD_BH_WH_ID].Text[strlen(LabelCmdMsg[OSD_BH_WH_ID].Text)] = 0;	
		
		LabelCmdMsg[OSD_BH_WH_ID].Style_id_Color_Size_BackGro_text[0] = OSD_BH_WH_ID;
		memset(&LabelCmdMsg[OSD_BH_WH_ID].Style_id_Color_Size_BackGro_text[1],0xFF,sizeof(unsigned short));		
		memset(&LabelCmdMsg[OSD_BH_WH_ID].Style_id_Color_Size_BackGro_text[3],10,sizeof(unsigned char));		
		memset(&LabelCmdMsg[OSD_BH_WH_ID].Style_id_Color_Size_BackGro_text[4],0,sizeof(unsigned short));		
		memset(&LabelCmdMsg[OSD_BH_WH_ID].Style_id_Color_Size_BackGro_text[6],1,sizeof(unsigned char));


		/*-------------------------------------------------------------------------------------------------*/


		LabelCmdMsg[OSD_VERSION_ID].Label_id_Visibility[0]		= OSD_VERSION_ID; 	// 0
		LabelCmdMsg[OSD_VERSION_ID].Label_id_Visibility[1]		= Visibility;

		l_uiXposition = 40;
		l_uiYposition = 220;
		LabelCmdMsg[OSD_VERSION_ID].Position_id_X_Y[0]		= OSD_VERSION_ID;
		memcpy(&LabelCmdMsg[OSD_VERSION_ID].Position_id_X_Y[1],(unsigned char *)&l_uiXposition,sizeof(unsigned int));
		memcpy(&LabelCmdMsg[OSD_VERSION_ID].Position_id_X_Y[3],(unsigned char *)&l_uiYposition,sizeof(unsigned int));	

		LabelCmdMsg[OSD_VERSION_ID].Text_id			= OSD_VERSION_ID;
		sprintf(LabelCmdMsg[OSD_VERSION_ID].Text,"%s",BBVersion); 	//	9					
		LabelCmdMsg[OSD_VERSION_ID].Text[strlen(LabelCmdMsg[OSD_VERSION_ID].Text)] = 0;	
		
		LabelCmdMsg[OSD_VERSION_ID].Style_id_Color_Size_BackGro_text[0] = OSD_VERSION_ID;
		memset(&LabelCmdMsg[OSD_VERSION_ID].Style_id_Color_Size_BackGro_text[1],0xFF,sizeof(unsigned short));		
		memset(&LabelCmdMsg[OSD_VERSION_ID].Style_id_Color_Size_BackGro_text[3],10,sizeof(unsigned char));		
		memset(&LabelCmdMsg[OSD_VERSION_ID].Style_id_Color_Size_BackGro_text[4],0,sizeof(unsigned short));		
		memset(&LabelCmdMsg[OSD_VERSION_ID].Style_id_Color_Size_BackGro_text[6],1,sizeof(unsigned char));


		/*-------------------------------------------------------------------------------------------------*/


		LabelCmdMsg[OSD_ROLL_LEB].Label_id_Visibility[0]		= OSD_ROLL_LEB; 	// 0
		LabelCmdMsg[OSD_ROLL_LEB].Label_id_Visibility[1]		= Visibility;

		l_uiXposition = 40;
		l_uiYposition = 240;
		LabelCmdMsg[OSD_ROLL_LEB].Position_id_X_Y[0]		= OSD_ROLL_LEB;
		memcpy(&LabelCmdMsg[OSD_ROLL_LEB].Position_id_X_Y[1],(unsigned char *)&l_uiXposition,sizeof(unsigned int));
		memcpy(&LabelCmdMsg[OSD_ROLL_LEB].Position_id_X_Y[3],(unsigned char *)&l_uiYposition,sizeof(unsigned int));	

		LabelCmdMsg[OSD_ROLL_LEB].Text_id			= OSD_ROLL_LEB;
		strcpy(LabelCmdMsg[OSD_ROLL_LEB].Text,"Roll:");
		LabelCmdMsg[OSD_ROLL_LEB].Text[strlen(LabelCmdMsg[OSD_ROLL_LEB].Text)] = 0;	
		
		LabelCmdMsg[OSD_ROLL_LEB].Style_id_Color_Size_BackGro_text[0] = OSD_ROLL_LEB;
		memset(&LabelCmdMsg[OSD_ROLL_LEB].Style_id_Color_Size_BackGro_text[1],0xFF,sizeof(unsigned short));		
		memset(&LabelCmdMsg[OSD_ROLL_LEB].Style_id_Color_Size_BackGro_text[3],10,sizeof(unsigned char));		
		memset(&LabelCmdMsg[OSD_ROLL_LEB].Style_id_Color_Size_BackGro_text[4],0,sizeof(unsigned short));		
		memset(&LabelCmdMsg[OSD_ROLL_LEB].Style_id_Color_Size_BackGro_text[6],1,sizeof(unsigned char));


		/*-------------------------------------------------------------------------------------------------*/


		LabelCmdMsg[OSD_PITCH_LEB].Label_id_Visibility[0]		= OSD_PITCH_LEB; 	// 0
		LabelCmdMsg[OSD_PITCH_LEB].Label_id_Visibility[1]		= Visibility;

		l_uiXposition = 40;
		l_uiYposition = 250;
		LabelCmdMsg[OSD_PITCH_LEB].Position_id_X_Y[0]		= OSD_PITCH_LEB;
		memcpy(&LabelCmdMsg[OSD_PITCH_LEB].Position_id_X_Y[1],(unsigned char *)&l_uiXposition,sizeof(unsigned int));
		memcpy(&LabelCmdMsg[OSD_PITCH_LEB].Position_id_X_Y[3],(unsigned char *)&l_uiYposition,sizeof(unsigned int));	

		LabelCmdMsg[OSD_PITCH_LEB].Text_id			= OSD_PITCH_LEB;
		strcpy(LabelCmdMsg[OSD_PITCH_LEB].Text,"Pitch:");
		LabelCmdMsg[OSD_PITCH_LEB].Text[strlen(LabelCmdMsg[OSD_PITCH_LEB].Text)] = 0;	
		
		LabelCmdMsg[OSD_PITCH_LEB].Style_id_Color_Size_BackGro_text[0] = OSD_PITCH_LEB;
		memset(&LabelCmdMsg[OSD_PITCH_LEB].Style_id_Color_Size_BackGro_text[1],0xFF,sizeof(unsigned short));		
		memset(&LabelCmdMsg[OSD_PITCH_LEB].Style_id_Color_Size_BackGro_text[3],10,sizeof(unsigned char));		
		memset(&LabelCmdMsg[OSD_PITCH_LEB].Style_id_Color_Size_BackGro_text[4],0,sizeof(unsigned short));		
		memset(&LabelCmdMsg[OSD_PITCH_LEB].Style_id_Color_Size_BackGro_text[6],1,sizeof(unsigned char));

		/*-------------------------------------------------------------------------------------------------*/



		LabelCmdMsg[OSD_ROLL_POS].Label_id_Visibility[0]		= OSD_ROLL_POS; 	// 0
		LabelCmdMsg[OSD_ROLL_POS].Label_id_Visibility[1]		= Visibility;

		l_uiXposition = 70;
		l_uiYposition = 240;
		LabelCmdMsg[OSD_ROLL_POS].Position_id_X_Y[0]		= OSD_ROLL_POS;
		memcpy(&LabelCmdMsg[OSD_ROLL_POS].Position_id_X_Y[1],(unsigned char *)&l_uiXposition,sizeof(unsigned int));
		memcpy(&LabelCmdMsg[OSD_ROLL_POS].Position_id_X_Y[3],(unsigned char *)&l_uiYposition,sizeof(unsigned int));	

		LabelCmdMsg[OSD_ROLL_POS].Text_id			= OSD_ROLL_POS;
		strcpy(LabelCmdMsg[OSD_ROLL_POS].Text,"1.1");
		LabelCmdMsg[OSD_ROLL_POS].Text[strlen(LabelCmdMsg[OSD_ROLL_POS].Text)] = 0;	
		
		LabelCmdMsg[OSD_ROLL_POS].Style_id_Color_Size_BackGro_text[0] = OSD_ROLL_POS;
		memset(&LabelCmdMsg[OSD_ROLL_POS].Style_id_Color_Size_BackGro_text[1],0xFF,sizeof(unsigned short));		
		memset(&LabelCmdMsg[OSD_ROLL_POS].Style_id_Color_Size_BackGro_text[3],10,sizeof(unsigned char));		
		memset(&LabelCmdMsg[OSD_ROLL_POS].Style_id_Color_Size_BackGro_text[4],0,sizeof(unsigned short));		
		memset(&LabelCmdMsg[OSD_ROLL_POS].Style_id_Color_Size_BackGro_text[6],1,sizeof(unsigned char));

		/*-------------------------------------------------------------------------------------------------*/


		LabelCmdMsg[OSD_PITCH_POS].Label_id_Visibility[0]		= OSD_PITCH_POS; 	// 0
		LabelCmdMsg[OSD_PITCH_POS].Label_id_Visibility[1]		= Visibility;

		l_uiXposition = 70;
		l_uiYposition = 250;
		LabelCmdMsg[OSD_PITCH_POS].Position_id_X_Y[0]		= OSD_PITCH_POS;
		memcpy(&LabelCmdMsg[OSD_PITCH_POS].Position_id_X_Y[1],(unsigned char *)&l_uiXposition,sizeof(unsigned int));
		memcpy(&LabelCmdMsg[OSD_PITCH_POS].Position_id_X_Y[3],(unsigned char *)&l_uiYposition,sizeof(unsigned int));	

		LabelCmdMsg[OSD_PITCH_POS].Text_id			= OSD_PITCH_POS;
		strcpy(LabelCmdMsg[OSD_PITCH_POS].Text,"2.2");
		LabelCmdMsg[OSD_PITCH_POS].Text[strlen(LabelCmdMsg[OSD_PITCH_POS].Text)] = 0;	
		
		LabelCmdMsg[OSD_PITCH_POS].Style_id_Color_Size_BackGro_text[0] = OSD_PITCH_POS;
		memset(&LabelCmdMsg[OSD_PITCH_POS].Style_id_Color_Size_BackGro_text[1],0xFF,sizeof(unsigned short));		
		memset(&LabelCmdMsg[OSD_PITCH_POS].Style_id_Color_Size_BackGro_text[3],10,sizeof(unsigned char));		
		memset(&LabelCmdMsg[OSD_PITCH_POS].Style_id_Color_Size_BackGro_text[4],0,sizeof(unsigned short));		
		memset(&LabelCmdMsg[OSD_PITCH_POS].Style_id_Color_Size_BackGro_text[6],1,sizeof(unsigned char));


		/*-------------------------------------------------------------------------------------------------*/
		

		LabelCmdMsg[OSD_AGC_ON_OFF].Label_id_Visibility[0]		= OSD_AGC_ON_OFF; 	// 0
		LabelCmdMsg[OSD_AGC_ON_OFF].Label_id_Visibility[1]		= Visibility;

		l_uiXposition = 30;
		l_uiYposition = 30;
		LabelCmdMsg[OSD_AGC_ON_OFF].Position_id_X_Y[0]		= OSD_AGC_ON_OFF;
		memcpy(&LabelCmdMsg[OSD_AGC_ON_OFF].Position_id_X_Y[1],(unsigned char *)&l_uiXposition,sizeof(unsigned int));
		memcpy(&LabelCmdMsg[OSD_AGC_ON_OFF].Position_id_X_Y[3],(unsigned char *)&l_uiYposition,sizeof(unsigned int));	

		LabelCmdMsg[OSD_AGC_ON_OFF].Text_id			= OSD_AGC_ON_OFF;
		strcpy(LabelCmdMsg[OSD_AGC_ON_OFF].Text,"AGC-ON");
		LabelCmdMsg[OSD_AGC_ON_OFF].Text[strlen(LabelCmdMsg[OSD_AGC_ON_OFF].Text)] = 0;	
		

		LabelCmdMsg[OSD_AGC_ON_OFF].Style_id_Color_Size_BackGro_text[0] = OSD_AGC_ON_OFF;
		memset(&LabelCmdMsg[OSD_AGC_ON_OFF].Style_id_Color_Size_BackGro_text[1],0xFF,sizeof(unsigned short));		
		memset(&LabelCmdMsg[OSD_AGC_ON_OFF].Style_id_Color_Size_BackGro_text[3],10,sizeof(unsigned char));		
		memset(&LabelCmdMsg[OSD_AGC_ON_OFF].Style_id_Color_Size_BackGro_text[4],0,sizeof(unsigned short));		
		memset(&LabelCmdMsg[OSD_AGC_ON_OFF].Style_id_Color_Size_BackGro_text[6],1,sizeof(unsigned char));	


		/*-------------------------------------------------------------------------------------------------*/
		#if 1
		LabelCmdMsg[OSD_ELEVET].Label_id_Visibility[0]		= OSD_ELEVET; 	// 0
		LabelCmdMsg[OSD_ELEVET].Label_id_Visibility[1]		= Visibility;

		l_uiXposition = 30;
		l_uiYposition = 50;
		LabelCmdMsg[OSD_ELEVET].Position_id_X_Y[0]		= OSD_ELEVET;
		memcpy(&LabelCmdMsg[OSD_ELEVET].Position_id_X_Y[1],(unsigned char *)&l_uiXposition,sizeof(unsigned int));
		memcpy(&LabelCmdMsg[OSD_ELEVET].Position_id_X_Y[3],(unsigned char *)&l_uiYposition,sizeof(unsigned int));	

		LabelCmdMsg[OSD_ELEVET].Text_id			= OSD_ELEVET;
		strcpy(LabelCmdMsg[OSD_ELEVET].Text,"T");
		LabelCmdMsg[OSD_ELEVET].Text[strlen(LabelCmdMsg[OSD_ELEVET].Text)] = 0;	
		
		LabelCmdMsg[OSD_ELEVET].Style_id_Color_Size_BackGro_text[0] = OSD_ELEVET;
		//memset(&LabelCmdMsg[OSD_ELEVET].Style_id_Color_Size_BackGro_text[1],0xBB,sizeof(unsigned short));	
		l_uiXposition = 0xFFFF;
		memcpy(&LabelCmdMsg[OSD_ELEVET].Style_id_Color_Size_BackGro_text[1],(unsigned char *)&l_uiXposition,sizeof(unsigned int));		
		memset(&LabelCmdMsg[OSD_ELEVET].Style_id_Color_Size_BackGro_text[3],16,sizeof(unsigned char));		
		//memset(&LabelCmdMsg[OSD_ELEVET].Style_id_Color_Size_BackGro_text[4],0xCC,sizeof(unsigned short));	
		l_uiXposition = 0xF800;
		memcpy(&LabelCmdMsg[OSD_ELEVET].Style_id_Color_Size_BackGro_text[4],(unsigned char *)&l_uiXposition,sizeof(unsigned int));			
		memset(&LabelCmdMsg[OSD_ELEVET].Style_id_Color_Size_BackGro_text[6],1,sizeof(unsigned char));
		#endif

	
}

void InitParams(void)
{

	SetDMCflag(0xFFFF, 0);	
	SetDMCflag(DMC_DISABLE_ZOOM,DMC_DISABLE_ZOOM); 	
	SetDMCflag(DMC_STABILIZE_OFF,DMC_STABILIZE_OFF);
	SetDMCflag(DMC_BLACK_HOT,DMC_BLACK_HOT); 
		
	cpReset(0);
	cpReset(1);
	init_ctrl();
	Init_m_Zoom_Flip();
	Init_LABEL_Roll_Pitch(1);
 	Init_DRC_Gain_Params();
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
				  //BBLedToggle();
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


int mpGetUserTelemetry(char *pData)
{

	int i = 2;
	int idx = 3;
	int checksum;
	int ret = -1;

	if((pData[0] == 0xFF) && (pData[1] == 0x00) && (pData[2] == 0x04))
	{			
		while(pData[i] != 0xFE){
			i++;
			if(pData[i] == 0xFD)
			{
				i++;
				if(pData[i] == 0x00)		
					pData[idx++] = 0xFD;
				else
				if(pData[i] == 0x01)							
					pData[idx++] = 0xFE;
				else
				if(pData[i] == 0x02)							
					pData[idx++] = 0xFF;
				}
				else
					pData[idx++] = pData[i];									
			}
					
			checksum = 0;
			for (i=2;i<(idx-2);i++)
				checksum+=pData[i];
			checksum  = checksum % 256;
				
			if(checksum == pData[idx-2])
			{
				if(pData[2] == 0x04){
					servo27 = pData[35];
					servo27 |= pData[36]<<8;	
				
					servo28 = pData[41];
					servo28 |= pData[42]<<8;
				}
			}
			else
			{
				servo27 = -1;
				servo28 = -1;
				return -1;
			}
	}
	
return 1;	
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
 * FUNCTION: cpGetMessageData - Returns received message data 
 */ 
unsigned char * cpGetPayLoadMode(unsigned char cid,unsigned short mgs_data)
{
   return (message[cid] + PAYLOAD_MODE); // ne virno !!!
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

	  float tmpPhic=0,tmpThetac=0;
	  static char l_cstatus = 0;
	  static unsigned char l_ucgraphics_status = 0;
	  	
	  #if 1
	  if ((g_ulTickCount - ctrl.time[ONLINE]) >= 300) //-- every 1 sec
	  {	  
					if(GetDMCflag(DMC_ENGINE_CONNECT))
					{
						if(g_curentmode != l_cstatus)
						{
							l_cstatus = g_curentmode;
							if ( l_cstatus == OBSERVATION_MODE) 
							{
								strcpy(LabelCmdMsg[OSD_MODE_ID].Text,"RATE");
								LabelCmdMsg[OSD_MODE_ID].Text[strlen(LabelCmdMsg[OSD_MODE_ID].Text)] = 0;

								ctrl.state[TEXT_LABEL1] = LEBEL_TEXT;	 
								
								Set_LABEL_Visibility(OSD_MODE_ID,1);
								Put_Text_Label(OSD_MODE_ID,1);
								cpReset(1);
								mode = ONLINE_STATE;
							}
							else if ( l_cstatus == HOLD_COORDINATE_MODE) 
							{
								strcpy(LabelCmdMsg[OSD_MODE_ID].Text,"HOLD");
								LabelCmdMsg[OSD_MODE_ID].Text[strlen(LabelCmdMsg[OSD_MODE_ID].Text)] = 0;	

								ctrl.state[TEXT_LABEL1] = LEBEL_TEXT;	 
								
								Set_LABEL_Visibility(OSD_MODE_ID,1);
								Put_Text_Label(OSD_MODE_ID,1);
								cpReset(1);
								mode = ONLINE_STATE;


							}
							else if ( l_cstatus == POINT_COORDINATE_MODE)
							{
								strcpy(LabelCmdMsg[OSD_MODE_ID].Text,"PILOT");
								LabelCmdMsg[OSD_MODE_ID].Text[strlen(LabelCmdMsg[OSD_MODE_ID].Text)] = 0;

								ctrl.state[TEXT_LABEL1] = LEBEL_TEXT;	 
								
								Set_LABEL_Visibility(OSD_MODE_ID,1);
								Put_Text_Label(OSD_MODE_ID,1);
								cpReset(1);
								mode = ONLINE_STATE;


							}
							else if ( l_cstatus == PILOT_WINDOW_MODE)
							{
								strcpy(LabelCmdMsg[OSD_MODE_ID].Text,"PILOT");
								LabelCmdMsg[OSD_MODE_ID].Text[strlen(LabelCmdMsg[OSD_MODE_ID].Text)] = 0;

								ctrl.state[TEXT_LABEL1] = LEBEL_TEXT;	 
								
								Set_LABEL_Visibility(OSD_MODE_ID,1);
								Put_Text_Label(OSD_MODE_ID,1);
								cpReset(1);
								mode = ONLINE_STATE;

							}
							else if ( l_cstatus == STOW_MODE)
							{

								strcpy(LabelCmdMsg[OSD_MODE_ID].Text,"SAFE");
								LabelCmdMsg[OSD_MODE_ID].Text[strlen(LabelCmdMsg[OSD_MODE_ID].Text)] = 0;

								ctrl.state[TEXT_LABEL1] = LEBEL_TEXT;	 
								
								Set_LABEL_Visibility(OSD_MODE_ID,1);
								Put_Text_Label(OSD_MODE_ID,1);
								cpReset(1);
								mode = ONLINE_STATE;

							}
							else if ( l_cstatus == CENTER_MODE)
							{
								strcpy(LabelCmdMsg[OSD_MODE_ID].Text,"CENTER");
								LabelCmdMsg[OSD_MODE_ID].Text[strlen(LabelCmdMsg[OSD_MODE_ID].Text)] = 0;	
								
								ctrl.state[TEXT_LABEL1] = LEBEL_TEXT;	 

								Set_LABEL_Visibility(OSD_MODE_ID,1);
								Put_Text_Label(OSD_MODE_ID,1);
								cpReset(1);
								mode = ONLINE_STATE;
								

							}		
							else if ( l_cstatus == BIT_MODE_CON) 
							{
								strcpy(LabelCmdMsg[OSD_MODE_ID].Text,"BIT");
								LabelCmdMsg[OSD_MODE_ID].Text[strlen(LabelCmdMsg[OSD_MODE_ID].Text)] = 0;	
								
								ctrl.state[TEXT_LABEL1] = LEBEL_TEXT;	 

								Set_LABEL_Visibility(OSD_MODE_ID,1);
								Put_Text_Label(OSD_MODE_ID,1);
								cpReset(1);
								mode = BIT_MODE;
							}									
				

											
						}
						else
						{
							//cAz+=0.01;
							//tmpPhic = (Phic * RAD2DEG);	  
							//tmpThetac= (Thetac* RAD2DEG); 	  
							
							
							//sprintf(LabelCmdMsg[OSD_ROLL_POS].Text,"%d",servo27);	   //  9	Roll
							sprintf(LabelCmdMsg[OSD_ROLL_POS].Text,"%.1f",Motor[ROLL_MOTOR].aangle);	   //  9	Roll
							//sprintf(LabelCmdMsg[OSD_ROLL_POS].Text,"%.1f",u_rate_roll);	   //  9	Roll
							
							//sprintf(LabelCmdMsg[OSD_PITCH_POS].Text,"%d",servo28);	   //  9	Roll
							sprintf(LabelCmdMsg[OSD_PITCH_POS].Text,"%.1f",Motor[PITCH_MOTOR].aangle);	   //  9	Pitch	
						
							
							//sprintf(LabelCmdMsg[OSD_VERSION_ID].Text,"Axm:%.2f  Aym:%.2f Azm:%.2f g:%.2f",Axm,Aym,Azm,g);    //  9									
							LabelCmdMsg[OSD_PITCH_POS].Text[strlen(LabelCmdMsg[OSD_PITCH_POS].Text)] = 0; 
							LabelCmdMsg[OSD_ROLL_POS].Text[strlen(LabelCmdMsg[OSD_ROLL_POS].Text)] = 0;


							ctrl.state[TEXT_LABEL1] = LEBEL_TEXT;					  
							Put_Text_Label(OSD_PITCH_POS,1);
							
							ctrl.state[TEXT_LABEL1] = LEBEL_TEXT;					  
							Put_Text_Label(OSD_ROLL_POS,1);
							mode = BIRD_STATUS_RESPONSE;

						}

					}
					ctrl.time[ONLINE] = g_ulTickCount;	
					//BBLedToggle1();


	  }	  
	  else
	  {
	  	if(g_ucvisible_lebel != l_ucgraphics_status)
		//if(GetDMCflag(DMC_OSD_ON) != GetDMCflag(DMC_OSD_ON))
	  	{
			l_ucgraphics_status = g_ucvisible_lebel;
			mode = GRAPHICS_ON_OFF;
	  	}
		
	  }
	  #endif

				
   return mode;
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


WORD DownloadEngineVersion(WORD mode)
{	

	return UPLOAD_VERSION;

}


WORD UploadEngineVersion(WORD mode)
{

	WORD n;
	WORD   id = 0;
	WORD i = 4;
	WORD ret = 0;

	/*
	 * Switch for state of tge message protocol processing_
	 */
	
	switch (ctrl.state[ACT_11])
	{
	
		  case SEND_DOWNLOAD_MESSAGE: // Locks for a FLAG character
		  

				ctrl.ret_mode[ACT_11] = 0xF011;
				cpPutMessage(UART1_BASE,0xF011,0,NULL);
				ctrl.state[ACT_11] = WAIT_DOWNLOAD_VERSION;
				
		  break;

		  case WAIT_DOWNLOAD_VERSION: // Locks for a FLAG character

		  
				if (cpMessageReady(COM1))
				{
					n = cpGetMessageCommand(COM1); //tbd
					if (n==ctrl.ret_mode[ACT_11])
					{			
				
						cpReset(COM1);		
						ctrl.try[ACT_11] = 0;		

						strcpy(LabelCmdMsg[OSD_VERSION_ID].Text,"Elev");
						LabelCmdMsg[OSD_VERSION_ID].Text[strlen(LabelCmdMsg[OSD_VERSION_ID].Text)] = 0;


						if(GetDMCflag(DMC_BLACK_HOT))
						{
							strcpy(LabelCmdMsg[OSD_BH_WH_ID].Text,"BH");
							LabelCmdMsg[OSD_BH_WH_ID].Text[strlen(LabelCmdMsg[OSD_BH_WH_ID].Text)] = 0;										
						
						
						}
						else
						{
							strcpy(LabelCmdMsg[OSD_BH_WH_ID].Text,"WH");
							LabelCmdMsg[OSD_BH_WH_ID].Text[strlen(LabelCmdMsg[OSD_BH_WH_ID].Text)] = 0;										
							
						}

						strcpy(LabelCmdMsg[OSD_MODE_ID].Text,"BIT");
						LabelCmdMsg[OSD_MODE_ID].Text[strlen(LabelCmdMsg[OSD_MODE_ID].Text)] = 0;										
						
						
						cpReset(COM1);
						ctrl.state[TEXT_LABEL] = HIDE_SHOW_LABEL;					
						ctrl.time[TEXT_LABEL] = g_ulTickCount;			
						mode = SET_LABEL_COMMAND;
					
					}
					else
					{			
						ctrl.time[ACT_11] = g_ulTickCount;
						ctrl.state[ACT_11] = SEND_DOWNLOAD_MESSAGE;
						ctrl.try[ACT_11] = 0;									
						cpReset(COM1);				   

					}  
				}
				else
				if ((WORD)(GetSysTickCount() - ctrl.time[ACT_11]) >= (WORD)3000) //-- every 3 sec
				{		
					  ctrl.time[ACT_11]	= g_ulTickCount;		  
					  ctrl.state[ACT_11] = SEND_DOWNLOAD_MESSAGE;								  
					  cpReset(COM1);				 		  
				}	
				
			break;

	  		default:

					  ctrl.state[ACT_11] = SEND_DOWNLOAD_MESSAGE; 					  
					  ctrl.time[ACT_11] = g_ulTickCount;					  					  
					  cpReset(1);
					  					  
		 	break;
	}			
	
	return mode;


}


WORD UploadGeneralParameters(WORD mode)
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


WORD ShutterReadStatus(WORD mode)
{
	char arr[9] ={ 0x00, 0xA5, 0x90, 0x20, 0x20, 0x20,0x20, 0x20, 0x68};

	ctrl.ret_mode[TRAN_16] = 0xFD05;
	//ctrl.next_mode = SHUTTER_WRITE_STATUS;
	ctrl.next_mode[TRAN_16] = SHUTTER_WRITE_STATUS;
		
	cpPutMessage(UART1_BASE,0xFD05,sizeof(arr),(unsigned char *)&arr);

	return ONLINE_STATE;

}

WORD ShutterWriteStatus(WORD mode)
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

WORD WhiteBlackHot(WORD mode)
{

	//int i=0;
	WORD n;
	unsigned char Polarity = 0x00;

	/*
	 * Switch for state of tge message protocol processing_
	 */
	
	switch (ctrl.state[TIME_BH])
	{
	
		  case BH_SENT_MESSAGE: // Locks for a FLAG character


			   
			   ctrl.ret_mode[TIME_BH] = 0xF201;			   
			   ctrl.next_mode[TIME_BH] = BH_HIDE_SHOW_LABEL;			   
			   ctrl.exit_mode[TIME_BH] = BH_SENT_MESSAGE;
			   ctrl.state[TIME_BH] = BH_WAIT_FOR_MESSAGE_READY;
		
			   #ifdef VERSION_1_3
			   cpPutMessage(UART1_BASE,0xF021,sizeof(m_DRC_Params),(unsigned char *)&m_DRC_Params);
			   #else	  
			   if(GetDMCflag(DMC_WHITE_HOT))
			   	Polarity = 0x00;
			   else
			   	Polarity = 0x01;
			   cpPutMessage(UART1_BASE,0xF201,1,(unsigned char *)&Polarity);
			   #endif
			   ctrl.time[TIME_BH] = g_ulTickCount;		   
		
		  break;	

			 
			 case BH_HIDE_SHOW_LABEL: // Locks for a FLAG character
  
				  ctrl.ret_mode[TIME_BH] = 0xF050;			  
				  ctrl.next_mode[TIME_BH] = BH_X_LABEL_POSITION;			  
				  ctrl.exit_mode[TIME_BH] = BH_HIDE_SHOW_LABEL;
				  cpPutMessage(UART1_BASE,0xF050,0x2,&LabelCmdMsg[OSD_BH_WH_ID].Label_id_Visibility[0]);			  
				  ctrl.state[TIME_BH] = BH_WAIT_FOR_MESSAGE_READY;  
				  ctrl.time[TIME_BH] = g_ulTickCount;
  
			 break;
  
			 case BH_X_LABEL_POSITION: // Locks for a FLAG character
			 
				  ctrl.ret_mode[TIME_BH] = 0xF051;
				  ctrl.next_mode[TIME_BH] = BH_LEBEL_TEXT;
				  ctrl.exit_mode[TIME_BH] = BH_X_LABEL_POSITION;
				  cpPutMessage(UART1_BASE,0xF051,0x5,&LabelCmdMsg[OSD_BH_WH_ID].Position_id_X_Y[0]);			  
				  ctrl.state[TIME_BH] = BH_WAIT_FOR_MESSAGE_READY;
				  ctrl.time[TIME_BH] = g_ulTickCount;
				  
			 
			 break;
  
			 case BH_LEBEL_TEXT: // Locks for a FLAG character 			 

			 
				  ctrl.ret_mode[TIME_BH] = 0xF052;
				  ctrl.next_mode[TIME_BH] = BH_STYLE_FORMAT;
				  ctrl.exit_mode[TIME_BH] = BH_LEBEL_TEXT;

				  if(GetDMCflag(DMC_BLACK_HOT))
				  {
				      	strcpy(LabelCmdMsg[2].Text,"BH");
				      	LabelCmdMsg[2].Text[strlen(LabelCmdMsg[2].Text)] = 0;									   
				  }
				  else
				  {
					 	strcpy(LabelCmdMsg[2].Text,"WH");
					 	LabelCmdMsg[2].Text[strlen(LabelCmdMsg[2].Text)] = 0;										 
				  }
					
				  cpPutMessage(UART1_BASE,0xF052,strlen(LabelCmdMsg[2].Text)+2,&LabelCmdMsg[2].Text_id);
				  ctrl.state[TIME_BH] = BH_WAIT_FOR_MESSAGE_READY;
				  ctrl.time[TIME_BH] = g_ulTickCount;		  
				  
			 
			 break;
  
			 case BH_STYLE_FORMAT: // Locks for a FLAG character


			 
				 ctrl.ret_mode[TIME_BH] = 0xF053;
				 ctrl.next_mode[TIME_BH] = BH_END_LEBEL_CMD;
				 ctrl.exit_mode[TIME_BH] = BH_STYLE_FORMAT;

				 if(GetDMCflag(DMC_BLACK_HOT))
				 {
	 			   	memset(&LabelCmdMsg[OSD_BH_WH_ID].Style_id_Color_Size_BackGro_text[1],0xFF,sizeof(unsigned short));
				  	memset(&LabelCmdMsg[OSD_BH_WH_ID].Style_id_Color_Size_BackGro_text[4],0,sizeof(unsigned short));
				  
				 }
				 else
				 {
				 	memset(&LabelCmdMsg[OSD_BH_WH_ID].Style_id_Color_Size_BackGro_text[1],0,sizeof(unsigned short));
				 	memset(&LabelCmdMsg[OSD_BH_WH_ID].Style_id_Color_Size_BackGro_text[4],0xFF,sizeof(unsigned short));
					 
				 }
				
				 cpPutMessage(UART1_BASE,0xF053,0x7,&LabelCmdMsg[OSD_BH_WH_ID].Style_id_Color_Size_BackGro_text[0]);
				 ctrl.state[TIME_BH] = BH_WAIT_FOR_MESSAGE_READY;
				 ctrl.time[TIME_BH] = g_ulTickCount;
				 
			 
			 break;
  
			 case BH_END_LEBEL_CMD: // Locks for a FLAG character
			 
				 ctrl.state[TIME_BH] = BH_HIDE_SHOW_LABEL;	  
				 ctrl.time[TIME_BH] = g_ulTickCount;
				 cpReset(1);  
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


/*
 * FUNCTION:  FreezeUnfreezeVideo - Main Com Proc message processing loop
 * RETURN: Non-zero of message detected  
 */
WORD UploadVFlipEnable(WORD mode)
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
 * FUNCTION:  FreezeUnfreezeVideo - Main Com Proc message processing loop
 * RETURN: Non-zero of message detected  
 */
WORD UploadVFlipDisable(WORD mode)
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
 * FUNCTION:  FreezeUnfreezeVideo - Main Com Proc message processing loop
 * RETURN: Non-zero of message detected  
 */
WORD UploadHFlipEnable(WORD mode)

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
 * FUNCTION:  FreezeUnfreezeVideo - Main Com Proc message processing loop
 * RETURN: Non-zero of message detected  
 */
WORD UploadHFlipDisable(WORD mode)

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
 * FUNCTION:  FreezeUnfreezeVideo - Main Com Proc message processing loop
 * RETURN: Non-zero of message detected  
 */
WORD OpenCloseShutter(WORD mode)

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
 * FUNCTION:  Upload_ZoomX2 - Main Com Proc message processing loop
 * RETURN: Non-zero of message detected  
 */
WORD ZoomFlippingCommand(WORD mode)
{
	
		WORD n; 

		unsigned int l_uiXposition = 30;
		unsigned int l_uiYposition = 0;
		
		switch (ctrl.state[TIME_DIS])
		{
		
				  case DIS_SENT_MESSAGE: // Locks for a FLAG character

				   ctrl.ret_mode[TIME_DIS] = 0xA020; 		   
				   ctrl.next_mode[TIME_DIS] = DIS_HIDE_SHOW_LABEL; 		   
				   ctrl.exit_mode[TIME_DIS] = DIS_SENT_MESSAGE;
				   ctrl.state[TIME_DIS] = DIS_WAIT_FOR_MESSAGE_READY;	
	
				   if(GetDMCflag(DMC_ZOOM_X2))
				   {
					   m_Zoom_Flip.byte1 = 0x01; //  Zoom X2
					   m_Zoom_Flip.byte2 = 0x11; // H-Flip V-Flip enable   
					   #ifdef BIRD_384
					   m_Zoom_Flip.center_x = 200;
					   m_Zoom_Flip.center_y = 188;
					   #else
					   m_Zoom_Flip.center_x = 300;
					   m_Zoom_Flip.center_y = 250;
					   #endif
					   cpPutMessage(UART1_BASE,0xA020,sizeof(m_Zoom_Flip),(unsigned char *)&m_Zoom_Flip);

				   }
				   else if(GetDMCflag(DMC_ZOOM_X4))
				   {
					   m_Zoom_Flip.byte1 = 0x11; //  Zoom X4	   
					   m_Zoom_Flip.byte2 = 0x11; // H-Flip V-Flip enable	   
					   #ifdef BIRD_384
					   m_Zoom_Flip.center_x = 300;
					   m_Zoom_Flip.center_y = 250;
					   #else
					   m_Zoom_Flip.center_x = 300;
					   m_Zoom_Flip.center_y = 250;
					   #endif
					   cpPutMessage(UART1_BASE,0xA020,sizeof(m_Zoom_Flip),(unsigned char *)&m_Zoom_Flip);
				   }
				   else
				   {
					   m_Zoom_Flip.byte1 = 0x00; // Disable Zoom	   
					   m_Zoom_Flip.byte2 = 0x11; // H-Flip V-Flip enable  
	   				   cpPutMessage(UART1_BASE,0xA020,0x6,(unsigned char *)&m_Zoom_Flip);
				   }
				   

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

					  //if(GetDMCflag(DMC_ZOOM_X2))
    					//l_uiYposition = 200;									 
					  //else if(GetDMCflag(DMC_ZOOM_X4))
    					//l_uiYposition = 300;									   
					  //else if(GetDMCflag(DMC_DISABLE_ZOOM))
    					l_uiYposition = 90; 									   
	 
    				  LabelCmdMsg[OSD_ZOOM_ID].Position_id_X_Y[0]		= OSD_ZOOM_ID;
					  memcpy(&LabelCmdMsg[OSD_ZOOM_ID].Position_id_X_Y[1],(unsigned char *)&l_uiXposition,sizeof(unsigned int));
     				  memcpy(&LabelCmdMsg[OSD_ZOOM_ID].Position_id_X_Y[3],(unsigned char *)&l_uiYposition,sizeof(unsigned int));

	
		
					  ctrl.ret_mode[TIME_DIS] = 0xF051;
					  ctrl.next_mode[TIME_DIS] = DIS_LEBEL_TEXT;
					  ctrl.exit_mode[TIME_DIS] = DIS_X_LABEL_POSITION;
					  ctrl.state[TIME_DIS] = DIS_WAIT_FOR_MESSAGE_READY;					  
					  cpPutMessage(UART1_BASE,0xF051,0x5,&LabelCmdMsg[OSD_ZOOM_ID].Position_id_X_Y[0]);			  
					  ctrl.time[TIME_DIS] = g_ulTickCount;
					  
				 
				 break;
	  
				 case DIS_LEBEL_TEXT: // Locks for a FLAG character 			 

					  if(GetDMCflag(DMC_ZOOM_X2))
					  {
							LabelCmdMsg[OSD_ZOOM_ID].Text_id			= OSD_ZOOM_ID;
							strcpy(LabelCmdMsg[OSD_ZOOM_ID].Text,"Zx2");
							LabelCmdMsg[OSD_ZOOM_ID].Text[strlen(LabelCmdMsg[OSD_ZOOM_ID].Text)] = 0;	
										   
					  }
					  else if(GetDMCflag(DMC_ZOOM_X4))
					  {
							LabelCmdMsg[OSD_ZOOM_ID].Text_id			= OSD_ZOOM_ID;
							strcpy(LabelCmdMsg[OSD_ZOOM_ID].Text,"Zx4");
							LabelCmdMsg[OSD_ZOOM_ID].Text[strlen(LabelCmdMsg[OSD_ZOOM_ID].Text)] = 0;										 
					  }
					  else if(GetDMCflag(DMC_DISABLE_ZOOM))
					  {
							LabelCmdMsg[OSD_ZOOM_ID].Text_id			= OSD_ZOOM_ID;
							strcpy(LabelCmdMsg[OSD_ZOOM_ID].Text,"Zx0");
							LabelCmdMsg[OSD_ZOOM_ID].Text[strlen(LabelCmdMsg[OSD_ZOOM_ID].Text)] = 0;											 
					  }		
					  
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
WORD UploadZoomPosition(WORD mode)
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
 * FUNCTION:  FreezeUnfreezeVideo - Main Com Proc message processing loop
 * RETURN: Non-zero of message detected  
 */
WORD FreezeUnfreezeVideo(WORD mode)
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
 * FUNCTION:  FreezeUnfreezeVideo - Main Com Proc message processing loop
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
 * FUNCTION:  Update2P - Main Com Proc message processing loop
 * RETURN: Non-zero of message detected  
 */
WORD Update2P(WORD mode)
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
  WORD SetTextLabelCommand(WORD mode)
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
				  cpPutMessage(UART1_BASE,0xF050,0x2,&LabelCmdMsg[g_ucid_lebel].Label_id_Visibility[0]);			  
				  ctrl.state[TEXT_LABEL] = WAIT_FOR_MESSAGE_READY;

  
			 break;
  
			 case X_LABEL_POSITION: // Locks for a FLAG character
			 
				  ctrl.ret_mode[TEXT_LABEL] = 0xF051;
				  ctrl.next_mode[TEXT_LABEL] = LEBEL_TEXT;
				  ctrl.exit_mode[TEXT_LABEL] = X_LABEL_POSITION;
				  cpPutMessage(UART1_BASE,0xF051,0x5,&LabelCmdMsg[g_ucid_lebel].Position_id_X_Y[0]);			  
				  ctrl.state[TEXT_LABEL] = WAIT_FOR_MESSAGE_READY;
				  ctrl.time[TEXT_LABEL] = g_ulTickCount;
				  
			 
			 break;
  
			 case LEBEL_TEXT: // Locks for a FLAG character 			 
			 
				  ctrl.ret_mode[TEXT_LABEL] = 0xF052;
				  ctrl.next_mode[TEXT_LABEL] = STYLE_FORMAT;
				  ctrl.exit_mode[TEXT_LABEL] = LEBEL_TEXT;
				  cpPutMessage(UART1_BASE,0xF052,strlen(LabelCmdMsg[g_ucid_lebel].Text)+2,&LabelCmdMsg[g_ucid_lebel].Text_id);
				  ctrl.state[TEXT_LABEL] = WAIT_FOR_MESSAGE_READY;
				  ctrl.time[TEXT_LABEL] = g_ulTickCount;		  
				  
			 
			 break;
  
			 case STYLE_FORMAT: // Locks for a FLAG character
			 
				 ctrl.ret_mode[TEXT_LABEL] = 0xF053;
				 ctrl.next_mode[TEXT_LABEL] = END_LEBEL_CMD;
				 ctrl.exit_mode[TEXT_LABEL] = STYLE_FORMAT;
				 
				 cpPutMessage(UART1_BASE,0xF053,0x7,&LabelCmdMsg[g_ucid_lebel].Style_id_Color_Size_BackGro_text[0]);
				 ctrl.state[TEXT_LABEL] = WAIT_FOR_MESSAGE_READY;
				 ctrl.time[TEXT_LABEL] = g_ulTickCount;
			 
			 break;
  
			 case END_LEBEL_CMD: // Locks for a FLAG character

			 	 if(++g_ucid_lebel < MAX_LABEL_ID)
			 	 {
				 	ctrl.state[TEXT_LABEL] = HIDE_SHOW_LABEL;	  
				 	ctrl.time[TEXT_LABEL] = g_ulTickCount;
					cpReset(1); 
				 	mode = SET_LABEL_COMMAND;
			 	 }
				 else
				 {
					 ctrl.state[TEXT_LABEL] = HIDE_SHOW_LABEL;	   
					 ctrl.time[TEXT_LABEL] = g_ulTickCount;
					 cpReset(1);  
					 g_ucid_lebel = 0;
		 			 mode = ONLINE_STATE;

				 }
				 
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
						 g_ucid_lebel = 0;
						 mode = SET_LABEL_COMMAND;
					 }
					   
				 }			
				 else
				 if ((WORD)(g_ulTickCount - ctrl.time[TEXT_LABEL]) >= (WORD)3000) //-- every 0.5 sec  
				 {
					   ctrl.state[TEXT_LABEL] = HIDE_SHOW_LABEL;	
					   ctrl.time[TEXT_LABEL] = g_ulTickCount;
					   cpReset(1);	
					   g_ucid_lebel = 0;
					   mode = SET_LABEL_COMMAND;
					   
				 }
			 
			 break;
		  
			 default:
			 	
				 ctrl.state[TEXT_LABEL] = END_LEBEL_CMD; 
				 cpReset(1);	
				 ctrl.time[TEXT_LABEL] = g_ulTickCount;
				 g_ucid_lebel = 0;
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
			
				ctrl.state[TEXT_LABEL1] = LEBEL_TEXT;	 
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
			   if ((WORD)(GetSysTickCount() - ctrl.time[TEXT_LABEL1]) >= (WORD)1000) //-- every 0.5 sec	
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
 
void Set_LABEL_Visibility(BYTE idx,BYTE Visib)
{
	LabelCmdMsg[idx].Label_id_Visibility[1]	=Visib;		
}


WORD OnOffStabilize(WORD mode)
{
	
	cpPutMessage(UART1_BASE,0xF032,1,&Stabilize_status);
	
	return ONLINE_STATE;

}

  /*
   * FUNCTION:	Set_Label_Position 
   * RETURN: Non-zero of message detected  
   */
  WORD GraphicsOnOff(WORD mode)
  {
  
	  WORD n; 

		  /*
		   * Switch for state of tge message protocol processing
		   */
		   switch (ctrl.state[TEXT_LABEL])
		   {
		  
			 case HIDE_SHOW_LABEL: // Locks for a FLAG character
  
				  ctrl.ret_mode[TEXT_LABEL] = 0xF050;			  
				  ctrl.next_mode[TEXT_LABEL] = END_LEBEL_CMD;			  
				  ctrl.exit_mode[TEXT_LABEL] = HIDE_SHOW_LABEL;
				  ctrl.time[TEXT_LABEL] = g_ulTickCount;
				  Set_LABEL_Visibility(g_ucid_lebel,g_ucvisible_lebel);
				  cpPutMessage(UART1_BASE,0xF050,0x2,&LabelCmdMsg[g_ucid_lebel].Label_id_Visibility[0]);			  
				  ctrl.state[TEXT_LABEL] = WAIT_FOR_MESSAGE_READY;
  
  
			 break;
  
			 case END_LEBEL_CMD: // Locks for a FLAG character

			 	 if(++g_ucid_lebel < MAX_LABEL_ID)
			 	 {


					 ctrl.state[TEXT_LABEL] = HIDE_SHOW_LABEL;	  
					 ctrl.time[TEXT_LABEL] = g_ulTickCount;
					 mode = GRAPHICS_ON_OFF;					
			 	 }
				 else
				 {
					 ctrl.state[TEXT_LABEL] = HIDE_SHOW_LABEL;	   
					 ctrl.time[TEXT_LABEL] = g_ulTickCount;
					 g_ucid_lebel = 0;
					 cpReset(1);  
					 mode = ONLINE_STATE;


				 }
				 
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
						 g_ucid_lebel = 0;
						 ctrl.time[TEXT_LABEL] = g_ulTickCount;
						 mode = GRAPHICS_ON_OFF;
					 }
					   
				 }			
				 else
				 if ((WORD)(g_ulTickCount - ctrl.time[TEXT_LABEL]) >= (WORD)1000) //-- every 0.5 sec  
				 {
					   ctrl.state[TEXT_LABEL] = HIDE_SHOW_LABEL;	
					   ctrl.time[TEXT_LABEL] = g_ulTickCount;
					   cpReset(1);	
					   g_ucid_lebel = 0;
					   mode = GRAPHICS_ON_OFF;
					   
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
   * FUNCTION:	Set_Label_Position 
   * RETURN: Non-zero of message detected  
   */
WORD SetContrastBrithness(WORD mode)
{
	  
		  WORD n; 
		  /*
		   * Switch for state of tge message protocol processing
		   */
			   switch (ctrl.state[CON_BRIT])
			   {
			  
				 case SET_CONTRAST: // Locks for a FLAG character
	  
					  ctrl.ret_mode[CON_BRIT] = 0xF208;			  
					  ctrl.next_mode[CON_BRIT] = END_CONTRAST_CMD;			  
					  ctrl.exit_mode[CON_BRIT] = SET_CONTRAST;
					  cpPutMessage(UART1_BASE,0xF208,0x2,&m_Contrast_Set.contrast);			  
					  ctrl.state[CON_BRIT] = CONTRAST_WAIT_FOR_MESSAGE_READY;

					  
		  			  //m_Contrast_Set.contrast += 20;
	  
				 break;
	  
				 case END_CONTRAST_CMD: // Locks for a FLAG character
	
					 ctrl.state[CON_BRIT] = SET_CONTRAST;	   
					 ctrl.time[CON_BRIT] = g_ulTickCount;
					 cpReset(1);  
					 mode = ONLINE_STATE;
	
				 
				 break;
	  
				 case CONTRAST_WAIT_FOR_MESSAGE_READY:
				 
					 if (cpMessageReady(COM1))
					 {
						 n = cpGetMessageCommand(COM1); //tbd
						 if (n==ctrl.ret_mode[CON_BRIT])
						 {
							  ctrl.state[CON_BRIT] = ctrl.next_mode[CON_BRIT]; 
							  ctrl.time[CON_BRIT] = g_ulTickCount;
							  cpReset(1);
						 }
						 else
						 {
							 ctrl.state[CON_BRIT] = END_CONTRAST_CMD; 
							 cpReset(1);	
							 g_ucid_lebel = 0;
							 ctrl.time[CON_BRIT] = g_ulTickCount;
							 mode = SET_CONTRAST_BRITHNESS;
						 }
						   
					 }			
					 else
					 if ((WORD)(g_ulTickCount - ctrl.time[CON_BRIT]) >= (WORD)1000) //-- every 0.5 sec  
					 {
						   ctrl.state[CON_BRIT] = SET_CONTRAST;	
						   ctrl.time[CON_BRIT] = g_ulTickCount;
						   cpReset(1);	
						   g_ucid_lebel = 0;
						   mode = SET_CONTRAST_BRITHNESS;
						   
					 }
				 
				 break;
			  
				 default:
					
					 ctrl.state[CON_BRIT] = END_LEBEL_CMD; 
					 cpReset(1);	
					 ctrl.time[CON_BRIT] = g_ulTickCount;
					 mode = ONLINE_STATE;
	
					break;
			   }
						  
		  return mode;
	  
}	



  /*
   * FUNCTION:	Set_Label_Position 
   * RETURN: Non-zero of message detected  
   */
WORD UpdateGainLevel(WORD mode)
{
	  
		  WORD n; 
		  /*
		   * Switch for state of tge message protocol processing
		   */
			   switch (ctrl.state[UPDATE_GAIN])
			   {
			  
				 case SET_GAIN: // Locks for a FLAG character
	  
					  ctrl.ret_mode[UPDATE_GAIN] = 0xF204;			  
					  ctrl.next_mode[UPDATE_GAIN] = END_GAIN_CMD;			  
					  ctrl.exit_mode[UPDATE_GAIN] = SET_GAIN;
					  cpPutMessage(UART1_BASE,0xF204,0x01,(unsigned char*)&m_Gain_Params.gain_minimum_input);			  
					  ctrl.state[UPDATE_GAIN] = GAIN_WAIT_FOR_MESSAGE_READY;

					  
		  			  //m_Contrast_Set.contrast += 20;
	  
				 break;
	  
				 case END_GAIN_CMD: // Locks for a FLAG character
	
					 ctrl.state[UPDATE_GAIN] = SET_GAIN;	   
					 ctrl.time[UPDATE_GAIN] = g_ulTickCount;
					 cpReset(1);  
					 mode = ONLINE_STATE;
	
				 
				 break;
	  
				 case GAIN_WAIT_FOR_MESSAGE_READY:
				 
					 if (cpMessageReady(COM1))
					 {
						 n = cpGetMessageCommand(COM1); //tbd
						 if (n==ctrl.ret_mode[UPDATE_GAIN])
						 {
							  ctrl.state[UPDATE_GAIN] = ctrl.next_mode[UPDATE_GAIN]; 
							  ctrl.time[UPDATE_GAIN] = g_ulTickCount;
							  cpReset(1);
						 }
						 else
						 {
							 ctrl.state[UPDATE_GAIN] = END_GAIN_CMD; 
							 cpReset(1);	
							 g_ucid_lebel = 0;
							 ctrl.time[UPDATE_GAIN] = g_ulTickCount;
							 mode = SET_CONTRAST_BRITHNESS;
						 }
						   
					 }			
					 else
					 if ((WORD)(g_ulTickCount - ctrl.time[UPDATE_GAIN]) >= (WORD)1000) //-- every 0.5 sec  
					 {
						   ctrl.state[UPDATE_GAIN] = SET_GAIN;	
						   ctrl.time[UPDATE_GAIN] = g_ulTickCount;
						   cpReset(1);	
						   g_ucid_lebel = 0;
						   mode = SET_CONTRAST_BRITHNESS;
						   
					 }
				 
				 break;
			  
				 default:
					
					 ctrl.state[UPDATE_GAIN] = END_LEBEL_CMD; 
					 cpReset(1);	
					 ctrl.time[UPDATE_GAIN] = g_ulTickCount;
					 mode = ONLINE_STATE;
	
					break;
			   }
						  
		  return mode;
	  
}	




/*
  * FUNCTION:	Set_Label_Position 
  * RETURN: Non-zero of message detected  
 */
WORD SetBitMode(WORD mode)
{
	  
		  WORD n; 
		  /*
		   * Switch for state of tge message protocol processing
		   */
			   switch (ctrl.state[TIME_BIT])
			   {
			  
				 case SET_ENCODER: // Locks for a FLAG character

	  
					  //
					  // PITCH_MOTOR is PORTB & J6- PITCH Encoder 
					  //
					  // moves gimbal to extreme right and down
					  MotorBrakePositionClean(PITCH_MOTOR);
					  //GeneralPitchRoll(PITCH_MOTOR,A3906_FORWARD,500);	 
					  EncoderInitReset(QEI0_BASE,PITCH_MOTOR,A3906_REVERSE);  // J6 PITCH Encoder  
					  //EncoderInitReset(QEI0_BASE,PITCH_MOTOR,A3906_FORWARD);	// J6 PITCH Encoder  
					  //EncoderInitReset(QEI0_BASE,PITCH_MOTOR,A3906_REVERSE);	// J6 PITCH Encoder  
					  
					  //
					  // ROLL_MOTOR is PORTA & J8- ROLL Encoder
					  //
					  // moves gimbal to extreme right and down    
					  MotorBrakePositionClean(ROLL_MOTOR);
					  //GeneralPitchRoll(ROLL_MOTOR,A3906_FORWARD,800);
					  EncoderInitReset(QEI1_BASE,ROLL_MOTOR,A3906_FORWARD); // J8 ROLL Encoder	
					  //EncoderInitReset(QEI1_BASE,ROLL_MOTOR,A3906_REVERSE); // J8 ROLL Encoder
					  //EncoderInitReset(QEI1_BASE,ROLL_MOTOR,A3906_FORWARD); // J8 ROLL Encoder  
					  ctrl.state[TIME_BIT] = SET_AHRS;
	  
				 break;
	  
				 case SET_AHRS: // Locks for a FLAG character
	
					   AHRS_Init();
					   StabilizeInit();
				 	   ctrl.state[TIME_BIT] = SET_RELATIVE_MOTOR;
					   
				 break;

	  			 case SET_RELATIVE_MOTOR:
				 	
						 EncoderWhile2Point(PITCH_MOTOR,A3906_FORWARD,Motor[PITCH_MOTOR].relative);
						 EncoderWhile2Point(ROLL_MOTOR,A3906_REVERSE,Motor[ROLL_MOTOR].relative);
										 
						 Motor[PITCH_MOTOR].zero_pos = QEIPositionGet(QEI0_BASE);
						 Motor[ROLL_MOTOR].zero_pos = QEIPositionGet(QEI1_BASE);
						 
						 /*------------------------------------------------------------*/
						 /* BMA180GetXYZ() time is 30~ microsecond							*/
						 /*------------------------------------------------------------*/				 
						 BMA180GetXYZ();				 
						 /*------------------------------------------------------------*/
						 /* AHRS() time is 25~ microsecond										  */
						 /*------------------------------------------------------------*/					 
						 AHRS();
						 
						 SetRelativeInertial();
						 ctrl.state[TIME_BIT] = SET_ENCODER;
						 SetDMCflag(DMC_ENGINE_CONNECT,DMC_ENGINE_CONNECT);
						 mode = GET_EMBEDDED_INIT_STATUS;
						 //MasterI2C0Disable(SysCtlClockGet() >= 40000000L);

				 break;

				   
				 default:
					
					 ctrl.state[TIME_BIT] = END_LEBEL_CMD; 
					 cpReset(1);	
					 ctrl.time[TIME_BIT] = g_ulTickCount;
					 mode = BIT_MODE;
	
					break;
			   }
						  
		  return mode;
	  
}	




  /*
   * FUNCTION:	Set_Label_Position 
   * RETURN: Non-zero of message detected  
   */
WORD BirdStatusResponse(WORD mode)
{
	  
		  WORD n; 
		  /*
		   * Switch for state of tge message protocol processing
		   */
			   switch (ctrl.state[TIME_BIRD_RES])
			   {
			  
				 case BIRD_STATUS: // Locks for a FLAG character
	  
					  ctrl.ret_mode[TIME_BIRD_RES] = 0xF00C;			  
					  ctrl.next_mode[TIME_BIRD_RES] = END_BIRD_STATUS;			  
					  ctrl.exit_mode[TIME_BIRD_RES] = BIRD_STATUS;
					  cpPutMessage(UART1_BASE,0xF00C,0,NULL);			  
					  ctrl.state[TIME_BIRD_RES] = BIRD_WAIT_FOR_MESSAGE_READY;

	  
				 break;
	  
				 case END_BIRD_STATUS: // Locks for a FLAG character
	
					 ctrl.state[TIME_BIRD_RES] = BIRD_STATUS;	   
					 ctrl.time[TIME_BIRD_RES] = g_ulTickCount;
					 cpReset(1);  
					 mode = ONLINE_STATE;
	
				 
				 break;
	  
				 case BIRD_WAIT_FOR_MESSAGE_READY:
				 
					 if (cpMessageReady(COM1))
					 {
						 n = cpGetMessageCommand(COM1); //tbd
						 if (n==ctrl.ret_mode[TIME_BIRD_RES])
						 {						 	  
							  BirdStatus.birdbit.raw = *(cpGetMessageData(COM1,0));
							  if(BirdStatus.birdbit.birdS.bit4)
							  {
								strcpy(LabelCmdMsg[OSD_ELEVET].Text,"S");
								LabelCmdMsg[OSD_ELEVET].Text[strlen(LabelCmdMsg[OSD_ELEVET].Text)] = 0;
								ctrl.state[TEXT_LABEL1] = LEBEL_TEXT;	 		
								Set_LABEL_Visibility(OSD_ELEVET,1);
								Put_Text_Label(OSD_ELEVET,1);
							  }
							  else
							  if(!BirdStatus.birdbit.birdS.bit4)
							  {
								strcpy(LabelCmdMsg[OSD_ELEVET].Text,"");
								LabelCmdMsg[OSD_ELEVET].Text[strlen(LabelCmdMsg[OSD_ELEVET].Text)] = 0;
								ctrl.state[TEXT_LABEL1] = LEBEL_TEXT;	 		
								Set_LABEL_Visibility(OSD_ELEVET,0);
								Put_Text_Label(OSD_ELEVET,0);
							  }							  	
							  
							  ctrl.state[TIME_BIRD_RES] = ctrl.next_mode[TIME_BIRD_RES]; 
							  ctrl.time[TIME_BIRD_RES] = g_ulTickCount;
							  cpReset(1);
							  mode = ONLINE_STATE;
						 }
						 else
						 {
							 ctrl.state[TIME_BIRD_RES] = BIRD_STATUS; 
							 cpReset(1);	
							 ctrl.time[TIME_BIRD_RES] = g_ulTickCount;
							 mode = BIRD_STATUS_RESPONSE;
						 }
						   
					 }			
					 else
					 if ((WORD)(g_ulTickCount - ctrl.time[TIME_BIRD_RES]) >= (WORD)5000) //-- every 0.5 sec  
					 {
						   ctrl.state[TIME_BIRD_RES] = BIRD_STATUS;	
						   ctrl.time[TIME_BIRD_RES] = g_ulTickCount;
						   cpReset(1);	
						   mode = BIRD_STATUS_RESPONSE;
						   
					 }
				 
				 break;
			  
				 default:
					
					 ctrl.state[TIME_BIRD_RES] = BIRD_STATUS; 
					 cpReset(1);	
					 ctrl.time[TIME_BIRD_RES] = g_ulTickCount;
					 mode = ONLINE_STATE;
	
					break;
			   }
						  
		  return mode;
	  
}	





  /*
   * FUNCTION:	Set_Label_Position 
   * RETURN: Non-zero of message detected  
   */
WORD GetEmbeddedUIInitStatus(WORD mode)
{
	  
		  WORD n; 
		  /*
		   * Switch for state of tge message protocol processing
		   */
			   switch (ctrl.state[INIT_BIRD])
			   {
			  
				 case BIRD_INIT: // Locks for a FLAG character
	  
					  ctrl.ret_mode[INIT_BIRD] = 0xF049;			  
					  ctrl.next_mode[INIT_BIRD] = END_BIRD_STATUS;			  
					  ctrl.exit_mode[INIT_BIRD] = BIRD_STATUS;
					  cpPutMessage(UART1_BASE,0xF049,0,NULL);			  
					  ctrl.state[INIT_BIRD] = BIRD_WAIT_FOR_MESSAGE_READY;

	  
				 break;
	  
				 case END_BIRD_INIT: // Locks for a FLAG character
	
					 ctrl.state[INIT_BIRD] = BIRD_STATUS;	   
					 ctrl.time[INIT_BIRD] = g_ulTickCount;
					 cpReset(1);  
					 mode = ONLINE_STATE;
	
				 
				 break;
	  
				 case BIRD_INIT_WAIT_FOR_MESSAGE_READY:
				 
					 if (cpMessageReady(COM1))
					 {
						 n = cpGetMessageCommand(COM1); //tbd
						 if (n==ctrl.ret_mode[INIT_BIRD])
						 {						 	  
							  BirdStatus.birdbit.raw = *(cpGetMessageData(COM1,0));
							  if(BirdStatus.birdbit.birdS.bit0)
							  {
								ctrl.state[INIT_BIRD] = END_BIRD_INIT; 
								ctrl.time[INIT_BIRD] = g_ulTickCount;
								cpReset(1);
								mode = UPLOAD_VERSION;
							  
							  }
							  else
							  {
								ctrl.state[INIT_BIRD] = BIRD_INIT; 
								ctrl.time[INIT_BIRD] = g_ulTickCount;
								cpReset(1);
								mode = GET_EMBEDDED_INIT_STATUS;
							  }							  	
							  
						 }
						 else
						 {
							 ctrl.state[INIT_BIRD] = BIRD_INIT; 
							 cpReset(1);	
							 ctrl.time[INIT_BIRD] = g_ulTickCount;
							 mode = GET_EMBEDDED_INIT_STATUS;
						 }
						   
					 }			
					 else
					 if ((WORD)(g_ulTickCount - ctrl.time[INIT_BIRD]) >= (WORD)5000) //-- every 0.5 sec  
					 {
						   ctrl.state[INIT_BIRD] = BIRD_INIT;	
						   ctrl.time[INIT_BIRD] = g_ulTickCount;
						   cpReset(1);	
						   mode = GET_EMBEDDED_INIT_STATUS;
						   
					 }
				 
				 break;
			  
				 default:
					
					 ctrl.state[INIT_BIRD] = BIRD_INIT; 
					 cpReset(1);	
					 ctrl.time[INIT_BIRD] = g_ulTickCount;
					 mode = BIT_MODE;
	
					break;
			   }
						  
		  return mode;
	  
}	
  
