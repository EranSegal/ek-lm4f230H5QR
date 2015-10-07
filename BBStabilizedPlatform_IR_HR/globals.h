//*****************************************************************************
//
// globals.h - Defines and Macros for the UART.
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

#ifndef __GLOBALS_H__
#define __GLOBALS_H__

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

#ifndef __REG_H__
#include "reg.h"
#endif

#define BIRD_384

//#define VERSION_1_3

#define VERSION_1_5

#define FALSE 0
#define TRUE 1

//#define NULL	((void *)0)

//#define OMAP_RESIZER_SUPPORTED
#define OMAP_VE
#define ENGINE_TYPE ENGINE_OMAP_25U_384

// Supported features
#define TEC_SUPPORTED
//#define TECLESS_SUPPORTED
#define POWER_SAVE_SUPPORTED
#define ARC_SUPPORTED
#define SBC_SUPPORTED
#define CIC_SUPPORTED
#define FPGA_COMP_SUPPORTED
#define MAX_NUM_OF_FRAMES_FOR_ARC_OFFSET_UPDATE 25
	
#define SHUTTER_SUPPORTED
	
#define	BAUDRATE0 B115200
#define	BAUDRATE1 B115200

#define	BAUDRATE_PC_UART	BAUDRATE0
#define	BAUDRATE_FPGA_UART	BAUDRATE1
#define FPGA_BAUD_DEF	16	//115200 baud
// End of supported features

#ifdef BIRD_384
#define NUM_OF_COLUMNS				396
#define NUM_OF_ROWS					288
#define IMAGE_WIDTH					396
#define IMAGE_HIGHT					288
#define NUM_OF_TABLE_COLUMNS		384
#define NUM_OF_AUX_COLUMNS			2
#define OUTPUT_BYTE_SHIFT			168
#define NET_IMAGE_START				6 //zero based
#define NET_IMAGE_END				391//zero based
#define CDS_ROW_BLANKING_HEIGHT		0
#elif BIRD_640
#define NUM_OF_COLUMNS				396
#define NUM_OF_ROWS					288
#define IMAGE_WIDTH					656
#define IMAGE_HIGHT					480
#define NUM_OF_TABLE_COLUMNS		384
#define NUM_OF_AUX_COLUMNS			2
#define OUTPUT_BYTE_SHIFT			168
#define NET_IMAGE_START				6 //zero based
#define NET_IMAGE_END				391//zero based
#define CDS_ROW_BLANKING_HEIGHT		0
#endif
#define	CASE_REFERENCE_COLUMN_1		5
#define	CASE_REFERENCE_COLUMN_2		6

#define COMPENS_ROWS				(NUM_OF_ROWS+2)
#define COMPENS_WIDTH				(NUM_OF_COLUMNS+5)
#define COMPENS_BYTES_OFFSET		(5 * sizeof(Uint16))
#define COMPENS_WORDS_OFFSET		5
#define ROW_DATA_CMD_ID				0x02C9
#define ROW_DATA_CMD_GLOBALS		0
#define RRB_BUFF_SIZE				1024 //in words
#define RRB_CMD_SIZE				(NUM_OF_ROWS*4+4) //2 bolometers per row * 2 bytes per bolometer + 2 Global_bolometers + 2 ID and ~ID
#define RRB_CMD_ID					0x009B

#define DRC_ALG_TYPE_PARAM			600

#define ARC_THRESHOLD_FINE_DEFAULT  10
#define ARC_THRESHOLD_COARSE_DEFAULT 40
#define ARC_THRESHOLD_FINE_MIN      10
#define ARC_THRESHOLD_COARSE_MIN    40
#define ARC_THRESHOLD_FINE_MAX      10
#define ARC_THRESHOLD_COARSE_MAX    40
#define ARC_TIME_TRIGGER            180

/*  DRC definitions */
#define DRC_TYPE_LIN_AROUND_MEDIAN_BY_STDEV		4
#define DRC_TYPE_LIN_AROUND_MEDIAN_BY_TAIL		3
#define DRC_TYPE_LIN_DROP_BOTH_SIDES_TAIL		2
#define DRC_TYPE_CLHE							1
#define DRC_TYPE_LIN							0
#define DRC_NO									5
#define B_HOT_POLARITY						    0
#define W_HOT_POLARITY						    1

#define A_VIDEO									0
#define D_VIDEO								    1

//*****************************************************************************
//
// DMC ( Data Mode Command )	flags mode bit mask
//
//*****************************************************************************
#define DMC_PILOT_WINDOW    0x00000001
#define DMC_STOW		    0x00000002
#define DMC_ZOOM_X2        	0x00000004
#define DMC_ZOOM_X4   		0x00000008

#define DMC_DISABLE_ZOOM	0x00000010
#define DMC_STABILIZE_ON   	0x00000020
#define DMC_STABILIZE_OFF	0x00000040
#define DMC_CAM_GUIDE 		0x00000080

#define DMC_RATE			0x00000100 
#define DMC_POINT_2_COORD  	0x00000200
#define DMC_PTC_CG			0x00000400 // CAM_GUIED+POINT2COORDENET
#define DMC_OSD_ON         	0x00000800

#define DMC_BIT_MODE   		0x00001000
#define DMC_BLACK_HOT		0x00002000
#define DMC_WHITE_HOT		0x00004000
#define DMC_ENGINE_CONNECT	0x00008000
#define DMC_HOLD_COORDINATE	0x00010000
#define DMC_CENTER_MODE		0x00020000


/********************************************************************/
/*				  message structure (SCD protocol)											*/
/*			 byte 0 	  -- header (0xAA for port 0, 0x55 for port 1)					*/
/*			 byte 1 - 2   -- command id 													*/
/*			 byte 3 - 4   -- length 															*/
/*			 byte 5 - 4 + length   -- data (if length is 0 , no 						*/
/*			 byte 4 + length + 1   -- checksum												*/
/********************************************************************/
	
#define UDPATE_SHUTTER		  	0
#define WAIT_DONE 		 		1

enum Open_Clode_Shutter {

	SET_TEMPERATURE = 0,		// 0x00
	READ_SHUTTER_STATUS,		// 0x01
	WRITE_SHUTTER_STATUS,		// 0x02
	SET_CLOSE_OPEN_SHUTTER,		// 0x03
	X_WRITE_SHUTTER_STATUS,		// 0x04
	X_READ_SHUTTER_STATUS,		// 0x05
	WAIT_FOR_MESSAGE			// 0x06
};

enum Update_2_P_Mode {

	DISABLED_CIC_2_P = 0,				//	0x00
	DISABLED_ARC_2_P,					//	0x01
	CLOSE_SHUTTER_2_P,					//	0x02
	READ_SHUTTER_STATUS_2_P,			//	0x03
	WRITE_SHUTTER_STATUS_2_P,			//	0x04
	F_AVG_SET_PARAM,					//	0x05
	POLL_COMPILETION_FRAME_AVERAGING,	//	0x06
	COMPLETED_FRAME_AVERAGING,			//	0x07
	ENABLED_CIC_2_P,					//	0x08
	SET_NEW_CIC_REFERENCE,				//	0x09
	SET_NEW_NUC_REFERENCE,				//	0x0A
	OPEN_SHUTTER_2_P,					//	0x0B
	READ_SHUTTER_STATUS_2_P_2,			//	0x0C
	WRITE_SHUTTER_STATUS_2_P_2,			//	0x0D
	ENABLED_ARC_2_P,					//	0x0E
	END_UPDATE_2_P,						//	0x0F
	WAIT_FOR_MESSAGE_READY				//	0x10
};

enum Refresh_NUC_Data {

	REFRESH_NUC_CMD = 0,				//	0x00
	WAIT_FOR_NUC_MESSAGE_READY			//	0x01
};

enum UV_Label_Text {

  	  UV_SENT_MESSAGE = 0,					// 0x00
	  UV_HIDE_SHOW_LABEL, 			  	//  0x01
	  UV_X_LABEL_POSITION, 		    //  0x02
	  UV_LEBEL_TEXT,				  	//  0x03
	  UV_STYLE_FORMAT,			  		//  0x04
	  UV_END_LEBEL_CMD,					// 0x05
	  UV_WAIT_FOR_MESSAGE_READY			//0x06
};

enum Contrast_Gain {

	  SET_CONTRAST = 0, 			  	//  0x00
	  END_CONTRAST_CMD,				// 0x01
	  CONTRAST_WAIT_FOR_MESSAGE_READY
};

enum Gain_Level {

	  SET_GAIN = 0, 			  	//  0x00
	  END_GAIN_CMD,				// 0x01
	  GAIN_WAIT_FOR_MESSAGE_READY
};

enum Bird_Response {

	  BIRD_STATUS = 0, 			  	//  0x00
	  END_BIRD_STATUS,				// 0x01
	  BIRD_WAIT_FOR_MESSAGE_READY
};

enum Bird_Init {

	  BIRD_INIT = 0, 			  	//  0x00
	  END_BIRD_INIT,				// 0x01
	  BIRD_INIT_WAIT_FOR_MESSAGE_READY
};

enum Bit_Mode_Set {

	  SET_ENCODER = 0, 			  	// 0x00
	  SET_AHRS,						// 0x01
	  SET_RELATIVE_MOTOR,			// 0x02
	  BTI_WAIT_FOR_READY			// 0x03
};

enum Label_Text {

	  HIDE_SHOW_LABEL = 0, 			  	//  0x01
	  X_LABEL_POSITION, 		    //  0x02
	  LEBEL_TEXT,				  	//  0x03
	  STYLE_FORMAT,			  		//  0x04
	  END_LEBEL_CMD					// 0x05
};

enum Dwonload_version {

	  SEND_DOWNLOAD_MESSAGE = 0, 		//  0x00
	  WAIT_DOWNLOAD_VERSION, 		    //  0x01
};

enum BH_Label_Text {

  	  BH_SENT_MESSAGE = 0,					// 0x00
  	  BH_SENT_LABEL,
	  BH_HIDE_SHOW_LABEL, 			  	//  0x01
	  BH_X_LABEL_POSITION, 		    //  0x02
	  BH_LEBEL_TEXT,				  	//  0x03
	  BH_STYLE_FORMAT,			  		//  0x04
	  BH_END_LEBEL_CMD,					// 0x05
	  BH_WAIT_FOR_MESSAGE_READY			//0x06
};

enum WH_Label_Text {

  	  WH_SENT_MESSAGE = 0,					// 0x00
	  WH_HIDE_SHOW_LABEL, 			  	//  0x01
	  WH_X_LABEL_POSITION, 		    //  0x02
	  WH_LEBEL_TEXT,				  	//  0x03
	  WH_STYLE_FORMAT,			  		//  0x04
	  WH_END_LEBEL_CMD,					// 0x05
	  WH_WAIT_FOR_MESSAGE_READY			//0x06
};

enum ZX2_Label_Text {

  	  Z2_SENT_MESSAGE = 0,					// 0x00
	  Z2_HIDE_SHOW_LABEL, 			  	//  0x01
	  Z2_X_LABEL_POSITION, 		    //  0x02
	  Z2_LEBEL_TEXT,				  	//  0x03
	  Z2_STYLE_FORMAT,			  		//  0x04
	  Z2_END_LEBEL_CMD,					// 0x05
	  Z2_WAIT_FOR_MESSAGE_READY			//0x06
};

enum ZX4_Label_Text {

  	  Z4_SENT_MESSAGE = 0,					// 0x00
	  Z4_HIDE_SHOW_LABEL, 			  	//  0x01
	  Z4_X_LABEL_POSITION, 		    //  0x02
	  Z4_LEBEL_TEXT,				  	//  0x03
	  Z4_STYLE_FORMAT,			  		//  0x04
	  Z4_END_LEBEL_CMD,					// 0x05
	  Z4_WAIT_FOR_MESSAGE_READY			//0x06
};

enum Dis_Label_Text {

  	  DIS_SENT_MESSAGE = 0,					// 0x00
	  DIS_HIDE_SHOW_LABEL, 			  	//  0x01
	  DIS_X_LABEL_POSITION, 		    //  0x02
	  DIS_LEBEL_TEXT,				  	//  0x03
	  DIS_STYLE_FORMAT,			  		//  0x04
	  DIS_END_LEBEL_CMD,					// 0x05
	  DIS_WAIT_FOR_MESSAGE_READY			//0x06
};

WORD DownloadEngineVersion(WORD mode);	// DOWNLOAD_VERSION 		0x00
WORD UploadEngineVersion(WORD mode);		// DOWNLOAD_VERSION 	0x01   
WORD OnlineState(WORD mode);				// ONLINE_STATE (stop)		0x02  
WORD WhiteBlackHot(WORD mode);			//	BLACK_HOT					0x03   
WORD UploadGeneralParameters(WORD mode);	// UPLOAD_G_PARAM			0x05
WORD OpenCloseShutter(WORD mode);			// OPEN CLOSE SHUTTER
WORD ShutterReadStatus(WORD mode);		// SHUTTER_READ_STATUS	0x08
WORD ShutterWriteStatus(WORD mode);		// SHUTTER_WRITE_STATUS 	0x09
WORD Enabled_ARC(WORD mode);				// ENABLED_ARC			0x0A   
WORD Disabled_ARC(WORD mode);				// DISABLED_ARC 		0x0B
WORD Enabled_CIC(WORD mode);				// ENABLED_CIC				0x0C
WORD Disabled_CIC(WORD mode);				// DISABLED_CIC 		0x0D
WORD Update2P(WORD mode); 				// UPDATE_2_P				0x0E   
WORD NullMode(WORD mode);					//									0x0F  
WORD OfflineState(WORD mode);				// OFFLINE_STATE (stop) 	0x10
WORD FreezeUnfreezeVideo(WORD mode);	// 
WORD ZoomFlippingCommand(WORD mode);
WORD UploadZoomPosition(WORD mode);
WORD UploadHFlipEnable(WORD mode);
WORD UploadHFlipDisable(WORD mode);
WORD UploadVFlipEnable(WORD mode);
WORD UploadVFlipDisable(WORD mode);
WORD Tec_Read_Handler(WORD mode);
WORD SetTextLabelCommand(WORD mode);
WORD OnOffStabilize(WORD mode);
WORD RefreshNUCDataCommand(WORD mode);
WORD GraphicsOnOff(WORD mode);
WORD SetContrastBrithness(WORD mode);
WORD UpdateGainLevel(WORD mode);
WORD SetBitMode(WORD mode);
WORD BirdStatusResponse(WORD mode);
WORD GetEmbeddedUIInitStatus(WORD mode);



WORD InitializationUnit(WORD mode);

#endif
