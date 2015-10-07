/*
  Copyright Bluebird Aero Systems Engineering 2011

  $Id: mtype.h,v 1.3 2011/04/10 10:37:29 EranS Exp $
  $Log: ITG3200.h,v $
  Revision 1.3  2011/04/10 10:37:29  EranS
  Add filer3.c and AHRS from Lumus project

  Revision 1.2  2011/04/06 06:44:20  EranS
  cleaning up, add some constants, move gyro_at_rest code

  Revision 1.1  2011/03/15 12:23:45  EranS
  First save


*/

#ifndef _MODEID_H
#define _MODEID_H

// RX/TX commands from Control  (via communication):
enum VectorMode {
	DOWNLOAD_VERSION   = 0,	// 0x00
	UPLOAD_VERSION,			// 0x01
	ONLINE_STATE,			// 0x02
	BLACK_HOT,				// 0x03   	
	WHITE_HOT,				// 0x04	     
	UPLOAD_G_PARAM,			// 0x05
	NULL_MODE_1,			// 0x06	  
	OPNE_CLOSE_SHUTTER,		// 0x07	  
	SHUTTER_READ_STATUS,	// 0x08			  
	SHUTTER_WRITE_STATUS,	// 0x09			  
	ENABLED_ARC,			// 0x0A	 
 	DISABLED_ARC,			// 0x0B
	ENABLED_CIC,			// 0x0C
	DISABLED_CIC,			// 0x0D
	UPDATE2_P,				// 0x0E
	NULL_MODE,			   	// 0x0F
	OFFLINE_STATE,			// 0x10
	FREEZE_UNFREEZE,		// 0x11
	UPDATE_V_FLIP,			// 0x12
	UPDATE_V_FLIP_D,		// 0x13
	UPDATE_H_FLIP,			// 0x14
	UPDATE_H_FLIP_D,		// 0x15
	UPDATE_ZOOMX2,			// 0x16
	UPDATE_ZOOMX4,			// 0x17
	UPDATE_DISABLE_ZOOM,	// 0x18	
	UPDATE_ZOOM_POSITION,	// 0x19
	TEC_READ_HANDLER,		// 0x1A
	SHOW_HIDE_LABEL,		// 0x1B
	SET_LEBEL_POSITION,		// 0x1C
	SET_LEBEL_TEXT,			// 0x1D
	SET_LEBEL_STYLE_FORMAT, // 0x1E
	SET_OSD_ON,				// 0x1F
	SET_OSD_OFF,			// 0x20	
	MINTRON_ZOOM_IN_OUT,	// 0x21
	MINTRON_READ_ZOOM,		// 0x22
	OSD_TITEL_ITEM,			// 0x23
	OSD_TITEL_POSITION,		// 0x24	
	ZOOM_MAG_ON,			// 0x25	
	ZOOM_MAG_OFF,			// 0x26
	MARK_ON_ON,				// 0x27
	MARK_ON_OFF,			// 0x28
	INITIALIZATION_UNIT		// 0x29
};


#endif
