//*****************************************************************************
//
// encoder.h - Prototypes for the QEI control module.
//
// Copyright (c) 2008-2009 Luminary Micro, Inc.  All rights reserved.
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
// This is part of revision 4652 of the RDK-BDC Firmware Package.
//
//*****************************************************************************

#ifndef __ENCODER_H__
#define __ENCODER_H__

#define BELT			2
#define CHANEL_SHAFT	4
#define ENC_LINES		64
//#define QEI0_GEAR_RATION		1518
#define QEI0_GEAR_RATION		650
#define QEI0_POSCNT_MAX 		CHANEL_SHAFT*ENC_LINES*QEI0_GEAR_RATION*BELT

//#define QEI1_GEAR_RATION		1518
#define QEI1_GEAR_RATION		650
#define QEI1_POSCNT_MAX 		CHANEL_SHAFT*ENC_LINES*QEI1_GEAR_RATION*BELT

#define FULL_ROLL_RANGE 290000
#define FULL_PITCH_RANGE 200000

#define FULL_SCALE_PITCH_MOTOR  133800L
#define FULL_SCALE_ROLL_MOTOR   193000L

#define POS2DEG (QEI0_POSCNT_MAX/360.0) // ~462.222
#define DEG2POS (360.0/QEI0_POSCNT_MAX)	// ~0.00216

#define SYSCLK_16MHZ        16*1024*1024
#define SYSCLK_20MHZ        20*1024*1024
#define SYSCLK_50MHZ        50*1024*1024

//*****************************************************************************
//
// A set of flags that track the state of the encoder stop tracking.
//
//*****************************************************************************
#define ENCODER_FLAG_VALID      0
#define ENCODER_FLAG_EDGE       1
#define ENCODER_FLAG_PREVIOUS   2
#define ENCODER_FLAG_PITCH   	3
#define ENCODER_FLAG_ROLL   	4
#define ENCODER_FLAG_EDGE_FUALSE   			5

//*****************************************************************************
//
// Function prototypes.
//
//*****************************************************************************
void Encoder_Init(unsigned long ulBase);
void EncoderTick(unsigned long ulBase);
void QEI0IntHandler(void);
void QEI1IntHandler(void);
void EncoderLinesSet(unsigned long ulBase,unsigned long ulLines);
unsigned long EncoderLinesGet(void);
void EncoderPositionSet(long lPosition);
float EncoderPositionGet(unsigned long ulBase);
long EncoderVelocityGet(long lSigned);
unsigned long EncoderCountGet(void);
unsigned long EncoderCountSet(unsigned long count);


#endif // __ENCODER_H__
