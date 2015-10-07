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

#ifndef __UART_ECHO_H__
#define __UART_ECHO_H__

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

int InitUART(unsigned long ulBase, unsigned long ulUARTClk,unsigned long ulBaud, unsigned long ulConfi);
void UART0IntHandler(void);
void UART1IntHandler(void);
void UART0Send(unsigned char *pucBuffer, unsigned ulCount);
void UART1Send(unsigned char *pucBuffer, unsigned ulCount);   
#endif

