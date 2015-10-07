/*
  Copyright Bluebird Aero Systems Engineering 2012

  $Id: ITG3200.c,v 1.6 2011/04/17 08:44:11 EranS Exp $
  $Log: ITG3200.c,v $
  Revision 1.6  2011/04/17 08:44:11  EranS

  Revision 1.1  2011/03/15 12:23:45  EranS
  First save

 */

#include <string.h>
#include "bluebird.h"
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/debug.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/sysctl.h"
#include "MasterI2C.h"
#include "utilities.h"
#define ITG_MAIN
#include "ITG3200.h"
#include "SysTick.h"
#include "GPIOhandler.h"
#include "Stabilize.h"
#include "Pins.h"
#include "reg.h"

void ITG3200_Write_I2C_Register(BYTE page, BYTE Addr, BYTE Data);
BYTE ITG3200_Read_I2C_Register(BYTE page, BYTE Addr);

void ITG3200Init()
{
    unsigned long timeout;
	
    // force a wait
    //timeout = SysTickCountd + 1000;
    //while (SysTickCountd < timeout);
  
    // sanity check that we are talking    
    while (itgregs[WHO_AM_I] != ITG3200_SLAVE_ADDRESS_A1)
      MasterI2C0Read(ITG3200_SLAVE_ADDRESS,&itgregs[WHO_AM_I],WHO_AM_I,1);    // read ITG3200_WHO_AM_I
    // set full scale and low pass filter
    itgregs[DLPF_FS] = FS_SEL + Hz256;
    MasterI2C0Write(ITG3200_SLAVE_ADDRESS,&itgregs[DLPF_FS], (unsigned long)DLPF_FS,1L);

	#ifdef USE_ITG3200_INTERRUPT
    // congfigure ITG3200 to interrupt at desired rate
    itgregs[SMPLRT_DIV] = (8000/ITG3200_SAMPLE_RATE)-1;
    MasterI2C0Write(ITG3200_SLAVE_ADDRESS,&itgregs[SMPLRT_DIV], (unsigned long)SMPLRT_DIV,1L);
    // sanity check that I wrote OK
    MasterI2C0Read(ITG3200_SLAVE_ADDRESS,&itgregs[SMPLRT_DIV],SMPLRT_DIV,1);
    while (itgregs[SMPLRT_DIV] != ((8000/ITG3200_SAMPLE_RATE)-1))
      MasterI2C0Read(ITG3200_SLAVE_ADDRESS,&itgregs[0],SMPLRT_DIV,1);
    #endif
	
    itgregs[INT_CFG] = RAW_RDY_EN;
    MasterI2C0Write(ITG3200_SLAVE_ADDRESS,&itgregs[INT_CFG], (unsigned long)INT_CFG,1L);
    MasterI2C0Read(ITG3200_SLAVE_ADDRESS,&itgregs[INT_CFG],INT_CFG,1);
    while (itgregs[INT_CFG] != RAW_RDY_EN)
      MasterI2C0Read(ITG3200_SLAVE_ADDRESS,&itgregs[INT_CFG],INT_CFG,1);
#ifdef USE_ITG3200_INTERRUPT
    itgready = false;
    GPIOPinInit(ITG3200_INTERRUPT_PORT,ITG3200_INTERRUPT_PIN,true,true);
#endif
}

void ITG3200Action()
{
  itgready = true;
}

void ITG3200_Write_I2C_Register(BYTE page, BYTE Addr, BYTE Data)
{
   I2C_Start(); 
   I2CSendByte(ITG3200_SLAVE_ADDRESS);
   I2CSendByte(page);
   I2CSendByte(Addr);
   I2CSendByte(Data);
   I2C_Stop();
}

BYTE ITG3200_Read_I2C_Register(BYTE page, BYTE Addr)
{
   BYTE data;

   I2C_Start(); 
   I2CSendByte(ITG3200_SLAVE_ADDRESS);
   I2CSendByte(page);
   I2CSendByte(Addr);
   I2C_Start(); 
   I2CSendByte(ITG3200_SLAVE_ADDRESS | 0x01); // read slave address.
   data = I2CReceiveLastByte();
   I2C_Stop();

   return data;
}

// read X/Y/Z registers and swap endian
void ITG3200getXYZ()
{
  int i;
#ifndef USE_ITG3200_INTERRUPT
  // poll status until raw data ready
  itgregs[INT_STATUS] = 0;
  while (!(itgregs[INT_STATUS] & RAW_RDY_EN))
    MasterI2C0Read(ITG3200_SLAVE_ADDRESS,&itgregs[INT_STATUS],INT_STATUS,1);  // read status
#endif

#if 1
  itgready = false;
  MasterI2C0Read(ITG3200_SLAVE_ADDRESS,&itgregs[XOUT_H],XOUT_H,6);    // read X,Y,Z
 
  memcpy(ITG3200values,&itgregs[XOUT_H],6);
  for (i=0; i<3; i++)
    swapb((unsigned short *)&ITG3200values[i]);
#else
  // sanity check that we are talking    
  I2C_Send1(ITG3200_SLAVE_ADDRESS, XOUT_H); 
  ITG3200values[0] = I2C_Recv2(ITG3200_SLAVE_ADDRESS);    // read ITG3200_WHO_AM_I

  I2C_Send1(ITG3200_SLAVE_ADDRESS, YOUT_H); 
  ITG3200values[1] = I2C_Recv2(ITG3200_SLAVE_ADDRESS);    // read ITG3200_WHO_AM_I

  I2C_Send1(ITG3200_SLAVE_ADDRESS, ZOUT_H); 
  ITG3200values[2] = I2C_Recv2(ITG3200_SLAVE_ADDRESS);    // read ITG3200_WHO_AM_I

  for (i=0; i<3; i++)
    swapb((unsigned short *)&ITG3200values[i]);
#endif

}
