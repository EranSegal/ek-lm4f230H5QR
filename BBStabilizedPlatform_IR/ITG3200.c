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

void ITG3200Init()
{
    unsigned long timeout;
	
    // force a wait
    timeout = GetSysTickCount() + 1000;
    while (GetSysTickCount() < timeout);
  
    // sanity check that we are talking    
    while (itgregs[WHO_AM_I] != ITG3200_SLAVE_ADDRESS_A1)
      MasterI2C0Read(ITG3200_SLAVE_ADDRESS,&itgregs[WHO_AM_I],WHO_AM_I,1);    // read ITG3200_WHO_AM_I
    // set full scale and low pass filter
    itgregs[DLPF_FS] = FS_SEL + Hz256;
    MasterI2C0Write(ITG3200_SLAVE_ADDRESS,&itgregs[DLPF_FS], (unsigned long)DLPF_FS,1L);
    // congfigure ITG3200 to interrupt at desired rate
    itgregs[SMPLRT_DIV] = (8000/ITG3200_SAMPLE_RATE)-1;
    MasterI2C0Write(ITG3200_SLAVE_ADDRESS,&itgregs[SMPLRT_DIV], (unsigned long)SMPLRT_DIV,1L);
    // sanity check that I wrote OK
    MasterI2C0Read(ITG3200_SLAVE_ADDRESS,&itgregs[SMPLRT_DIV],SMPLRT_DIV,1);
    while (itgregs[SMPLRT_DIV] != ((8000/ITG3200_SAMPLE_RATE)-1))
      MasterI2C0Read(ITG3200_SLAVE_ADDRESS,&itgregs[0],SMPLRT_DIV,1);
    
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
  itgready = false;
  MasterI2C0Read(ITG3200_SLAVE_ADDRESS,&itgregs[XOUT_H],XOUT_H,6);    // read X,Y,Z

  //DEBUG_TP6(1);  
  // This code takes 1.7 us
  ITG3200values[0] = itgregs[XOUT_H] << 8;
  ITG3200values[0] |= itgregs[XOUT_L];

  ITG3200values[1] = itgregs[YOUT_H] << 8;
  ITG3200values[1] |= itgregs[YOUT_L];

  ITG3200values[2] = itgregs[ZOUT_H] << 8;
  ITG3200values[2] |= itgregs[ZOUT_L];
  //DEBUG_TP6(0); 

  //DEBUG_TP6(1); 
  // This code takes 2.7 us
  //memcpy(ITG3200values,&itgregs[XOUT_H],6);
  //for (i=0; i<3; i++)
    //swapb((unsigned short *)&ITG3200values[i]);
  //DEBUG_TP6(0);  
}
