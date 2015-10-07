/*
  Copyright Bluebird Aero Systems Engineering 2012

  $Id: ITG3200.c,v 1.6 2011/04/17 08:44:11 EranS Exp $
  $Log: ITG3200.c,v $
  Revision 1.6  2011/04/17 08:44:11  EranS

  Revision 1.1  2011/03/15 12:23:45  EranS
  First save

 */

#include <stdint.h>
#include <string.h>
#include "bluebird.h"
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/debug.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "sensorlib/hw_ak8975.h"
#include "sensorlib/hw_mpu9150.h"
#include "sensorlib/i2cm_drv.h"
#include "sensorlib/ak8975.h"
#include "sensorlib/mpu9150.h"
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

//*****************************************************************************
//
// The factors used to convert the acceleration readings from the MPU9150 into
// floating point values in radians per second.
//
// Values are obtained by taking the degree per second conversion factors
// from the data sheet and then converting to radians per sec (1 degree =
// 0.0174532925 radians).
//
//*****************************************************************************
static const float g_fMPU9150GyroFactors[] =
{
    1.3323124e-4,                           // Range = +/- 250 dps (131.0)
    2.6646248e-4,                           // Range = +/- 500 dps (65.5)
    5.3211258e-4,                           // Range = +/- 1000 dps (32.8)
    0.0010642252                            // Range = +/- 2000 dps (16.4)
};


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
void ITG3200getXYZ(float *pfGyroX, float *pfGyroY,float *pfGyroZ)
{
  int i;
  float fFactor;
  int16_t i16Temp;
  
  itgready = false;
  MasterI2C0Read(ITG3200_SLAVE_ADDRESS,&itgregs[XOUT_H],XOUT_H,6);    // read X,Y,Z
  //DEBUG_TP6(1); 
  // This code takes 2.7 us
  memcpy(ITG3200values,&itgregs[XOUT_H],6);
  for (i=0; i<3; i++)
    swapb((unsigned short *)&ITG3200values[i]);
  //DEBUG_TP6(0);  
  
  //
  // Get the gyroscope conversion factor for the current data format.
  //
  fFactor = g_fMPU9150GyroFactors[3];

  //DEBUG_TP5(1);  
  // This code takes 1.7 us
  
  i16Temp = (int16_t)itgregs[XOUT_H];
  i16Temp <<= 8;
  i16Temp += itgregs[XOUT_L];
  *pfGyroX = (float)i16Temp;
  *pfGyroX *= fFactor;
		

  i16Temp = (int16_t)itgregs[YOUT_H];
  i16Temp <<= 8;
  i16Temp += itgregs[YOUT_L];  
  *pfGyroY = (float)i16Temp;
  *pfGyroY *= fFactor;
  
  i16Temp = (int16_t)itgregs[ZOUT_H];
  i16Temp <<= 8;
  i16Temp += itgregs[ZOUT_L];  
  *pfGyroZ  = (float)i16Temp;
  *pfGyroZ  *= fFactor;
 
  //DEBUG_TP5(0); 


}
