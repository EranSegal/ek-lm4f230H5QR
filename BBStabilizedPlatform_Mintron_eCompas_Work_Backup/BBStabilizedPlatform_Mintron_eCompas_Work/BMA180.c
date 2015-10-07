/*
  Copyright Bluebird Aero Systems Engineering 2012
  http://www.zickel.net

  $Id: BMA180.c,v 1.3 2011/04/17 08:44:11 EranS Exp $
  $Log: BMA180.c,v $
  Revision 1.3  2011/04/17 08:44:11  EranS
  add proper reset of encoders via GPIO

  Revision 1.2  2011/03/22 15:42:49  EranS
  added telemetry
  fixed adc code

  Revision 1.1  2011/03/15 12:23:45  EranS
  First save


*/

#include "bluebird.h"
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
//#include "inc/hw_nvic.h"
#include "inc/hw_types.h"
#include "driverlib/debug.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/sysctl.h"
#include "MasterI2C.h"
#define BMA180_MAIN
#include "bma180.h"
#include "SysTick.h"
#include "GPIOhandler.h"
#include "Pins.h"
#include "reg.h"

void BMA180Init()
{
  unsigned long timeout;
  bmaregs[SOFT_RESET] |= MAGIC_NUMBER;  
  MasterI2C0Write(BMA180_SLAVE_ADDRESS,&bmaregs[SOFT_RESET],SOFT_RESET,1);
  // force a wait
  timeout = GetSysTickCount() + 10;
  while (GetSysTickCount() < timeout);
  // read ID and version as sanity check
  while (bmaregs[CHIP_ID] != 3)
    MasterI2C0Read(BMA180_SLAVE_ADDRESS,&bmaregs[CHIP_ID],CHIP_ID,sizeof(bmaregs));  
  bmaregs[CTRLREG0] |= EE_W;    // unlock write
  MasterI2C0Write(BMA180_SLAVE_ADDRESS,&bmaregs[CTRLREG0],CTRLREG0,1);
  
  bmaregs[BWTCS] = (BW10HZ << BWSHIFT) | 4;  // typical TCS value
  MasterI2C0Write(BMA180_SLAVE_ADDRESS,&bmaregs[BWTCS],BWTCS,1);
  
  bmaregs[TCO_Z] &= MODE_MASK;
  bmaregs[TCO_Z] |= MODE_CONFIG_0;
  MasterI2C0Write(BMA180_SLAVE_ADDRESS,&bmaregs[TCO_Z],TCO_Z,1);

  bmaregs[OLSB1] = RANGE2<< RANGESHIFT;
  MasterI2C0Write(BMA180_SLAVE_ADDRESS,&bmaregs[OLSB1],OLSB1,1);  
  
 #ifdef USE_BMA180_INTERRUPT
  bmaregs[CTRLREG3] |= NEW_DATA_INT;
  #ifdef USE_BMA180_LATCH
    bmaregs[CTRLREG3] |= LAT_INT;
  #endif
  bmaregs[CTRLREG3] &= ~ADV_INT;
  MasterI2C0Write(BMA180_SLAVE_ADDRESS,&bmaregs[CTRLREG3],CTRLREG3,1);
  // sanity check
  MasterI2C0Read(BMA180_SLAVE_ADDRESS,&bmaregs[CHIP_ID],CHIP_ID,sizeof(bmaregs));  
#endif

  bmaregs[CTRLREG0] &= ~EE_W;    // reset the ee_w lock bit
  MasterI2C0Write(BMA180_SLAVE_ADDRESS,&bmaregs[CTRLREG0],CTRLREG0,1);
  
#ifdef USE_BMA180_INTERRUPT
  xbmaready = false;
  GPIOPinInit(BMA180_INTERRUPT_PORT,BMA180_INTERRUPT_PIN,true,true);
#endif
}

void BMA180Action()
{
  xbmaready = true;
}

/*
  Performance
    BW mode0 mode1 mode2 mode3
    10  1000  800   600   600
   150  1000              900
  1200  1000             1000
*/

// Read XYZ, need to check LSB of Z axis for data ready bit
void BMA180GetXYZ()
{
  int i;
  signed short *s;
  //bmaregs[ACCZLSB] = 0;
  //while ((bmaregs[ACCZLSB] & 1) == 0)
    MasterI2C0Read(BMA180_SLAVE_ADDRESS,&bmaregs[ACCXLSB],ACCXLSB,6); 
  #if 0
  DEBUG_TP6(1); 

  // This code takes 1.7 us
  BMA180values[0] = bmaregs[ACCXMSB] << 8;
  BMA180values[0] |= bmaregs[ACCXLSB];

  BMA180values[1] = bmaregs[ACCYMSB] << 8;
  BMA180values[1] |= bmaregs[ACCYLSB];

  BMA180values[2] = bmaregs[ACCZMSB] << 8;
  BMA180values[2] |= bmaregs[ACCZLSB];
  DEBUG_TP6(0); 
  #endif
  xbmaready = false;
  // data now ready, shift 14 bits out of the 16 bits
  for (i=0, s = (signed short *)&bmaregs[ACCXLSB]; i< 3; i++,s++)
    BMA180values[i] = (*s)>>2;
 }
