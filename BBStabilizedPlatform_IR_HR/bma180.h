/*
  Copyright Bluebird Aero Systems Engineering 2011

  $Id: bma180.h,v 1.2 2011/03/22 15:42:49 EranS Exp $
  $Log: bma180.h,v $
  Revision 1.2  2011/03/22 15:42:49  EranS
  added telemetry
  fixed adc code

  Revision 1.1  2011/03/15 12:23:45  EranS
  First save

 
 */

#ifndef BMA180_H_
#define BMA180_H_

// prototypes
void BMA180Init();
void BMA180Action(void);
void BMA180GetXYZ(void);

// Address defines for BMA180//
// all these values are in a JPG on p21 of the datasheet i.e. cannot be searched for!!
//====================//
//ID and Version Registers
#define CHIP_ID 0x00
#define Version 0x01
#define ACCXLSB 0x02
#define ACCXMSB 0x03
#define ACCYLSB 0x04
#define ACCYMSB 0x05
#define ACCZLSB 0x06
#define ACCZMSB 0x07
#define TEMPERATURE 0x08
#define STATREG1 0x09
  #define EE_WRITE 1
  #define ALERT 0x20
  #define STR 0x40
#define STATREG2 0x0A
#define STATREG3 0x0B
#define STATREG4 0x0C
#define CTRLREG0 0x0D
  #define ST0 0x04
  #define ST1 0x08
  #define EE_W 0x10
  #define RESET_INT 0x40
  #define ST_DAMP 0x80
#define CTRLREG1 0x0E
  #define EN_OFFSET_X 0x80  // in ctrl_reg1
  #define EN_OFFSET_Y 0x40  
  #define EN_OFFSET_Z 0x20  
#define CTRLREG2 0x0F
#define SOFT_RESET 0x10
  #define MAGIC_NUMBER 0xb6
// 0x11 to 0x1f reserved
// to write to 0x20 - 0x3b need to undo write protect, set EE_W in CTRLREG0
#define BWTCS 0x20
  #define BWMASK 0xf0
  #define BWSHIFT 4
  #define BW10HZ 0
  #define BW20HZ 1
  #define BW40HZ 2
  #define BW75HZ 3
  #define BW150HZ 4
  #define BW300HZ 5
  #define BW600HZ 6
  #define BW1200HZ 7
#define CTRLREG3 0x21
  #define LAT_INT 1
  #define NEW_DATA_INT 2
  #define ADV_INT 4
  #define TAPSENS_INT 8
  #define LOW_INT 0x10
  #define HIGH_INT 0x20
  #define SLOPE_INT 0X40
  #define  SLOPE_ALERT 0X80
#define CTRLREG4 0x22
  #define FINE_TUNING_MASK 0x3
  #define NO_FINE_TUNING 0
  #define FINE_CALIBRATION 1
  #define COARSE_CALIBRATION 2
  #define FULL_CALIBRATION 3
#define HY 0x23
#define SLOPE_TAPSENS_INFO 0x24
#define HI_LOW_INFO 0x25
  #define LOW_FILT 1
  #define LOW_INT_Z 2
  #define LOW_INT_Y 4
  #define LOW_INT_X 8
  #define HIGH_FILT 0X10
  #define HIGH_INT_Z 0x20
  #define HIGH_INT_Y 0x40
  #define HIGH_INT_X 0x80
#define LOW_DUR 0x26
#define HIGH_DUR 0x27
#define TAPSENS_TH 0x28
#define LOW_TH 0x29
#define HIGH_TH 0x2a
#define SLOPE_TH 0x2b
#define CD1 0x2c
#define CD2 0x2d
#define TCO_X 0x2e
#define TCO_Y 0x2F
#define TCO_Z 0x30
  #define MODE_CONFIG_0 0
  #define MODE_CONFIG_1 1
  #define MODE_CONFIG_2 2
  #define MODE_CONFIG_3 3
  #define MODE_MASK   0xfc
#define GAIN_T 0x31
#define GAIN_X 0x32
#define GAIN_Y 0x33
#define GAIN_Z 0x34
#define OLSB1 0x35
  //Range setting
  #define RANGESHIFT 1
  #define RANGEMASK 0x0E
  #define RANGE1 0
  #define RANGE15 1
  #define RANGE2 2
  #define RANGE3 3
  #define RANGE4 4
  #define RANGE8 5
  #define RANGE16 6
#define OLSB2 0x36
#define OFFSET_T 0x37
#define OFFSET_X 0x38
#define OFFSET_Y 0x39
#define OFFSET_Z 0x3a

#define EE_OFFSET_LSB1 0x55
#define EE_OFFSET_LSB2 0x56
#define EE_OFFSET_T 0x57
#define EE_OFFSET_X_MSB 0x58
#define EE_OFFSET_Y_MSB 0x59
#define EE_OFFSET_Z_MSB 0x5A
//====================//

#define BMA180_TEMP_SCALE 2
#define BMA180_TEMP_ZERO 24

#define USE_BMA180_INTERRUPT
//#define USE_BMA180_LATCH

#define BMA180_SLAVE_ADDRESS 0x40

#ifdef BMA180_MAIN
unsigned char bmaregs[EE_OFFSET_Z_MSB+1];
signed short BMA180values[3];
tBoolean xbmaready = false;
#else
extern unsigned char bmaregs[];
extern signed short BMA180values[];
extern unsigned char xbmaready;
#endif

#endif /* BMA180_H_ */
