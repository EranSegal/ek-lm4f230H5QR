/*
  Copyright BlueBird Aero Systaems Ltd 2011

  $Id: MAG31110.h,v 1.3 2011/04/10 10:37:29 EranS Exp $
   $Log: MAG3110.h,v $

  Revision 1.1  2011/03/15 12:23:45  EranS
  First save

*/

#ifndef _MAG3110_H_
#define _MAG3110_H_

typedef signed char VINT8;
typedef unsigned char VUINT8;
typedef signed short VINT16;
typedef unsigned short VUINT16;
typedef  signed long VINT32;
typedef  unsigned long VUINT32;

typedef struct {
  VINT16  X;
  VINT16  Y;
  VINT16  Z;  
} SPARAMETERS;

typedef union {
  VINT16 s16word;
  struct {
    VINT8 s8msb;
    VUINT8 u8lsb;
  } Bytes;
} s16DataUnion;

#define MAG3110_ADD   0x0E    //Please contact the factory to request a different IIC address
#define MAG3110_SLAVE_ADDRESS 0xE

// IIC Register Address
#define MAG3110_DR_STATUS 0x00
#define MAG3110_OUT_X_MSB 0x01
#define MAG3110_OUT_X_LSB 0x02
#define MAG3110_OUT_Y_MSB 0x03
#define MAG3110_OUT_Y_LSB 0x04
#define MAG3110_OUT_Z_MSB 0x05
#define MAG3110_OUT_Z_LSB 0x06
#define MAG3110_WHO_AM_I  0x07
#define MAG3110_SYSMOD    0x08
#define MAG3110_OFF_X_MSB 0x09
#define MAG3110_OFF_X_LSB 0x0A
#define MAG3110_OFF_Y_MSB 0x0B
#define MAG3110_OFF_Y_LSB 0x0C
#define MAG3110_OFF_Z_MSB 0x0D
#define MAG3110_OFF_Z_LSB 0x0E
#define MAG3110_DIE_TEMP  0x0F
#define MAG3110_CTRL_REG1 0x10
#define MAG3110_CTRL_REG2 0x11


#define MAG_FilterStrength 4
#define MAG_RSDL_POS 100
#define MAG_RSDL_NEG -100

void MAG3110_XYZ_Read_and_Filter(void);
VUINT8 MAG3110_Init(void);
void IIC_Read_MAG3110_XYZ16(VINT16 *pX, VINT16 *pY, VINT16 *pZ);
VUINT8 IIC_Read_MAG3110_XYZ8(VINT8 *pX, VINT8 *pY, VINT8 *pZ);

// prototypes
void MAG3110Init();
void MAG3110Action(void);
void Read_MAG3110_XYZ16(void);
void MAG3110getXYZ();
void MAG3110getX(void);
int getHeading(float x, float y, float z);

#define MAG3110_SAMPLE_RATE 200
#define DATA_READY 0x1
#define MGA_MGM 17
#define MAG_NUM_SAMPLES 100

#ifdef MAG_MAIN
unsigned char magregs[MGA_MGM+1];
tBoolean magready = false;
signed short MAG3110values[3];
signed short MAG3110AtRest[3];
#else
extern unsigned char magregs[];
extern unsigned char magready;
extern signed short MAG3110values[];
extern signed short MAG3110AtRest[];
#endif

#define X_MAG3110_REST MAG3110AtRest[0]
#define Y_MAG3110_REST MAG3110AtRest[1]
#define Z_MAG3110_REST MAG3110AtRest[2]

#define FULL_SCALE_RANGE 2000.0f   // degrees/sec

#endif

