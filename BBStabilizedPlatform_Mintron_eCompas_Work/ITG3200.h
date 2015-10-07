/*
  Copyright Bluebird Aero Systems Engineering 2011

  $Id: ITG3200.h,v 1.3 2011/04/10 10:37:29 EranS Exp $
  $Log: ITG3200.h,v $
  Revision 1.3  2011/04/10 10:37:29  EranS
  Add filer3.c and AHRS from Lumus project

  Revision 1.2  2011/04/06 06:44:20  EranS
  cleaning up, add some constants, move gyro_at_rest code

  Revision 1.1  2011/03/15 12:23:45  EranS
  First save


*/

#define WHO_AM_I 0
#define SMPLRT_DIV 21
#define DLPF_FS 22
#define Hz256 0
#define Hz188 1
#define Hz98  2
#define Hz42  3
#define Hz20  4
#define Hz10  5
#define Hz5   6
#define FS_SEL 0x18
#define INT_CFG 23
#define RAW_RDY_EN 1
// no 2
#define ITG_RDY_EN 4
// no 8
#define INT_ANYRD_2CLEAR 0x10 
#define LATCH_INT_EN 0x20 
#define OPEN 0x40 
#define ACTL 0x80 
#define INT_STATUS 26
#define TEMP_OUT_H 27
#define TEMP_OUT_L 28
#define XOUT_H 29
#define XOUT_L 30
#define YOUT_H 31
#define YOUT_L 32
#define ZOUT_H 33
#define ZOUT_L 34
#define PWR_MGM 62
#define H_RESET 0x80
#define SLEEP   0x40
#define STBY_XG 0x20
#define STBY_YG 0x10
#define STBY_ZG 0x08

#define ITG3200_SLAVE_ADDRESS_A0 0x68   // 7 bit address pin 9 low
#define ITG3200_SLAVE_ADDRESS_A1 0x69   // 7 bit address pin 9 high
#define ITG3200_SLAVE_ADDRESS ITG3200_SLAVE_ADDRESS_A1

// values for converting temperature
#define ITG3200_TEMP_OFFSET 13200
#define ITG3200_TEMP_SCALE 280
#define ITG3200_TEMP_ZERO 35

// prototypes
void ITG3200Init();
void ITG3200Action(void);
void ITG3200getXYZ(float *pfGyroX, float *pfGyroY,float *pfGyroZ);

#define ITG3200_SAMPLE_RATE 8000

#ifdef ITG_MAIN
unsigned char itgregs[PWR_MGM+1];
tBoolean itgready = false;
unsigned char itgreadyN = 0;
signed short ITG3200values[3];
signed short ITG3200AtRest[3];
#else
extern unsigned char itgregs[];
extern unsigned char itgready;
extern unsigned char itgreadyN;
extern signed short ITG3200values[];
extern signed short ITG3200AtRest[];
#endif

#define FULL_SCALE_RANGE 2000.0f   // degrees/sec
