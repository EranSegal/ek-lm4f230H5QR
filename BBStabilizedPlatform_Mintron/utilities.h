/*
  Copyright Bluebird Aero Systems Engineering 2012

  $Id:%
  %Log:%

  Various utilities
*/

/*
  swap 2 bytes
*/
void swapb(unsigned short *w);
void microsecdelay(unsigned long delay);
#define max(a,b) ((a > b) ? a : b)
#define min(a,b) ((a < b) ? a : b)

// useful numbers
#define TWO_POWER_4  0x0010
#define TWO_POWER_8  0x0100
#define TWO_POWER_11 0x0800
#define TWO_POWER_12 0x1000
#define TWO_POWER_13 0x2000
#define TWO_POWER_15 0x8000
#define TWO_POWER_16 0x10000
