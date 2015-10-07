/*
  
  $Id: utilities.c,v 1.1 2011/03/15 12:23:45 $
  $Log: utilities.c,v $
  Revision 1.1  2011/03/15 12:23:45
  First save

  Various utilities
*/

/*
  swap 2 bytes
*/
#include "bluebird.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/sysctl.h"

void swapb(unsigned short *w)
{
  unsigned char *t;
  unsigned char tc;
  t = (unsigned char *)w;
  tc = t[0];
  t[0] = t[1];
  t[1] = tc;
}

/*
  SysCtlDelay takes 3 instructions per loop
  Convert desired delay to delay loops
*/
void microsecdelay(unsigned long delay)
{
  SysCtlDelay((3*delay)/(SysCtlClockGet()/1000));
}
