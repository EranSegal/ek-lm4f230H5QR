/*
  Copyright Bluebird Aero Systems Engineering 2012

  $Id: Stabilize.h,v 1.1 2011/04/06 07:20:51 EranS Exp $
  $Log: Stabilize.h,v $
  Revision 1.1  2011/04/06 07:20:51  EranS
  Stabilize moved to separate file

*/

// get gyro values at rest by averaging out 200 readings
#define NUM_SAMPLES 200

#define X_AXIS 2  // gyro readings
#define Y_AXIS 1
#define Z_AXIS 0

void Stabilize(enum MOTOR motor);
void StabilizeInit();


