/*
  Copyright Bluebird Aero Systems Engineering 2012

  $Id: filter3.h,v 1.2 2011/04/12 15:03:35 EranS Exp $
  $Log: filter3.h,v $
  Revision 1.2  2011/04/12 15:03:35  EranS
  added joystick interface

  Revision 1.1  2011/04/12 05:50:35  EranS
  calibrated gimbal / gyro / accelerometer

*/

#ifdef FILTER_MAIN
float Psi,Theta,tTheta,Phi,Phic,Thetac;
float vfov,hfov;   
#else
extern float Psi,Theta,tTheta,Phi,Phic,Thetac;
extern float vfov,hfov;   
#endif

void AHRS_Step_c();
void AHRS_Init();
void AHRS();



