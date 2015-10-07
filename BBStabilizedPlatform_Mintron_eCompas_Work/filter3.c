#if 1
/*
  Copyright Bluebird Aero Systems Engineering 2012

  $Id: filter3.c,v 1.5 2011/07/14 07:19:48 EranS Exp $
  $Log: filter3.c,v $

  Revision 1.6  2012/02/11 07:19:48  EranS
  added fix phi angle
  addad fix Thetac and Phic if <-1 or >1 
  Rem unuse line
  
  Revision 1.5  2011/07/14 07:19:48  EranS
  added script engine

  Revision 1.4  2011/04/17 08:44:11  EranS
  add proper reset of encoders via GPIO

  Revision 1.3  2011/04/12 15:03:35  EranS
  added joystick interface

  Revision 1.2  2011/04/12 05:50:35  EranS
  calibrated gimbal / gyro / accelerometer

  Revision 1.1  2011/04/10 10:37:29  EranS
  Add filer3.c and AHRS from Lumus project


*/


#include "inc/hw_types.h"
#include <math.h>
#include <string.h>
#include "bluebird.h"
#include "ITG3200.h"
#include "BMA180.h"
#define FILTER_MAIN
#include "filter3.h"
#define S1 -1.0f
#define S2 1.0f

#define NOSE

float Ax,Ay,Az,Axm,Aym,Azm;
float AAx,AAy,AAz;
float Theta,dTheta,Thetag,sThetac,cThetac;
float sPhi,cPhi,sPhic;
float Gx,Gy,Gz,Gxm,Gym,Gzm;
float dt,g;

float vx,vy,alfa,vfov;   
float yDegRoll = 0;
float hx,hy,alfa,hfov;   
float yDegPitch = 0;
float LandDistance = 0;



short int Measurements[9];
tBoolean Init;

// DO NOT change order of next 9 lines
// accelerometer
const float cAx= -0.585   ,cAxx= -1.0/548.0 ,cAxy= 0        ,cAxz= 0;
const float cAy= 0.3      ,cAyx= 0          ,cAyy= 1.0/558.0,cAyz= 0;
const float cAz= -0.5624  ,cAzx= 0          ,cAzy= 0        ,cAzz= 1.0/567.0;


void AHRS_Init()
{
	//dt= (1.0/ITG3200_SAMPLE_RATE); // 0.005
	Phi=0;
	Theta=0;
    Init = true;
}

void AHRS()
{
    memcpy(&Measurements[0],BMA180values,sizeof(short int)*3);
    AHRS_Step_c();
}


void AHRS_Step_c()
{
  // correlation between x/y/z axis and index in sensor readings
  Axm= (float)Measurements[0];
  Aym= (float)Measurements[1];
  Azm= (float)Measurements[2];

  #ifdef NOSE
  //Calibration Accelerometerx

  Az=cAz+cAzx*Axm+cAzy*Aym+cAzz*Azm; //Az  = -0.5624 +  (1.0/567.0*Azm)
  AAx=cAx+cAxx*Axm+cAxy*Aym+cAxz*Azm; //Ax = -0.585   +  (-1.0/548.0*Axm)
  AAy=cAy+cAyx*Axm+cAyy*Aym+cAyz*Azm; //Ay = 0.3 	    +  (1.0/558.0*Aym)

  Ax = -AAy;
  Ay = AAx;
  
  #else
  //Calibration Accelerometerx
  AAx=cAx+cAxx*Axm+cAxy*Aym+cAxz*Azm; //Ax = -0.585   +  (-1.0/548.0*Axm)
  AAy=cAy+cAyx*Axm+cAyy*Aym+cAyz*Azm; //Ay = 0.3 	    +  (1.0/558.0*Aym)
  AAz=cAz+cAzx*Axm+cAzy*Aym+cAzz*Azm; //Az  = -0.5624 +  (1.0/567.0*Azm)

  Az = AAx;
  Ax = -AAz;
  #endif

  //Acceleration & Magnetometer Euler Angles
  g = sqrt(Ax * Ax + Ay * Ay + Az * Az) + 0.0001; // Add 0.0001 to g is not to  get Non while we 
  // calculate pitch motion
  sThetac = -(Ax/g);  // sin alfa     
  cThetac=sqrtf(1-sThetac*sThetac); // cosa = square root( 1 - sin²alfa)
  
  // calculate roll motion  
  sPhic     =  ((Ay/cThetac)/g);   //DH

  Phic   = asinf(sPhic);
  Thetac = asinf(sThetac);

  return;   // just Phic instead of Phi etc.
}

#else

/*
  $Id: filter3.c,v 1.5 2011/07/14 07:19:48 davidh Exp $
  $Log: filter3.c,v $
  Revision 1.5  2011/07/14 07:19:48  davidh
  added script engine

  Revision 1.4  2011/04/17 08:44:11  davidh
  add proper reset of encoders via GPIO

  Revision 1.3  2011/04/12 15:03:35  davidh
  added joystick interface

  Revision 1.2  2011/04/12 05:50:35  davidh
  calibrated gimbal / gyro / accelerometer

  Revision 1.1  2011/04/10 10:37:29  davidh
  Add filer3.c and AHRS from Lumus project


*/

//#define BLUEBIRD  // means no magnetometer

#include "inc/hw_types.h"
#include <math.h>
#include <string.h>

#include "ITG3200.h"
#include "BMA180.h"
#include "MAG3110.h"

#define FILTER_MAIN
#include "filter3.h"
#define S1 -1.0f
#define S2 1.0f

#define STANDALONE

float Ax,Ry,dTheta,Axm,Rz,dt;
float AAx,AAy,AAz;
float Ay,g,Aym;
float Tau = 0.5;
float Az,Azm,Thetag;
#ifndef BLUEBIRD
float Mx,Mxm,My,Mym,Mz,Mzm;
#endif
float Eps,Thetaz,sPhi;
float Gain,cPhi,sPhic;
float Gx,Phig,cPhic,sPhig;
float Gxm,Phiz,cPhig,sPsi;
float Gy,cPsi,sPsic;
float Gym,Psic,cPsic,sPsig;
float Gz,Psig,cPsig,sTheta;
float Gzm,Psiz,cTheta,sThetac;
float RWx,cThetac,sThetag;
float RWy,cThetag;
float Mix,RWz,dPhi;
float Miy,Rx,dPsi;

#ifndef BLUEBIRD
short int Measurements[9];
#else
short int Measurements[6];
#endif
tBoolean Init;
float rotc[3][3],rote[3][3],rotg[3][3];
// DO NOT change order of next 9 lines
// accelerometer
const float cAx= -0.585   ,cAxx= -1.0/548.0 ,cAxy= 0        ,cAxz= 0;
const float cAy= 0.3      ,cAyx= 0          ,cAyy= 1.0/558.0,cAyz= 0;
const float cAz= -0.5624  ,cAzx= 0          ,cAzy= 0        ,cAzz= 1.0/567.0;
// gyro
const float cGx= 0  ,cGxx= 0    ,cGxy= 0.25 ,cGxz= 0;
const float cGy= 0  ,cGyx= 0.25 ,cGyy= 0    ,cGyz= 0;
const float cGz= 0  ,cGzx= 0    ,cGzy= 0    ,cGzz= -0.25;

#if 0

#ifndef BLUEBIRD
float cMx=  -2.31756e+001 , cMxx=   7.37373e-001 , cMxy=   4.95486e-002 , cMxz=   1.44887e-002;
float cMy=   6.14254e+001 , cMyx=  -5.22259e-002 , cMyy=   7.85282e-001 , cMyz=  -3.01566e-002;
float cMz=   4.12797e+001 , cMzx=  -9.73106e-003 , cMzy=  -3.64315e-003 , cMzz=   6.76591e-001;
#endif
#else
float cMx=  0 , cMxx=  0 ,  cMxy=  1 , cMxz=   0;
float cMy=  0 , cMyx=  1 ,  cMyy=  0 , cMyz=   0;
float cMz=  0 , cMzx=  0 ,  cMzy=  0 , cMzz=   1.0;

#endif

void AHRS_Init()
{
//	Tau= 1;   // initialized above
	dt= (1.0/ITG3200_SAMPLE_RATE); //0.02;  // this should be taken from ITG3200 rate
	Rx=0;
	Ry=0;
	Rz=0;
	Phi=0;
	Theta=0;
	Psi=0;
    Init = 1;
}


void AHRS()
{
    memcpy(&Measurements[0],BMA180values,sizeof(short int)*3);
    memcpy(&Measurements[3],ITG3200values,sizeof(short int)*3);
#ifndef BLUEBIRD
    memcpy(&Measurements[6],MAG3110values,sizeof(short int)*3);
#endif
    AHRS_Step_c();
}

void AHRS_Step_c()
{
//[Psi,Theta,Phi]=AHRS_Step_c(Psi,Theta,Phi,Measurements,Rx,Ry,Rz,dt,Tau,Init);
//MyDebug=[Psig,Thetag,Phig,Psic,Thetac,Phic,Ax,Ay,Az,Gx,Gy,Gz,Mx,My,Mz];
//Written by Robert Zickel 2-Jan-2011 Based on Zeev Berman Algorithm

//persistent g Gain Eps Rx Ry Rz dt Tau
//Measurements
Axm= (float)Measurements[0];
Aym= (float)Measurements[1];
Azm= (float)Measurements[2];

Gxm= (float)Measurements[3];
Gym= (float)Measurements[4];
Gzm= (float)Measurements[5];

Mxm= (float)Measurements[6];
Mym= (float)Measurements[7];
Mzm= (float)Measurements[8];

//Calibration
//Ax=cAx+cAxx*Axm+cAxy*Aym+cAxz*Azm;
//Ay=cAy+cAyx*Axm+cAyy*Aym+cAyz*Azm;
//Az=cAz+cAzx*Axm+cAzy*Aym+cAzz*Azm;

//Calibration Accelerometerx
AAx=cAx+cAxx*Axm+cAxy*Aym+cAxz*Azm; //Ax = -0.585   +  (-1.0/548.0*Axm)
AAy=cAy+cAyx*Axm+cAyy*Aym+cAyz*Azm; //Ay = 0.3 	    +  (1.0/558.0*Aym)
Az=cAz+cAzx*Axm+cAzy*Aym+cAzz*Azm; //Az  = -0.5624 +  (1.0/567.0*Azm)
Ax = -AAy;
Ay = AAx;
  
Gx=cGx+cGxx*Gxm+cGxy*Gym+cGxz*Gzm;
Gy=cGy+cGyx*Gxm+cGyy*Gym+cGyz*Gzm;
Gz=cGz+cGzx*Gxm+cGzy*Gym+cGzz*Gzm;

Mx=cMx+cMxx*Mxm+cMxy*Mym+cMxz*Mzm;
My=cMy+cMyx*Mxm+cMyy*Mym+cMyz*Mzm;
Mz=cMz+cMzx*Mxm+cMzy*Mym+cMzz*Mzm;

if(Init==1)
{
    g=9.8f;
    Eps=0.01f;
    Gain=1-expf(-dt/Tau);
    Theta=-asinf(Ax/g);
    Phi=asinf((Ay/cosf(Theta))/g);  // DH
    Psi=atan2f(-My,Mx);
#ifndef STANDALONE
    if (isnan(Psi) || isnan(Phi) || isnan(Theta))
      markerror(2,__LINE__,FILTER_SOURCE);
#endif
    Init = 0;
}

//Auxillary calculations
  cPhi   = cosf(Phi);
  sPhi   = sinf(Phi);
  cTheta = cosf(Theta);
  sTheta = sinf(Theta);
  cPsi   = cosf(Psi);
  sPsi   = sinf(Psi);
#ifndef STANDALONE
  if (isnan(cPsi) || isnan(cPhi) || isnan(cTheta) || isnan(cPsi) || isnan(cPhi) || isnan(cTheta))
      markerror(3,__LINE__,FILTER_SOURCE);
#endif

//Euler Rate Integration
  if(fabs(cTheta)<Eps)
  {
    cTheta = Eps;
  }
  dPhi   = (Gx+(Gy*sPhi+Gz*cPhi)*sTheta/cTheta)*dt;
  dTheta = (Gy*cPhi-Gz*sPhi)*dt;
  dPsi   = (Gy*sPhi+Gz*cPhi)/cTheta*dt;
  Phig   = Phi+dPhi;
  Thetag = Theta+dTheta;
  Psig   = Psi+dPsi;
#ifndef STANDALONE
  if (isnan(dPsi) || isnan(dPhi) || isnan(dTheta) || isnan(Psig) || isnan(Phig) || isnan(Thetag))
      markerror(3,__LINE__,FILTER_SOURCE);
#endif

//Lever Arm Correction
  RWx=Gy*Rz-Gz*Ry;
  RWy=Gz*Rx-Gx*Rz;
  RWz=Gx*Ry-Gy*Rx;
  Ax=Ax-(Gy*RWz-Gz*RWy);
  Ay=Ay-(Gz*RWx-Gx*RWz);
  Az=Az-(Gx*RWy-Gy*RWx);

//Acceleration & Magnetometer Euler Angles
  g = sqrt(Ax * Ax + Ay * Ay + Az * Az);
  sThetac = -(Ax/g);
  cThetac=sqrtf(1-sThetac*sThetac);
  sPhic     =  ((Ay/cThetac)/g);   //DH
//sPhic     =  (Ay/sqrt(Ay^2+Az^2));
  cPhic=sqrtf(1-sPhic*sPhic);

//Inertial  componennts of Magnetic field
   Mix= cThetac*Mx +sPhic*sThetac*My+cPhic*sThetac*Mz;
   Miy=                     cPhic*My        -sPhic*Mz;
//Miz= -sThetac*Mx+cThetac*sPhic*My+cPhic*cThetac*Mz;
  Phic   = asinf(sPhic);
  Thetac = asinf(sThetac);
  Psic   = atan2f(-Miy,Mix); //0.072723=4.1667*pi/180
//rotc=B2I(Phic, Thetac, Psic);
  cPsic = cosf(Psic);
  sPsic = sinf(Psic);
  rotc[0][0] = cPsic*cThetac;
  rotc[0][1] = cPsic*sPhic*sThetac - cPhic*sPsic;
  rotc[0][2] = sPhic*sPsic + cPhic*cPsic*sThetac;
  rotc[1][0] = cThetac*sPsic;
  rotc[1][1] = cPhic*cPsic + sPhic*sPsic*sThetac;
  rotc[1][2] = cPhic*sPsic*sThetac - cPsic*sPhic;
  rotc[2][0] = -sThetac;
  rotc[2][1] = cThetac*sPhic;
  rotc[2][2] = cPhic*cThetac;

//rotg=B2I(Phig, Thetag, Psig);
  cPhig   = cosf(Phig);
  sPhig   = sinf(Phig);
  cThetag = cosf(Thetag);
  sThetag = sinf(Thetag);
  cPsig   = cosf(Psig);
  sPsig   = sinf(Psig);
  rotg[0][0] = cPsig*cThetag;
  rotg[0][1] = cPsig*sPhig*sThetag - cPhig*sPsig;
  rotg[0][2] = sPhig*sPsig + cPhig*cPsig*sThetag;
  rotg[1][0] = cThetag*sPsig;
  rotg[1][1] = cPhig*cPsig + sPhig*sPsig*sThetag;
  rotg[1][2] = cPhig*sPsig*sThetag - cPsig*sPhig;
  rotg[2][0] = -sThetag;
  rotg[2][1] = cThetag*sPhig;
  rotg[2][2] = cPhig*cThetag;

//rote=rotc'*rotg;
//rote[0][0]=rotc[0][0]*rotg[0][0] + rotc[1][0]*rotg[1][0] + rotc[2][0]*rotg[2][0];
   rote[0][1]=rotc[0][0]*rotg[0][1] + rotc[1][0]*rotg[1][1] + rotc[2][0]*rotg[2][1];
   rote[0][2]=rotc[0][0]*rotg[0][2] + rotc[1][0]*rotg[1][2] + rotc[2][0]*rotg[2][2];
   rote[1][0]=rotc[0][1]*rotg[0][0] + rotc[1][1]*rotg[1][0] + rotc[2][1]*rotg[2][0];
//rote[1][1]=rotc[0][1]*rotg[0][1] + rotc[1][1]*rotg[1][1] + rotc[2][1]*rotg[2][1];
   rote[1][2]=rotc[0][1]*rotg[0][2] + rotc[1][1]*rotg[1][2] + rotc[2][1]*rotg[2][2];
   rote[2][0]=rotc[0][2]*rotg[0][0] + rotc[1][2]*rotg[1][0] + rotc[2][2]*rotg[2][0];
   rote[2][1]=rotc[0][2]*rotg[0][1] + rotc[1][2]*rotg[1][1] + rotc[2][2]*rotg[2][1];
//rote[2][2]=rotc[0][2]*rotg[0][2] + rotc[1][2]*rotg[1][2] + rotc[2][2]*rotg[2][2];
//Filtered Angles
//     0    Psi   -Tet
//   -Psi    0     Phi
//    Tet  -Phi     0
  Phiz  =(rote[1][2]-rote[2][1])*0.5f;
  Thetaz=(rote[2][0]-rote[0][2])*0.5f;
  Psiz  =(rote[0][1]-rote[1][0])*0.5f;
#ifndef STANDALONE
  if (isnan(Psiz) || isnan(Phiz) || isnan(Thetaz))
      markerror(2,__LINE__,FILTER_SOURCE);
#endif
  Phi   = Phig   + Phiz*Gain;
  Theta = Thetag + Thetaz*Gain;
  Psi   = Psig   + Psiz*Gain;
//MyDebug=[Psig,Thetag,Phig,Psic,Thetac,Phic,Ax,Ay,Az,Gx,Gy,Gz,Mx,My,Mz];
}


#endif
