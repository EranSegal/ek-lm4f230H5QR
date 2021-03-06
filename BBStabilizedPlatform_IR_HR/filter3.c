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

short int Measurements[9];
tBoolean Init;

// DO NOT change order of next 9 lines
// accelerometer
const float cAx= -0.585   ,cAxx= -1.0/548.0 ,cAxy= 0        ,cAxz= 0;
const float cAy= 0.3      ,cAyx= 0          ,cAyy= 1.0/558.0,cAyz= 0;
const float cAz= -0.5624  ,cAzx= 0          ,cAzy= 0        ,cAzz= 1.0/567.0;

// DO NOT change order of next 9 lines
// Eran accelerometer
//float cAx= -0.585   ,cAxx= -1.0/558.0 ,cAxy= 0        ,cAxz= 0;
//float cAy= 0.3      ,cAyx= 0          ,cAyy= 1.0/558.0,cAyz= 0;
//float cAz= -0.1  ,cAzx= 0          ,cAzy= 0        ,cAzz= 1.0/567.0;

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
//  Ax=cAx+cAxx*Axm+cAxy*Aym+cAxz*Azm; //Ax = -0.585   +  (-1.0/548.0*Axm)
//  Ay=cAy+cAyx*Axm+cAyy*Aym+cAyz*Azm; //Ay = 0.3 	    +  (1.0/558.0*Aym)
  Az=cAz+cAzx*Axm+cAzy*Aym+cAzz*Azm; //Az  = -0.5624 +  (1.0/567.0*Azm)
  //Calibration Accelerometerx
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
  
  cThetac=sqrtf(1-sThetac*sThetac); // cosa = square root( 1 - sin�alfa)

  // calculate roll motion  
  sPhic     =  ((Ay/cThetac)/g);   //DH

  Phic   = asinf(sPhic);
  Thetac = asinf(sThetac);
  return;   // just Phic instead of Phi etc.
}

#else
/*
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

void filterUpdate(float w_x, float w_y, float w_z, float a_x, float a_y, float a_z);

// System constants
//#define deltat 0.001f // sampling period in seconds (shown as 1 ms)



#define gyroMeasError 3.14159265358979f * (5.0f / 180.0f) // gyroscope measurement error in rad/s (shown as 5 deg/s)
#define beta sqrt(3.0f / 4.0f) * gyroMeasError // compute beta
// Global system variables
//float a_x, a_y, a_z; // accelerometer measurements
//float w_x, w_y, w_z; // gyroscope measurements in rad/s
float SEq_1 = 1.0f, SEq_2 = 0.0f, SEq_3 = 0.0f, SEq_4 = 0.0f; // estimated orientation quaternion elements with initial conditions


short int Measurements[9];
tBoolean Init;

float Ax,Ay,Az,Axm,Aym,Azm;
float Gx,Gy,Gz,Gxm,Gym,Gzm;
float sThetac,cThetac;
float sPhic;
float Rx,Ry,Rz;
float g,dt,Eps,Gain;
float Tau = 0.5;

// DO NOT change order of next 9 lines
// accelerometer
const float cAx= -0.585   ,cAxx= -1.0/548.0 ,cAxy= 0        ,cAxz= 0;
const float cAy= 0.3      ,cAyx= 0          ,cAyy= 1.0/558.0,cAyz= 0;
const float cAz= -0.5624  ,cAzx= 0          ,cAzy= 0        ,cAzz= 1.0/567.0;

// gyro
const float cGx= 0  ,cGxx= 0    ,cGxy= 0.25 ,cGxz= 0;
const float cGy= 0  ,cGyx= 0.25 ,cGyy= 0    ,cGyz= 0;
const float cGz= 0  ,cGzx= 0    ,cGzy= 0    ,cGzz= -0.25;

void AHRS_Init()
{
	dt= (1.0/ITG3200_SAMPLE_RATE); 
	Rx=0;
	Ry=0;
	Rz=0;
	Phi=0;
	Theta=0;
	Psi=0;
    Init = true;
}

void AHRS()
{
    memcpy(&Measurements[0],BMA180values,sizeof(short int)*3);
    memcpy(&Measurements[3],ITG3200values,sizeof(short int)*3);
    AHRS_Step_c();
}

void AHRS_Step_c() // Attitude heading reference system 
{
  // correlation between x/y/z axis and index in sensor readings
  Axm= (float)BMA180values[0];
  Aym= (float)BMA180values[1];
  Azm= (float)BMA180values[2];

  Gxm= (float)Measurements[3+1]-ITG3200AtRest[1];
  Gym= (float)Measurements[3+0]-ITG3200AtRest[0];
  Gzm= (float)Measurements[3+2]-ITG3200AtRest[2];

  filterUpdate(Axm,Aym,Azm,Gxm,Gym,Gzm);
  //Calibration Accelerometerx
  Ax=cAx+cAxx*Axm+cAxy*Aym+cAxz*Azm; //Ax = -0.585   +  (-1.0/548.0*Axm)
  Ay=cAy+cAyx*Axm+cAyy*Aym+cAyz*Azm; //Ay = 0.3 	    +  (1.0/558.0*Aym)
  Az=cAz+cAzx*Axm+cAzy*Aym+cAzz*Azm; //Az  = -0.5624 +  (1.0/567.0*Azm)
  Gx=cGx+cGxx*Gxm+cGxy*Gym+cGxz*Gzm;
  Gy=cGy+cGyx*Gxm+cGyy*Gym+cGyz*Gzm;
  Gz=cGz+cGzx*Gxm+cGzy*Gym+cGzz*Gzm;

  if(Init)
  {
    g=9.8f;
    Eps=0.01f;
    Gain=1-expf(-dt/Tau);
    Theta=-asinf(Ax/g);
    Phi=asinf((Ay/cosf(Theta))/g);  // DH
    Psi = 0;
    Init = false;
  }
  
  //Acceleration & Magnetometer Euler Angles
  g = sqrt(Ax * Ax + Ay * Ay + Az * Az) + 0.0001; // Add 0.0001 to g is not to  get Non while we 
  sThetac = -(Ax/g);      
  cThetac=sqrtf(1-sThetac*sThetac);
  
  sPhic     =  ((Ay/cThetac)/g);   //DH

  Phic   = asinf(sPhic);
  Thetac = asinf(sThetac);
  return;   // just Phic instead of Phi etc.
}


void filterUpdate(float w_x, float w_y, float w_z, float a_x, float a_y, float a_z)
{
	// Local system variables
	float norm; // vector norm
	float SEqDot_omega_1, SEqDot_omega_2, SEqDot_omega_3, SEqDot_omega_4; // quaternion derrivative from gyroscopes elements
	float f_1, f_2, f_3; // objective function elements
	float J_11or24, J_12or23, J_13or22, J_14or21, J_32, J_33; // objective function Jacobian elements
	float SEqHatDot_1, SEqHatDot_2, SEqHatDot_3, SEqHatDot_4; // estimated direction of the gyroscope error
	
	// Axulirary variables to avoid reapeated calcualtions
	float halfSEq_1 = 0.5f * SEq_1;
	float halfSEq_2 = 0.5f * SEq_2;
	float halfSEq_3 = 0.5f * SEq_3;
	float halfSEq_4 = 0.5f * SEq_4;
	float twoSEq_1 = 2.0f * SEq_1;
	float twoSEq_2 = 2.0f * SEq_2;
	float twoSEq_3 = 2.0f * SEq_3;

	// Normalise the accelerometer measurement
	norm = sqrt(a_x * a_x + a_y * a_y + a_z * a_z);
	a_x /= norm;
	a_y /= norm;
	a_z /= norm;
	
	// Compute the objective function and Jacobian
	f_1 = twoSEq_2 * SEq_4 - twoSEq_1 * SEq_3 - a_x;
	f_2 = twoSEq_1 * SEq_2 + twoSEq_3 * SEq_4 - a_y;
	f_3 = 1.0f - twoSEq_2 * SEq_2 - twoSEq_3 * SEq_3 - a_z;
	J_11or24 = twoSEq_3; // J_11 negated in matrix multiplication
	J_12or23 = 2.0f * SEq_4;
	J_13or22 = twoSEq_1; // J_12 negated in matrix multiplication
	J_14or21 = twoSEq_2;
	J_32 = 2.0f * J_14or21; // negated in matrix multiplication
	J_33 = 2.0f * J_11or24; // negated in matrix multiplication
	
	// Compute the gradient (matrix multiplication)
	SEqHatDot_1 = J_14or21 * f_2 - J_11or24 * f_1;
	SEqHatDot_2 = J_12or23 * f_1 + J_13or22 * f_2 - J_32 * f_3;
	SEqHatDot_3 = J_12or23 * f_2 - J_33 * f_3 - J_13or22 * f_1;
	SEqHatDot_4 = J_14or21 * f_1 + J_11or24 * f_2;
	
	// Normalise the gradient
	norm = sqrt(SEqHatDot_1 * SEqHatDot_1 + SEqHatDot_2 * SEqHatDot_2 + SEqHatDot_3 * SEqHatDot_3 + SEqHatDot_4 * SEqHatDot_4);
	SEqHatDot_1 /= norm;
	SEqHatDot_2 /= norm;
	SEqHatDot_3 /= norm;
	SEqHatDot_4 /= norm;
	// Compute the quaternion derrivative measured by gyroscopes
	SEqDot_omega_1 = -halfSEq_2 * w_x - halfSEq_3 * w_y - halfSEq_4 * w_z;
	SEqDot_omega_2 = halfSEq_1 * w_x + halfSEq_3 * w_z - halfSEq_4 * w_y;
	SEqDot_omega_3 = halfSEq_1 * w_y - halfSEq_2 * w_z + halfSEq_4 * w_x;
	SEqDot_omega_4 = halfSEq_1 * w_z + halfSEq_2 * w_y - halfSEq_3 * w_x;
	// Compute then integrate the estimated quaternion derrivative
	SEq_1 += (SEqDot_omega_1 - (beta * SEqHatDot_1)) * dt;
	SEq_2 += (SEqDot_omega_2 - (beta * SEqHatDot_2)) * dt;
	SEq_3 += (SEqDot_omega_3 - (beta * SEqHatDot_3)) * dt;
	SEq_4 += (SEqDot_omega_4 - (beta * SEqHatDot_4)) * dt;
	// Normalise quaternion
	norm = sqrt(SEq_1 * SEq_1 + SEq_2 * SEq_2 + SEq_3 * SEq_3 + SEq_4 * SEq_4);
	SEq_1 /= norm;
	SEq_2 /= norm;
	SEq_3 /= norm;
	SEq_4 /= norm;
}



#endif
