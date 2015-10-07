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
  cThetac=sqrtf(1-sThetac*sThetac); // cosa = square root( 1 - sin²alfa)
  
  // calculate roll motion  
  sPhic     =  ((Ay/cThetac)/g);   //DH

  Phic   = asinf(sPhic);
  Thetac = asinf(sThetac);

  #if 0	
  // Vertical	calculation
  vx = 400.0f; // average altitude 400m
  yDegRoll = Phic * RAD2DEG; 
  // LandDistance =tan(alfa)*altitude 
  LandDistance = tanf(yDegRoll) * vx;
  //y = LandDistance/2
  vy = LandDistance/2;
  // FOV° = 2 * tan-1(y/x)
  vfov = 2 * atanf(vy/vx);

  // Horizontal calculation
  hx = 400.0f; // average altitude 400m
  yDegPitch = Thetac * RAD2DEG; 
  // LandDistance =tan(alfa)*altitude 
  LandDistance = tanf(yDegPitch) * hx;
  //y = LandDistance/2
  hy = LandDistance/2;
  // FOV° = 2 * tan-1(y/x)
  hfov = 2 * atanf(hy/hx);
  #endif
  
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

#if 0
/* -*- indent-tabs-mode:T; c-basic-offset:8; tab-width:8; -*- vi: set ts=8:
 * $Id: tilt.c,v 1.1 2003/07/09 18:23:29 john Exp $
 *
 * 1 dimensional tilt sensor using a dual axis accelerometer
 * and single axis angular rate gyro.  The two sensors are fused
 * via a two state Kalman filter, with one state being the angle
 * and the other state being the gyro bias.
 *
 * Gyro bias is automatically tracked by the filter.  This seems
 * like magic.
 *
 * Please note that there are lots of comments in the functions and
 * in blocks before the functions.  Kalman filtering is an already complex
 * subject, made even more so by extensive hand optimizations to the C code
 * that implements the filter.  I've tried to make an effort of explaining
 * the optimizations, but feel free to send mail to the mailing list,
 * autopilot-devel@lists.sf.net, with questions about this code.
 *
 * 
 * (c) 2003 Trammell Hudson <hudson@rotomotion.com>
 *
 *************
 *
 *  This file is part of the autopilot onboard code package.
 *  
 *  Autopilot is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *  
 *  Autopilot is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *  
 *  You should have received a copy of the GNU General Public License
 *  along with Autopilot; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */
#include <math.h>


/*
 * Our update rate.  This is how often our state is updated with
 * gyro rate measurements.  For now, we do it every time an
 * 8 bit counter running at CLK/1024 expires.  You will have to
 * change this value if you update at a different rate.
 */
static const float	dt	= ( 1024.0 * 256.0 ) / 8000000.0;


/*
 * Our covariance matrix.  This is updated at every time step to
 * determine how well the sensors are tracking the actual state.
 */
static float		P[2][2] = {
	{ 1, 0 },
	{ 0, 1 },
};


/*
 * Our two states, the angle and the gyro bias.  As a byproduct of computing
 * the angle, we also have an unbiased angular rate available.   These are
 * read-only to the user of the module.
 */
float			angle;
float			q_bias;
float			rate;


/*
 * R represents the measurement covariance noise.  In this case,
 * it is a 1x1 matrix that says that we expect 0.3 rad jitter
 * from the accelerometer.
 */
static const float	R_angle	= 0.3;


/*
 * Q is a 2x2 matrix that represents the process covariance noise.
 * In this case, it indicates how much we trust the acceleromter
 * relative to the gyros.
 */
static const float	Q_angle	= 0.001;
static const float	Q_gyro	= 0.003;


/*
 * state_update is called every dt with a biased gyro measurement
 * by the user of the module.  It updates the current angle and
 * rate estimate.
 *
 * The pitch gyro measurement should be scaled into real units, but
 * does not need any bias removal.  The filter will track the bias.
 *
 * Our state vector is:
 *
 *	X = [ angle, gyro_bias ]
 *
 * It runs the state estimation forward via the state functions:
 *
 *	Xdot = [ angle_dot, gyro_bias_dot ]
 *
 *	angle_dot	= gyro - gyro_bias
 *	gyro_bias_dot	= 0
 *
 * And updates the covariance matrix via the function:
 *
 *	Pdot = A*P + P*A' + Q
 *
 * A is the Jacobian of Xdot with respect to the states:
 *
 *	A = [ d(angle_dot)/d(angle)     d(angle_dot)/d(gyro_bias) ]
 *	    [ d(gyro_bias_dot)/d(angle) d(gyro_bias_dot)/d(gyro_bias) ]
 *
 *	  = [ 0 -1 ]
 *	    [ 0  0 ]
 *
 * Due to the small CPU available on the microcontroller, we've
 * hand optimized the C code to only compute the terms that are
 * explicitly non-zero, as well as expanded out the matrix math
 * to be done in as few steps as possible.  This does make it harder
 * to read, debug and extend, but also allows us to do this with
 * very little CPU time.
 */
void
state_update(
	const float		q_m	/* Pitch gyro measurement */
)
{
	/* Unbias our gyro */
	const float		q = q_m - q_bias;

	/*
	 * Compute the derivative of the covariance matrix
	 *
	 *	Pdot = A*P + P*A' + Q
	 *
	 * We've hand computed the expansion of A = [ 0 -1, 0 0 ] multiplied
	 * by P and P multiplied by A' = [ 0 0, -1, 0 ].  This is then added
	 * to the diagonal elements of Q, which are Q_angle and Q_gyro.
	 */
	const float		Pdot[2 * 2] = {
		Q_angle - P[0][1] - P[1][0],	/* 0,0 */
		        - P[1][1],		/* 0,1 */
		        - P[1][1],		/* 1,0 */
		Q_gyro				/* 1,1 */
	};

	/* Store our unbias gyro estimate */
	rate = q;

	/*
	 * Update our angle estimate
	 * angle += angle_dot * dt
	 *       += (gyro - gyro_bias) * dt
	 *       += q * dt
	 */
	angle += q * dt;

	/* Update the covariance matrix */
	P[0][0] += Pdot[0] * dt;
	P[0][1] += Pdot[1] * dt;
	P[1][0] += Pdot[2] * dt;
	P[1][1] += Pdot[3] * dt;
}


/*
 * kalman_update is called by a user of the module when a new
 * accelerometer measurement is available.  ax_m and az_m do not
 * need to be scaled into actual units, but must be zeroed and have
 * the same scale.
 *
 * This does not need to be called every time step, but can be if
 * the accelerometer data are available at the same rate as the
 * rate gyro measurement.
 *
 * For a two-axis accelerometer mounted perpendicular to the rotation
 * axis, we can compute the angle for the full 360 degree rotation
 * with no linearization errors by using the arctangent of the two
 * readings.
 *
 * As commented in state_update, the math here is simplified to
 * make it possible to execute on a small microcontroller with no
 * floating point unit.  It will be hard to read the actual code and
 * see what is happening, which is why there is this extensive
 * comment block.
 *
 * The C matrix is a 1x2 (measurements x states) matrix that
 * is the Jacobian matrix of the measurement value with respect
 * to the states.  In this case, C is:
 *
 *	C = [ d(angle_m)/d(angle)  d(angle_m)/d(gyro_bias) ]
 *	  = [ 1 0 ]
 *
 * because the angle measurement directly corresponds to the angle
 * estimate and the angle measurement has no relation to the gyro
 * bias.
 */
void
kalman_update(
	const float		ax_m,	/* X acceleration */
	const float		az_m	/* Z acceleration */
)
{
	/* Compute our measured angle and the error in our estimate */
	const float		angle_m = atan2( -az_m, ax_m );
	const float		angle_err = angle_m - angle;

	/*
	 * C_0 shows how the state measurement directly relates to
	 * the state estimate.
 	 *
	 * The C_1 shows that the state measurement does not relate
	 * to the gyro bias estimate.  We don't actually use this, so
	 * we comment it out.
	 */
	const float		C_0 = 1;
	/* const float		C_1 = 0; */

	/*
	 * PCt<2,1> = P<2,2> * C'<2,1>, which we use twice.  This makes
	 * it worthwhile to precompute and store the two values.
	 * Note that C[0,1] = C_1 is zero, so we do not compute that
	 * term.
	 */
	const float		PCt_0 = C_0 * P[0][0]; /* + C_1 * P[0][1] = 0 */
	const float		PCt_1 = C_0 * P[1][0]; /* + C_1 * P[1][1] = 0 */
		
	/*
	 * Compute the error estimate.  From the Kalman filter paper:
	 * 
	 *	E = C P C' + R
	 * 
	 * Dimensionally,
	 *
	 *	E<1,1> = C<1,2> P<2,2> C'<2,1> + R<1,1>
	 *
	 * Again, note that C_1 is zero, so we do not compute the term.
	 */
	const float		E =
		R_angle
		+ C_0 * PCt_0
	/*	+ C_1 * PCt_1 = 0 */
	;

	/*
	 * Compute the Kalman filter gains.  From the Kalman paper:
	 *
	 *	K = P C' inv(E)
	 *
	 * Dimensionally:
	 *
	 *	K<2,1> = P<2,2> C'<2,1> inv(E)<1,1>
	 *
	 * Luckilly, E is <1,1>, so the inverse of E is just 1/E.
	 */
	const float		K_0 = PCt_0 / E;
	const float		K_1 = PCt_1 / E;
		
	/*
	 * Update covariance matrix.  Again, from the Kalman filter paper:
	 *
	 *	P = P - K C P
	 *
	 * Dimensionally:
	 *
	 *	P<2,2> -= K<2,1> C<1,2> P<2,2>
	 *
	 * We first compute t<1,2> = C P.  Note that:
	 *
	 *	t[0,0] = C[0,0] * P[0,0] + C[0,1] * P[1,0]
	 *
	 * But, since C_1 is zero, we have:
	 *
	 *	t[0,0] = C[0,0] * P[0,0] = PCt[0,0]
	 *
	 * This saves us a floating point multiply.
	 */
	const float		t_0 = PCt_0; /* C_0 * P[0][0] + C_1 * P[1][0] */
	const float		t_1 = C_0 * P[0][1]; /* + C_1 * P[1][1]  = 0 */

	P[0][0] -= K_0 * t_0;
	P[0][1] -= K_0 * t_1;
	P[1][0] -= K_1 * t_0;
	P[1][1] -= K_1 * t_1;
	
	/*
	 * Update our state estimate.  Again, from the Kalman paper:
	 *
	 *	X += K * err
	 *
	 * And, dimensionally,
	 *
	 *	X<2> = X<2> + K<2,1> * err<1,1>
	 *
	 * err is a measurement of the difference in the measured state
	 * and the estimate state.  In our case, it is just the difference
	 * between the two accelerometer measured angle and our estimated
	 * angle.
	 */
	angle	+= K_0 * angle_err;
	q_bias	+= K_1 * angle_err;
}


#endif
