#define FILTER3
//#define STABILAZER_RELEASE
//#define THATAE_BY_ENCODER
//#define NOTCH_TEST_ROLL
//#define NOTCH_TEST_PITCH
#define LOAD_IO

#include <stdlib.h>
#include <math.h>

#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/lm4f230h5qr.h"
#include "driverlib/gpio.h"
#include "driverlib/qei.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/pin_map.h"
#include "driverlib/systick.h"
#include "driverlib/interrupt.h"

#include "cbuf.h"
#include "bluebird.h"
#include "inc/hw_ints.h"
#include "inc/hw_types.h"
#include "inc/lm4f230h5qr.h"
#include "driverlib/qei.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "A3906.h"
#include "rmb20d01.h"
#include "ITG3200.h"
#include "BMA180.h"
#include "Interpret.h"
#include "Stabilize.h"
#include "globals.h"



#ifdef FILTER3
#include "filter3.h"
#endif

extern tBoolean ignorejoystick;
extern float  vertical,horizontal;
extern unsigned long ulPeriod;


#define X_AT_REST ITG3200AtRest[X_AXIS]
#define Y_AT_REST ITG3200AtRest[Y_AXIS]
#define Z_AT_REST ITG3200AtRest[Z_AXIS]

float Phie,Thetae;
float omega_vertical, omega_horizontal; 
float pre_vertical = 0.0f;
	
float k_a_roll = 0.1f;
float k_g_roll = 0.2f;
float k_i_roll = 0.2f;
float k_7_roll = 0.02f;
float k_8_roll = 1.5f;
float k_9_roll = -1.5f;
float k_g_roll_0 = 1.5f;
float k_g_roll_1 = 0.3f;
float k_i_roll_0 = 0.3f;
float k_i_roll_1 = 0.2f;

float k_ig_roll = 0.01f;


 
float k_a_pitch = 0.1f;
float k_g_pitch = 0.2f;
float k_i_pitch = 0.2f;
float k_7_pitch = 0.02f;
float k_8_pitch = 1.5f;
float k_9_pitch = -1.5f;
float k_g_pitch_0 = 1.5f;
float k_g_pitch_1 = 0.3f;
float k_i_pitch_0 = 0.1f;
float k_i_pitch_1 = 0.1f;


float g_fmindutycycle[2][4] = { {0,8.0f,-8.0f,0}, //PITCH_MOTOR=0
								{0,10.0f,-10.0f,0}}; //ROLL_MOTOR=0

unsigned long g_uldutycycle = 0;
static unsigned long g_uCunter = 0;
static tBoolean g_bKdirection = FALSE;

float g_fdutycycle = 0;

//////// NeW ConsTantS ///////////////////////////////////////////////                
float ref_pos_roll = 0.0f;
float e_pos_roll = 0.0f;
float y_pos_roll = 0.0f;
float e_rate_roll = 0.0f;
float y_rate_roll  = 0.0f;
float u_rate_roll = 0.0f;
float DL_ROLL_SAT = 70.0f; // Const to play with 
float IL_ROLL_SAT = 50.0f;
float out_rate_roll = 0.0f;
float prev_e_pos_roll = 0.0f;
static float io_rate_roll = 0.0f;
float il_rate_roll = 0.0f;
float w_rate_roll = 0.0f;
float i_rate_roll = 0.0f;

float ref_pos_pitch = 0.0f;
float e_pos_pitch = 0.0f;
float y_pos_pitch = 0.0f;
float ref_rate_pitch = 0.0f;
float e_rate_pitch = 0.0f;
float y_rate_pitch  = 0.0f;
float u_rate_pitch = 0.0f;
float DL_PITCH_SAT = 70.0f; // Const to play with
float IL_PITCH_SAT = 50.0f;
float out_rate_pitch = 0.0f;
float io_rate_pitch = 0.0f;
float w_rate_pitch = 0.0f;

// NOTCH stuff
#ifdef NOTCH_TEST_ROLL
#define RN1 0.573
#define RN2 -0.6411
#define RN3 0.1793
#define RD1 1.0
#define RD2 -0.6411
#define RD3 -0.2477


float urolly[3],urollz[3],orolly[3],orollz[3],opitch[3];
#endif

// NOTCH stuff
#ifdef NOTCH_TEST_PITCH
//#define N1 0.573
//#define N2 -0.6411
//#define N3 0.1793
//#define D1 1.0
//#define D2 -0.6411
//#define D3 -0.2477

#define PN1 1.0f
#define PN2 -1.0f
#define PN3 1.0f
#define PD1 1.0f
#define PD2 -1.0f
#define PD3 -1.0f

float upitch[3];
#endif

////////END of NeW ConsTantS /////////////////////////////////////////////// 

void SetRelativeInertial(void)
{

	y_pos_roll = Phic * RAD2DEG; // ROLL
	y_pos_pitch = Thetac * RAD2DEG; // PITCH


    // move gimbal to centre, half down
    omega_vertical = y_pos_pitch; // ((QEIPositionGet(QEI0_BASE) - Motor[PITCH_MOTOR].zero_pos) / 924.444f);
    omega_horizontal = y_pos_roll; // ((QEIPositionGet(QEI1_BASE) - Motor[ROLL_MOTOR].zero_pos) / 924.444f);
  	
}

void StabilizeInit(void)
{
  long i;
  long rest[3] = {0,0,0};
  //CircularBufferInit(&que, BLOCKSIZE);

  for (i=0; i<NUM_SAMPLES;)
  {
    if (itgready)
    {
      ITG3200getXYZ();
      itgready = false;
      i++;
      rest[Z_AXIS] += ITG3200values[Z_AXIS];
      rest[Y_AXIS] += ITG3200values[Y_AXIS];
      rest[X_AXIS] += ITG3200values[X_AXIS];
    }
  }
  ITG3200AtRest[Z_AXIS] = rest[Z_AXIS]/NUM_SAMPLES;
  ITG3200AtRest[Y_AXIS] = rest[Y_AXIS]/NUM_SAMPLES;
  ITG3200AtRest[X_AXIS] = rest[X_AXIS]/NUM_SAMPLES;

}

void Stabilize(enum MOTOR motor)
{ 
  if (motor == ROLL_MOTOR)
  { 
    ///////////////////////////////////////////////////////////////////////////////////
    //////////////////// ROLL /////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////////
	#ifdef STABILAZER_RELEASE
	if( ( QEIPositionGet(QEI1_BASE) >= Motor[motor].min_position) && (horizontal > 0.0f) && (ignorejoystick == false) )
		 GeneralPitchRoll(motor,A3906_DISABLE,ulPeriod);	
	else
	if( (QEIPositionGet(QEI1_BASE) <= Motor[motor].max_position) && (horizontal < 0.0f) && (ignorejoystick == false) )	 
		GeneralPitchRoll(motor,A3906_DISABLE,ulPeriod); // Set BIT
	else	 
	#endif	
	{

	    ref_pos_roll = omega_horizontal;
	    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	    // ADD HERE THE CODE WHICH DISABLES THE MOTOR WHEN IT REACHES THE LIMITATION
		#ifdef USE_BMA180_INTERRUPT
		y_rate_roll = ((ITG3200values[Y_AXIS] - Y_AT_REST) * (FULL_SCALE_RANGE / 32768.0f))*cosf(Thetac) - ((ITG3200values[X_AXIS] - X_AT_REST) * (FULL_SCALE_RANGE / 32768.0f))*sinf(Thetac);		
		//i_rate_roll+= k_ig_roll*(y_rate_roll - ;
		#else
	    y_rate_roll = ((ITG3200values[Y_AXIS] - Y_AT_REST) * (FULL_SCALE_RANGE / 32768.0f)) - ((ITG3200values[X_AXIS] - X_AT_REST) * (FULL_SCALE_RANGE / 32768.0f));		
		#endif

		#if 0
	    if ((y_rate_roll < k_8_roll) && (y_rate_roll > k_9_roll)) // 2.0f
	    {
	    	//k_g_roll = k_g_roll_0;
	    	k_i_roll = k_i_roll_0;
	    	//g_bKdirection = TRUE;
			//io_rate_roll = g_fmindutycycle[ROLL_MOTOR][A3906_FORWARD];
	    }
		else
			k_i_roll = k_i_roll_1;
			//k_g_roll = k_g_roll_1;
	    #endif		
	    
		#ifdef USE_BMA180_INTERRUPT
	    //y_rate_roll - stores current rolling speed
	    y_pos_roll = Phic * RAD2DEG;
	    e_pos_roll = ref_pos_roll - y_pos_roll; // in degrees
	    e_rate_roll = k_a_roll*e_pos_roll-k_g_roll*y_rate_roll;
		#else
		e_rate_roll = -k_g_roll*y_rate_roll;
		#endif

		#ifdef STABILAZER_RELEASE
	    if (fabs(horizontal) > 0.0f)
	    {		  
	      e_rate_roll += horizontal;
	      omega_horizontal = (Phic * RAD2DEG);
	    }
		#endif
		
	    io_rate_roll += e_rate_roll*k_i_roll;
	
	    if (io_rate_roll > IL_ROLL_SAT)
	      io_rate_roll = IL_ROLL_SAT;
	    else if (io_rate_roll < -IL_ROLL_SAT)
	      io_rate_roll =-IL_ROLL_SAT;

		
	    u_rate_roll = e_rate_roll + io_rate_roll;


	    if (u_rate_roll > DL_ROLL_SAT)
	      out_rate_roll = DL_ROLL_SAT;
	    else if (u_rate_roll < -DL_ROLL_SAT)
	      out_rate_roll =-DL_ROLL_SAT;
	    else
	      out_rate_roll = u_rate_roll;
		  
	    //if (fabs(out_rate_roll) < k_7_roll) // 0.2
	      //GeneralPitchRoll(ROLL_MOTOR,A3906_DISABLE,dcycle[motor]);
	    //else
	    //{		
	      if (out_rate_roll > 0.0f)
	      {
	      	  #ifdef LOAD_IO
			  if((mdirection[ROLL_MOTOR] != A3906_FORWARD) && (horizontal > 0))
				  io_rate_roll = g_fmindutycycle[ROLL_MOTOR][A3906_FORWARD];
			  #endif
			  
			  g_uldutycycle = ( fabs(out_rate_roll) * ulPeriod ) / 100.0f;
			  
			  //if( (QEIPositionGet(QEI1_BASE) >= Motor[motor].min_position) && (ignorejoystick == false) )
				//GeneralPitchRoll(motor,A3906_DISABLE,ulPeriod);	
			  //else		
				GeneralPitchRoll(motor,A3906_FORWARD,  g_uldutycycle);
		  }
		  else
		  {
		  	  #ifdef LOAD_IO
			  if((mdirection[ROLL_MOTOR] != A3906_REVERSE) && (horizontal < 0))
				  io_rate_roll = g_fmindutycycle[ROLL_MOTOR][A3906_REVERSE];
			  #endif
			  
			  g_uldutycycle = ( fabs(out_rate_roll) * ulPeriod ) / 100.0f;	
			  
			  //if( (QEIPositionGet(QEI1_BASE) <= Motor[motor].max_position) && (ignorejoystick == false) )	   
				//GeneralPitchRoll(motor,A3906_DISABLE,ulPeriod); // Set BIT
			  //else		
				GeneralPitchRoll(motor,A3906_REVERSE, g_uldutycycle);
	      }
	    //}
	}
  }
  //////////////////////////////////////////////////////////////////////////////////
  //////////////////// END of ROLL ////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////////
  else
  { 
    //////////////////////////////////////////////////////////////////////////////////
    //////////////////// PITCH ///////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////////
    #ifdef STABILAZER_RELEASE
	if( (QEIPositionGet(QEI0_BASE) >= Motor[motor].max_position) && (vertical > 0.0f) && (ignorejoystick == false) )
	 	GeneralPitchRoll(motor,A3906_DISABLE,ulPeriod);  
	else
	if( (QEIPositionGet(QEI0_BASE) <= Motor[motor].min_position) && (vertical < 0.0f) && (ignorejoystick == false) )	 
		GeneralPitchRoll(motor,A3906_DISABLE,ulPeriod); // Set BIT
	else
	#endif
	{

	    ref_pos_pitch =  omega_vertical;
	    y_rate_pitch = ((ITG3200values[Z_AXIS] - Z_AT_REST)*(FULL_SCALE_RANGE/32768.0f));


		#if 0
		if ((y_rate_pitch < k_8_pitch) && (y_rate_pitch > k_9_pitch)) // 2.0f
		{
			//k_g_roll = k_g_roll_0;
			k_i_pitch = k_i_pitch_0;
			//g_bKdirection = TRUE;
			//io_rate_roll = g_fmindutycycle[ROLL_MOTOR][A3906_FORWARD];
			//DEBUG_LED1(0);
		}
		else
			k_i_pitch = k_i_pitch_1;
			//k_g_roll = k_g_roll_1;
		#endif		

		#ifdef USE_BMA180_INTERRUPT
	    y_pos_pitch = Thetac * RAD2DEG;
	    e_pos_pitch = ref_pos_pitch - y_pos_pitch;
		e_rate_pitch = k_a_pitch*e_pos_pitch-k_g_pitch*y_rate_pitch;
		#else
		e_rate_pitch = -k_g_pitch*y_rate_pitch;
		#endif
		
		#ifdef STABILAZER_RELEASE
	    if (fabs(vertical) > 0.0f)
	    {
	    	//if( (vertical > 0) && (pre_vertical < 0))
			 //u_rate_pitch = vertical;
			//else if( (vertical < 0) && (pre_vertical > 0))
				//u_rate_pitch = vertical;
			//else
	         e_rate_pitch += vertical;

			//pre_vertical = vertical;
	        omega_vertical = (Thetac * RAD2DEG);
	    }
		#endif
		
	    io_rate_pitch += e_rate_pitch*k_i_pitch;

	    if (io_rate_pitch > IL_PITCH_SAT)
	      io_rate_pitch = IL_PITCH_SAT;
	    else if (io_rate_pitch < -IL_PITCH_SAT)
	      io_rate_pitch =-IL_PITCH_SAT;
		
	    u_rate_pitch = e_rate_pitch + io_rate_pitch;

	    if (u_rate_pitch > DL_PITCH_SAT)
	      out_rate_pitch = DL_PITCH_SAT;
	    else if (u_rate_pitch < -DL_PITCH_SAT)
	      out_rate_pitch = -DL_PITCH_SAT;
	    else
	      out_rate_pitch = u_rate_pitch;

	    //if (fabs(out_rate_pitch) < k_7_pitch) // 0.2
	      //GeneralPitchRoll(PITCH_MOTOR,A3906_DISABLE,dcycle[motor]);
	    //else
	    //{		
	      if (out_rate_pitch > 0.0f)
	      {
	      	  #ifdef LOAD_IO
			  if((mdirection[PITCH_MOTOR] != A3906_FORWARD) && (vertical > 0))
					io_rate_pitch = g_fmindutycycle[PITCH_MOTOR][A3906_FORWARD];
			  #endif
			  
			  g_uldutycycle = ( fabs(out_rate_pitch) * ulPeriod ) / 100.0f;
			  
			  //if( (QEIPositionGet(QEI0_BASE) >= Motor[motor].max_position) && (ignorejoystick == false) )
				//GeneralPitchRoll(motor,A3906_DISABLE,ulPeriod);	
			  //else		  	      	      	      			  	
		        GeneralPitchRoll(motor,A3906_FORWARD, g_uldutycycle);
	      }
	      else
	      {
	      	  #ifdef LOAD_IO
			  if((mdirection[PITCH_MOTOR] != A3906_REVERSE) && (vertical < 0))
					io_rate_pitch = g_fmindutycycle[PITCH_MOTOR][A3906_REVERSE];
			  #endif
			  
	      	  g_uldutycycle = ( fabs(out_rate_pitch) * ulPeriod ) / 100.0f;
			  
			  //if( (QEIPositionGet(QEI0_BASE) <= Motor[motor].min_position) && (ignorejoystick == false) )	   
				//GeneralPitchRoll(motor,A3906_DISABLE,ulPeriod); // Set BIT
			  //else			
		        GeneralPitchRoll(motor,A3906_REVERSE, g_uldutycycle);
			
	      }
	    //}
	}
  }
  ///////////////////////////////////////////////////////////////////////////////////
  //////////////////// END of PITCH //////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////////
}


