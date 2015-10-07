/*
  Copyright BlueBird 2011

  $Id: rmb20d01.h
  $Log: rmb20d01.h,v $
  Revision 1.5  2011/05/03 06:46:07  EranS
  After Eran rewire - pay attention to encoder pins

  Revision 1.4  2011/04/17 08:44:11  EranS
  add proper reset of encoders via GPIO

  
*/
void RMBD01Init(void);
void EncoderInitReset(unsigned long ulBase,enum MOTOR motor,enum A3906Logic logic);
void EncoderInit2Point(enum MOTOR motor,enum A3906Logic driction,unsigned long point);
void EncoderWhile2Point(enum MOTOR motor,enum A3906Logic driction,unsigned long point);

typedef struct _MotorStatus
{
		unsigned long			position;
		long					direction;
		float					aangle;
		float					cur_angle;
		float 					deg2sec;		
		float					max_angle_position;
		float					min_angle_position;		
		float 					zero_pos;
		unsigned long			max_position;
		unsigned long			min_position;	
		unsigned long			avg_position;			
		unsigned long			stowpoint;	
		unsigned long			pilotpoint;							
		unsigned long			relative;
		
} MotorStatus;

#ifdef RMB20_MAIN
unsigned long PitchRollAmp[2];
MotorStatus Motor[4];
unsigned long read_encode;
tBoolean new_adc;
#else
extern unsigned long PitchRollAmp[2];
extern MotorStatus Motor[4];
extern unsigned long read_encode;
extern tBoolean new_adc;
#endif
