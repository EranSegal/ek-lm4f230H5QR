#ifndef _INTERPRET_H
#define _INTERPRET_H


typedef struct {
   unsigned short StartZoom;
   unsigned short EndZoom;
   float		  FOV2Deg;   
} ZoomPosition;

#define h_to_a(x)      ( ((x) >= 10 && (x) <= 15) ?   (x)-10+0x41  : (x)+'0' )
#define a_to_h(c)      ( ((c) >= 'A' && (c) <= 'F') ? (c)-'A'+10 : (c)-'0' )


#define S_MAX_ZX0		0x1000
#define E_MAX_ZX0		0x108F

#define S_MAX_ZX12		0x1090
#define E_MAX_ZX12		0x110F

#define S_MAX_ZX15		0x1110
#define E_MAX_ZX15		0x11CF

#define S_MAX_ZX2		0x11D2
#define E_MAX_ZX2		0x12A6

#define S_MAX_ZX3		0x12A8
#define E_MAX_ZX3		0x132F

#define S_MAX_ZX4		0x1330
#define E_MAX_ZX4		0x1367

#define S_MAX_ZX5		0x1368
#define E_MAX_ZX5		0x13AF

#define S_MAX_ZX6		0x13B0
#define E_MAX_ZX6		0x13EF

#define S_MAX_ZX7		0x13F0
#define E_MAX_ZX7		0x1427

#define S_MAX_ZX8		0x1428
#define E_MAX_ZX8		0x1447

#define S_MAX_ZX9		0x1448
#define E_MAX_ZX9		0x1457

#define S_MAX_ZX10		0x1458
#define E_MAX_ZX10		0x145B

#define MINI_ZOOM	S_MAX_ZX0
#define MAX_ZOOM	E_MAX_ZX10

#define MINI_AGC_LEVEL	0
#define MAX_AGC_LEVEL	8

#define MINI_SENS_LEVEL	0
#define MAX_SENS_LEVEL	8

//#define PI 3.14159265f
//#define RAD2DEG (180.0f/PI) 	// ~57.29
//#define DEG2RAD (PI/180.0f)	// 
#define ROLL_REPORT (360.0/4096)	// 


// Mode commands
#define  OBSERVATION_MODE  0
#define  POINT_COORDINATE_MODE 1
#define  HOLD_COORDINATE_MODE 2
#define  PILOT_WINDOW_MODE 3
#define  SAFE_MODE 4
#define  CAMERA_CALIB_MODE 5
#define  SCAN_MODE 6
#define  DRIFT_CALIB_MODE 7
#define  SCALE_CALIB_MODE 8
#define  GIMBAL_CALIB_MODE 9
#define  BIT_MODE 10
#define  FLIR_CALIB_MODE 11
#define  POSITION_MODE 12
#define  V_H_CALIB_MODE 13
#define  TEST_SCAN_MODE 14
#define  GPS_NAV_INIT_MODE 15
#define  NAV_INIT_MODE 16
#define  MODE_17_MODE 17
#define  GYROCOMPASS_MODE 18
#define  GIMBAL_CHECK_MODE 19
#define  GYRO_CALIB_MODE 20
#define  OBSTRUCTION_TEST1_MODE 21
#define  OBSTRUCTION_TEST2_MODE 22
#define  OBSTRUCTION_TEST3_MODE 23
#define  BORESIGHT_CALIB_MODE 24
#define  CENTER_MODE 25
#define  IMAGE_FLIP_MODE 26
#define  GIMBAL_ZERO_MODE 27
#define  RATE_POSITION_MODE 28
#define  MODE_29_MODE 29
#define  COMMLOSS_MODE 30
#define  MAINTENANCE_MODE 31

#define  OSD_ODE 0x80
#define  CHECKING_NUMBER_UP 0x1F
#define  CHECKING_CORR_NUMBER 0xF0


// Controller commands	 byte 8   D4	D5	D6	D7
#define  UAV_LATITUDE 				0  //0
#define  UAV_LONGITUDE 				1  //
#define  UAV_ALTITUDE_ABOVE_MSL		2  // (float) 0	1	0	0
#define  GROUND_HEIGHT_ABOVE_MSL	3
//#define  Rate_of_Climb		//* 	//	EREZ  ??		
//#define  UAV_data_source		//* //	EREZ  ??			
#define  UAV_BODY_AZIMUTH			4 //0 0	1	0
#define  GROUND_SPEED				5 	// 1	0	1	0
#define  GROUND_SPEED_AZIMUTH		10	  //  EREZ	??				
#define  TARGET_LATITUDE			6	// (REAL)	0	1	1	0
#define  TARGET_LONGITUDE			7 //(REAL)	1	1	1	0
#define  TARGET_ALTITUDE_ABOVE_MSL 	8	//	0	0	0	1
#define  DIAL_ADDRESS 			   	9   //	1	0	0	1
#define  ELEVATION_GIMBAL_ANGLE	   	10		// 0	1	0	1
#define  SPARE11					   	11		// 0	1	0	1
#define  SPARE12					   	12		// 0	1	0	1
#define  SPARE13					   	13		// 0	1	0	1
#define  SPARE14					   	14		// 0	1	0	1
#define  SPARE15					   	15		// 0	1	0	1

int RateMode(unsigned char* msg);
int SafeMode(unsigned char* msg);
int PilotWindowMode(unsigned char* msg);
int HoldCoordinateMode(unsigned char* msg);
int PointCoordinateMode(unsigned char* msg);

void Osd_Show(unsigned char* msg);
void Mark_Show(unsigned char* msg);
void UavLatitude(unsigned char* msg);
void UavLongitude(unsigned char* msg);
void SetDMCflag(unsigned int mask, unsigned int val);
void InterpretCmd(void);
void InitSens(void);
void SensorDebounce(unsigned char* msg);
float  FOV_command(int command);
void ZoomFovCalculation(void);
void SetZoomBitMode(void);


unsigned short GetSensorState(void);
unsigned int GetDMCflag(unsigned int mask);

#endif
