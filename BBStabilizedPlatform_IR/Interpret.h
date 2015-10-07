#ifndef _INTERPRET_H
#define _INTERPRET_H

#define h_to_a(x)      ( ((x) >= 10 && (x) <= 15) ?   (x)-10+0x41  : (x)+'0' )
#define a_to_h(c)      ( ((c) >= 'A' && (c) <= 'F') ? (c)-'A'+10 : (c)-'0' )

#define MAX_ZX0		0x1000

#define MAX_ZX3		0x12EC
#define MAX_ZX4 	0x134C
#define MAX_ZX6 	0x13D0
#define MAX_ZX10 	0x145B

#define MINI_ZOOM	MAX_ZX0
#define MAX_ZOOM	MAX_ZX10

#define PI 3.14159265
#define RAD2DEG (180.0/PI) 	// ~57.29
#define DEG2RAD (PI/180.0)	// 
#define ROLL_REPORT (360.0/4096)	// 


// Mode commands
#define  OBSERVATION_MODE  0
#define  POINT_COORDINATE_MODE 1
#define  HOLD_COORDINATE_MODE 2
#define  PILOT_WINDOW_MODE 3
#define  STOW_MODE 4
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


char RateMode(unsigned char* msg);
char StowMode(unsigned char* msg);
char PilotWindowMode(unsigned char* msg);
char HoldCoordinateMode(unsigned char* msg);
char PointCoordinateMode(unsigned char* msg);
char CenterMode(unsigned char* msg);

unsigned int GetDMCflag(unsigned int mask);
void SetDMCflag(unsigned int mask, unsigned int val);
extern void InterpretCmd(void);
extern void InterpretCmdMode(unsigned char *bdData1);
extern void InterpretCorrelatorMode(unsigned char *bdData1);
extern void CameraCalibMode(unsigned char* msg);
extern void ScanMode(unsigned char* msg);
extern void DriftCalibMode(unsigned char* msg);
extern void ScaleCalibMode(unsigned char* msg);
extern void GimbalCalibMode(unsigned char* msg);
extern char BitMode(unsigned char* msg);
extern void FlirCalibMode(unsigned char* msg);
extern void PositionMode(unsigned char* msg);
extern void VHCalibMode(unsigned char* msg);
extern void TestScanMode(unsigned char* msg);
extern void GpsNavInitMode(unsigned char* msg);
extern void NavInitMode(unsigned char* msg);
extern void Mode17Mode(unsigned char* msg);
extern void GyrocompassMode(unsigned char* msg);
extern void GimbalCheckMode(unsigned char* msg);
extern void GyroCalibMode(unsigned char* msg);
extern void ObstructionTest1Mode(unsigned char* msg);
extern void ObstructionTest2Mode(unsigned char* msg);
extern void ObstructionTest3Mode(unsigned char* msg);
extern void BoresightCalibMode(unsigned char* msg);
extern void ImageFlipMode(unsigned char* msg);
extern void GimbalZeroMode(unsigned char* msg);
extern void RatePositionMode(unsigned char* msg);
extern void Mode29Mode(unsigned char* msg);
extern void CommlossMode(unsigned char* msg);
extern void MaintenanceMode(unsigned char* msg);
extern void ReportData(unsigned long ulBase);
extern void Osd_Show(unsigned char* msg);
extern void UavLongitude(unsigned char* msg);
extern void UavLatitude(unsigned char* msg);
extern void	Ground_Height_above_MSL(unsigned char* msg);
extern void SoftwareInit();
extern void SoftwareBitMode();

void UtlDec2asc(char *buff,unsigned int deci,unsigned char *index);
void InitSens(void);
void SensorDebounce(unsigned char* msg);
unsigned short GetSensorState(void);

#endif
