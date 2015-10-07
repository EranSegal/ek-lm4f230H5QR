/***************************************************************************/
/*! 
@ingroup    MPSDKTelemetry
@brief      Standard Telemetry - Holds telemetry info
@par
This structure is used in mpGetStandardTelemetry() function.  It contains various
information that is retrieved from standard telemetry.  Consult the manual
for specific information regarding each of variables.

****************************************************************************/

typedef struct mpStdTelemetryData
{
    long   cbSize;  /*!< @brief ALWAYS init cbSize to the sizeof() this struct before passing to any API */

    /* original standard telemetry */
    int speed;      /*!< @brief Speed (ft/s)				*/
    int gpsSpeed;   /*!< @brief GPS speed (ft/s)			*/
    /* ALWAYS use e and n together */
    int eTemp;      /*!< @brief Unused.  Left for compatibility issues */
    int e;          /*!< @brief GPS Longitude (Radians*500000000 - integer scaled radians) */
    int n;          /*!< @brief GPS Latitude (Radians*500000000 - integer scaled radians) */
    int alt;        /*!< @brief Altitude (-8*ft - integer scaled negative feet) */
    int altDot;     /*!< @brief Altitude 1st derivative (-8*ft/s - integer scaled negative feet per second ) */
    int hdg;        /*!< @brief Heading (degrees*100 - integer scaled degrees) 0..360 degrees */
    int err;        /*!< @brief Last error code (see mperror.h for possible error code values) */
    int status;     /*!< @brief Status bit fields */
    int status2;    /*!< @brief Status bit fields */
    int batV;       /*!< @brief Main battery voltage (Volts*100 - integer scaled Volts) */
    int sbatV;      /*!< @brief Servo battery voltage (Volts*100 - integer scaled Volts) */
    int sTh;        /*!< @brief Servo throttle position. (FINE-SERVO units mapped to 0..255) */
    int tlmPitch;   /*!< @brief Autopilot Pitch (Radians*1024 - integer scaled radians) */
    int tlmRoll;    /*!< @brief Autopilot Roll (Radians*1024 - integer scaled radians) */

    /* exteneded standard telemetry data */
    long    ipStep;             /*!< @brief Instruction pointer position */
    long    ipCmd;              /*!< @brief Instruction being executed */
    long    patternId;          /*!< @brief Pattern invoked if applicable */
    long    failureId;          /*!< @brief Failure pattern invoked if applicable */
    long    targetSpeed_fps;    /*!< @brief Target speed (ft/s - feet per second) */
    long    targetAlt_ft;       /*!< @brief Target altitude (-8*ft - integer scaled negative feet) */
    long    targetHeading_deg;  /*!< @brief Target heading (degrees*100 - integer scaled degrees) 0..360 degrees */

    int     waypointversion ;   /*!< @brief Waypoint Version (incremented on waypoint move) */

    long    mEvent;             /*!< @brief Event warning or non-fatal error status from autopilot */
    
    //HH:2005-12-13:extended standard telemetry again (GCS-225)
    long    disableNewOriginSet;    /*!< @brief Disable pattern origin movement for certain command conditions*/

    //TM:2006-06-13:extend standard telemetry
    long    ownerGcsId;         /*!< @brief GCS id of the owner of this UAV ( 0 = no owner ) */

    //TM:2006-06-30:extend standard telemetry again
    long    gpsAlt;             /*!< @brief GPS altitude of the plane (-8*ft - integer scaled negative feet) */

    long    mpMode;    /*!< @brief Mode of the autopilot, shifted left by 4 bits. */

    long    lrcStatus; /*!< @brief Status of the LRC */

} MPSTDTELEMETRYDATA;

/*! @endcond */

typedef struct APTelemetryDataStruct
{
	MPSTDTELEMETRYDATA StandardTelem;
	unsigned short StandardTelemValid;
	long UserTelemetryData[100];
	unsigned short UserTelemValid;

	//======Standard Telemetry Data translated=============
	//======================================================
    double std_AirSpeed;      /*!< @brief Speed (ft/s)				*/
    double std_gpsSpeed;   /*!< @brief GPS speed (ft/s)			*/
    /* ALWAYS use e and n together */
    double std_Longitude;          /*!< @brief GPS Longitude (Radians*500000000 - integer scaled radians) */
    double std_Latitude;          /*!< @brief GPS Latitude (Radians*500000000 - integer scaled radians) */
    double std_alt;        /*!< @brief Altitude (-8*ft - integer scaled negative feet) */
    double std_altDot;     /*!< @brief Altitude 1st derivative (-8*ft/s - integer scaled negative feet per second ) */
    double std_hdg;        /*!< @brief Heading (degrees*100 - integer scaled degrees) 0..360 degrees */
    int std_err;        /*!< @brief Last error code (see mperror.h for possible error code values) */
    int std_status;     /*!< @brief Status bit fields */
    int std_status2;    /*!< @brief Status bit fields */
    double std_batV;       /*!< @brief Main battery voltage (Volts*100 - integer scaled Volts) */
    double std_sbatV;      /*!< @brief Servo battery voltage (Volts*100 - integer scaled Volts) */
    int std_sTh;        /*!< @brief Servo throttle position. (FINE-SERVO units mapped to 0..255) */
    double std_tlmPitch;   /*!< @brief Autopilot Pitch (Radians*1024 - integer scaled radians) */
    double std_tlmRoll;    /*!< @brief Autopilot Roll (Radians*1024 - integer scaled radians) */

    /* exteneded standard telemetry data */
    long    std_ipStep;             /*!< @brief Instruction pointer position */
    long    std_ipCmd;              /*!< @brief Instruction being executed */
    long    std_patternId;          /*!< @brief Pattern invoked if applicable */
    long    std_failureId;          /*!< @brief Failure pattern invoked if applicable */
    double    std_targetSpeed_fps;    /*!< @brief Target speed (ft/s - feet per second) */
    double    std_targetAlt_ft;       /*!< @brief Target altitude (-8*ft - integer scaled negative feet) */
    double    std_targetHeading_deg;  /*!< @brief Target heading (degrees*100 - integer scaled degrees) 0..360 degrees */

    int     std_waypointversion ;   /*!< @brief Waypoint Version (incremented on waypoint move) */
    long    std_mEvent;             /*!< @brief Event warning or non-fatal error status from autopilot */
    long    std_disableNewOriginSet;    /*!< @brief Disable pattern origin movement for certain command conditions*/

    long    std_ownerGcsId;         /*!< @brief GCS id of the owner of this UAV ( 0 = no owner ) */
    double    std_gpsAlt;             /*!< @brief GPS altitude of the plane (-8*ft - integer scaled negative feet) */
    long    std_mpMode;    /*!< @brief Mode of the autopilot, shifted left by 4 bits. */
    long    std_lrcStatus; /*!< @brief Status of the LRC */
	//======End of Std Telem Data translated

	//===User Telemetry Data decripted =========
	//==========================================
	// 5Hz Data
	double User_GPS_Time;
	double User_BodyRateYDot_rad2s;
	double User_BodyRateXDot_rad2s;
	double User_CurrentAltitude_m;
	double User_AirSpeed_m2s;
	double User_Pitch_rad;
	double User_Roll_rad;

	double User_CompasHeading_rad;
	double User_GPS_Velocity_East_m2s;
	double User_GPS_Velocity_North_m2s;
	double User_Yaw_rad;
	double User_GPS_Longitude_rad;
	double User_GPS_Latitude_rad;
	double User_GPS_Altitude_ASL_m;
	double User_BodyRateZDot_rad2s;
	double User_GPS_Velocity_Up_m2s;

	// 1Hz Data
	// id=0
	int User_CameraTargetEnabled; // Camera Target Enable. 0:Off,1:Absolute,2:Relative
	double User_CameraTargetLongitude_rad; //Camera Target EAST Co-ordinates
	double User_CameraTargetLatitude_rad; //Camera Target North Co-ordinates
	double User_CameraTargetAltitude_m; //Camera Target UP Co-ordinates 
	long User_throttle_Servo1_raw;
	long User_Elevator_Servo2_raw;
	long User_Servo20_raw; //Roll Rate
	long User_Servo21_raw; // Pitch Rate
	long User_Servo17_raw;
	long User_Servo18_raw;
	long User_Servo22_raw; //Zoom
	long User_Servo12_raw;

	long User_GPS_health;
	long User_GPS_Error;
	long User_GPS_Status;
	long User_GPS_Locked;

	double User_UAV_Position_Longitude_rad;
	double User_UAV_Position_Latitude_rad;

	double User_AvionicsBatVoltage_V;
	double User_ServoBatVoltage_V;
	double User_RollWithoutGPS_rad;
	double User_WindHeading_rad;
	double User_WindSpeed_m2s;
	long User_ADC_0_Temp_Raw;
	long User_ADC_1;
	long User_ADC_2;
	long User_ADC_3;
	double User_TargetAltitude;
	double User_DesiredRoll_rad;
	double User_TargetSpeed_m2s;
	double User_DesiredAltitude_m;
	double User_Inflight_Failure_raw;
	double User_MainBatVoltage_V;
	double User_MainBatCurrent_A;
	int   User_Step;
	double User_PitchDot_Alternative_rad2s;

}APTelemetryDataStruct;
#endif

