// accelerometer
float cAx= 0.585    ,cAxx= -1.0/548.0 ,cAxy= 0        ,cAxz= 0;
float cAy= 0.3      ,cAyx= 0          ,cAyy= 1/558.0  ,cAyz= 0;
float cAz= -0.5624  ,cAzx= 0          ,cAzy= 0        ,cAzz= 1/567.0;
// gyro
float cGx=  -3.66303e-002 , cGxx=   9.65170e-004 , cGxy=   4.02341e-005 , cGxz=   1.42294e-005;
float cGy=   2.35624e-002 , cGyx=  -5.18345e-005 , cGyy=   1.17287e-003 , cGyz=   1.41965e-005;
float cGz=   4.20406e-003 , cGzx=  -1.09106e-005 , cGzy=   5.37563e-006 , cGzz=   1.16814e-003;
#ifndef NO_MAGNETOMETER
float cMx=  -2.31756e+001 , cMxx=   7.37373e-001 , cMxy=   4.95486e-002 , cMxz=   1.44887e-002;
float cMy=   6.14254e+001 , cMyx=  -5.22259e-002 , cMyy=   7.85282e-001 , cMyz=  -3.01566e-002;
float cMz=   4.12797e+001 , cMzx=  -9.73106e-003 , cMzy=  -3.64315e-003 , cMzz=   6.76591e-001;
#endif
