/*********************************************************************
 
  (c) copyright Freescale Semiconductor Ltd. 2011
  ALL RIGHTS RESERVED
 
 *********************************************************************
 
 *********************************************************************

  File:				        MAG3110_Read_and_Filter.c
 
  Description:        Read data from MAG3110
    
********************************************************************
 Ver 1.0.0	Released Jun.2011
********************************************************************/


/*
  Copyright BlueBird Aero Systems Engineering Ltd 2011

  $Id: ITG3200.c,v 1.6 2012/02/13 17:44:11 EranS Exp $
  $Log: ITG3200.c,v $

  Revision 1.1  2012/02/13 12:23:45  EranS
  First save

 */
#include <math.h>
#include <string.h>
#include "bluebird.h"
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
//#include "inc/hw_nvic.h"
#include "inc/hw_types.h"
#include "driverlib/debug.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/sysctl.h"
#include "MasterI2C.h"
#include "utilities.h"
#define MAG_MAIN
#include "Pins.h"
#include "MAG3110.h"
#include "GPIOhandler.h"


//#include "MAG3110_Read_and_Filter.h" /* include register declarations */


VUINT8 MAG3110_CTRL_REG1_Value;
VUINT8 MAG3110_CTRL_REG2_Value;
VUINT8 MAG3110_OFF_X_MSB_Value;
VUINT8 MAG3110_OFF_X_LSB_Value;
VUINT8 MAG3110_OFF_Y_MSB_Value;
VUINT8 MAG3110_OFF_Y_LSB_Value;
VUINT8 MAG3110_OFF_Z_MSB_Value;
VUINT8 MAG3110_OFF_Z_LSB_Value;

s16DataUnion s16magXunion;
#define s16magXnew s16magXunion.s16word
#define s8magXmsb s16magXunion.Bytes.s8msb
#define u8magXlsb s16magXunion.Bytes.u8lsb

s16DataUnion s16magYunion;
#define s16magYnew s16magYunion.s16word
#define s8magYmsb s16magYunion.Bytes.s8msb
#define u8magYlsb s16magYunion.Bytes.u8lsb

s16DataUnion s16magZunion;
#define s16magZnew s16magZunion.s16word
#define s8magZmsb s16magZunion.Bytes.s8msb
#define u8magZlsb s16magZunion.Bytes.u8lsb

s16DataUnion s16magXfltunion;
#define s16magXflt s16magXfltunion.s16word
#define s8magXfltmsb s16magXfltunion.Bytes.s8msb
#define u8magXfltlsb s16magXfltunion.Bytes.u8lsb

s16DataUnion s16magYfltunion;
#define s16magYflt s16magYfltunion.s16word
#define s8magYfltmsb s16magYfltunion.Bytes.s8msb
#define u8magYfltlsb s16magYfltunion.Bytes.u8lsb

s16DataUnion s16magZfltunion;
#define s16magZflt s16magZfltunion.s16word
#define s8magZfltmsb s16magZfltunion.Bytes.s8msb
#define u8magZfltlsb s16magZfltunion.Bytes.u8lsb

VINT16 s16MagXfltRsdl;
VINT16 s16MagYfltRsdl;
VINT16 s16MagZfltRsdl;


void MAG3110Init()
{
    // sanity check that we are talking    
    while (magregs[MAG3110_WHO_AM_I] != 0xC4)
      MasterI2C0Read(MAG3110_ADD,&magregs[MAG3110_WHO_AM_I],MAG3110_WHO_AM_I,1);    // read ITG3200_WHO_AM_I

	magregs[MAG3110_CTRL_REG1] = 0;
    MasterI2C0Write(MAG3110_ADD,&magregs[MAG3110_OFF_X_MSB], (unsigned long)MAG3110_CTRL_REG1,1L);

    MasterI2C0Write(MAG3110_ADD,&magregs[MAG3110_OFF_X_MSB], (unsigned long)MAG3110_OFF_X_MSB,1L);
    MasterI2C0Write(MAG3110_ADD,&magregs[MAG3110_OFF_X_LSB], (unsigned long)MAG3110_OFF_X_LSB,1L);
    MasterI2C0Write(MAG3110_ADD,&magregs[MAG3110_OFF_Y_MSB], (unsigned long)MAG3110_OFF_Y_MSB,1L);
    MasterI2C0Write(MAG3110_ADD,&magregs[MAG3110_OFF_Y_LSB], (unsigned long)MAG3110_OFF_Y_LSB,1L);
    MasterI2C0Write(MAG3110_ADD,&magregs[MAG3110_OFF_Z_MSB], (unsigned long)MAG3110_OFF_Z_MSB,1L);
    MasterI2C0Write(MAG3110_ADD,&magregs[MAG3110_OFF_Z_LSB], (unsigned long)MAG3110_OFF_Z_LSB,1L);


	magregs[MAG3110_CTRL_REG2] = 0x80;

	MasterI2C0Write(MAG3110_ADD,&magregs[MAG3110_CTRL_REG2], (unsigned long)MAG3110_CTRL_REG2,1L);
	//													b00000000 :Default
	//													 || |
	//													 || +----- Mag_RST:
	//													 ||
	//													 |+------- RAW
	//													 +-------- AUTO_MRST_EN

	magregs[MAG3110_CTRL_REG1] = 0x1;
	MasterI2C0Write(MAG3110_ADD,&magregs[MAG3110_CTRL_REG1], (unsigned long)MAG3110_CTRL_REG1,1L);
	//													b00000000 :Default
	//													 ||||||||
	//													 |||||||+- AC
	//													 ||||||+-- TM
	//													 |||||+--- FR
	//													 |||++---- OS1:OS0
	//													 +++------ DR2:DR0

#ifdef USE_MAG3110_INTERRUPT
    magready = false;
    GPIOPinInit(MAG3110_INTERRUPT_PORT,MAG3110_INTERRUPT_PIN,true,true);
#endif
}


void MAG3110Action(void)
{
  magready = true;
}



void MAG3110_XYZ_Read_and_Filter(void){

  VINT16 s16temp, s16temp2;
  
  (void)IIC_Read_MAG3110_XYZ16(&s16magXnew, &s16magYnew, &s16magZnew);
  
  s16temp = s16magXnew - s16magXflt;
  if(s16temp < 0) s16temp2 = 0 - ((0 - s16temp) >> MAG_FilterStrength);
  else s16temp2 = (s16temp >> MAG_FilterStrength);
  s16magXflt += s16temp2;
  MAG3110values[0] += (s16temp  - (s16temp2 << MAG_FilterStrength));
  if(MAG3110values[0] >= MAG_RSDL_POS)
  {
    MAG3110values[0] = 0;
    s16magXflt++;
  }
  else if(MAG3110values[0] <= MAG_RSDL_NEG)
  {
    MAG3110values[0] = 0;
    s16magXflt--;
  }

  s16temp = s16magYnew - s16magYflt;
  if(s16temp < 0) s16temp2 =  0 - ((0 - s16temp) >> MAG_FilterStrength);
  else s16temp2 = (s16temp >> MAG_FilterStrength);
  s16magYflt += s16temp2;
  MAG3110values[1] += (s16temp  - (s16temp2 << MAG_FilterStrength));
  if(MAG3110values[1] >= MAG_RSDL_POS)
  {
    MAG3110values[1] = 0;
    s16magYflt++;
  }
  else if(MAG3110values[1] <= MAG_RSDL_NEG)
  {
    MAG3110values[1] = 0;
    s16magYflt--;
  }

  s16temp = s16magZnew - s16magZflt;
  if(s16temp < 0) s16temp2 = 0 - ((0 - s16temp) >> MAG_FilterStrength);
  else s16temp2 = (s16temp >> MAG_FilterStrength);
  s16magZflt += s16temp2;
  MAG3110values[2] += (s16temp  - (s16temp2 << MAG_FilterStrength));
  if(MAG3110values[2] >= MAG_RSDL_POS)
  {
    MAG3110values[2] = 0;
    s16magZflt++;
  }
  else if(MAG3110values[2] <= MAG_RSDL_NEG)
  {
    MAG3110values[2] = 0;
    s16magZflt--;
  }
  
}


// read X/Y/Z registers and swap endian
void IIC_Read_MAG3110_XYZ16(VINT16 *pX, VINT16 *pY, VINT16 *pZ)
{
  //int i;
  //VUINT8 res;
  VUINT8 u8Array[6];

  //res = ERR_OK;

  MasterI2C0Read(MAG3110_ADD,&u8Array[0],MAG3110_CTRL_REG1,1); 

  if(u8Array[0] & 0x04)
  {
	MasterI2C0Read(MAG3110_ADD,&u8Array[0],MAG3110_OUT_X_MSB,3);   
    //res = IIC_ReadBlock(MAG3110_ADD, MAG3110_OUT_X_MSB, 3, u8Array);

	MasterI2C0Read(MAG3110_ADD,&u8Array[3],MAG3110_OUT_X_LSB,1); 
	//	  res = IIC_ReadBlock(MAG3110_ADD, MAG3110_OUT_X_LSB, 1, &u8Array[3]);	


	MasterI2C0Read(MAG3110_ADD,&u8Array[4],MAG3110_OUT_Y_LSB,1); 
    //res = IIC_ReadBlock(MAG3110_ADD, MAG3110_OUT_Y_LSB, 1, &u8Array[4]);

	MasterI2C0Read(MAG3110_ADD,&u8Array[5],MAG3110_OUT_Z_LSB,1); 
    //res = IIC_ReadBlock(MAG3110_ADD, MAG3110_OUT_Z_LSB, 1, &u8Array[5]);

    *pX = (u8Array[0] << 8) | u8Array[3];
    *pY = (u8Array[1] << 8) | u8Array[4];
    *pZ = (u8Array[2] << 8) | u8Array[5];
  }
  else
  {
	MasterI2C0Read(MAG3110_ADD,&u8Array[0],MAG3110_OUT_X_MSB,6);    
    //res = IIC_ReadBlock(MAG3110_ADD, MAG3110_OUT_X_MSB, 6, u8Array);
    *pX = (u8Array[0] << 8) | u8Array[1];
    *pY = (u8Array[2] << 8) | u8Array[3];
    *pZ = (u8Array[4] << 8) | u8Array[5];
  }
  
  //memcpy(MAG3110values,&magregs[OUT_X_MSB],6);
  //for (i=0; i<3; i++)
    //swapb((unsigned short *)&MAG3110values[i]);
}



// read X/Y/Z registers and swap endian
void Read_MAG3110_XYZ16(void)
{
  //int i;
  //VUINT8 res;
  VUINT8 u8Array[6];

  //res = ERR_OK;

  MasterI2C0Read(MAG3110_ADD,&u8Array[0],MAG3110_CTRL_REG1,1); 

  if(u8Array[0] & 0x04)
  {
	MasterI2C0Read(MAG3110_ADD,&u8Array[0],MAG3110_OUT_X_MSB,3);   
    //res = IIC_ReadBlock(MAG3110_ADD, MAG3110_OUT_X_MSB, 3, u8Array);

	MasterI2C0Read(MAG3110_ADD,&u8Array[3],MAG3110_OUT_X_LSB,1); 
	//	  res = IIC_ReadBlock(MAG3110_ADD, MAG3110_OUT_X_LSB, 1, &u8Array[3]);	


	MasterI2C0Read(MAG3110_ADD,&u8Array[4],MAG3110_OUT_Y_LSB,1); 
    //res = IIC_ReadBlock(MAG3110_ADD, MAG3110_OUT_Y_LSB, 1, &u8Array[4]);

	MasterI2C0Read(MAG3110_ADD,&u8Array[5],MAG3110_OUT_Z_LSB,1); 
    //res = IIC_ReadBlock(MAG3110_ADD, MAG3110_OUT_Z_LSB, 1, &u8Array[5]);

    MAG3110values[0] = (u8Array[0] << 8) | u8Array[3];
    MAG3110values[1] = (u8Array[1] << 8) | u8Array[4];
    MAG3110values[2] = (u8Array[2] << 8) | u8Array[5];
  }
  else
  {
	MasterI2C0Read(MAG3110_ADD,&u8Array[0],MAG3110_OUT_X_MSB,6);    
    //res = IIC_ReadBlock(MAG3110_ADD, MAG3110_OUT_X_MSB, 6, u8Array);
    MAG3110values[0] = (u8Array[0] << 8) | u8Array[1];
    MAG3110values[1] = (u8Array[2] << 8) | u8Array[3];
    MAG3110values[2] = (u8Array[4] << 8) | u8Array[5];
  }
  
  //memcpy(MAG3110values,&magregs[OUT_X_MSB],6);
  //for (i=0; i<3; i++)
    //swapb((unsigned short *)&MAG3110values[i]);
}


/*
I calculated the heading with:
float heading = atan2(readx(), ready());
float headingDegrees = heading * 180/PI;
and i get these values:
x=3398,y=-5064,z=-769
heading=146.10.
what do you think it could be the problem?

I tried to use this to calculate heading and i have for a full rotation 5 degrees of difference, between 55 and 60 degree
*/
int getHeading(float x, float y, float z){  
  float heading=0;  
  if ((x == 0)&&(y < 0))  
    heading= PI/2.0;  
  if ((x == 0)&&(y > 0))  
    heading=3.0*PI/2.0;  
  if (x < 0)  
    heading = PI - atan(y/x);  
  if ((x > 0)&&(y < 0))  
    heading = -atan(y/x);  
  if ((x > 0)&&(y > 0))  
    heading = 2.0*PI - atan(y/x); 
  
  return  (int)(heading*RAD2DEG);  
  	
  //return  int(degrees(heading));  
}


#if 0
void SetMGAReg(unsigned long mask, unsigned long val, unsigned char reg)
{
   magregs[reg] &= ((~mask) | val);
   magregs[reg] |= (mask & val); 
}

void MAG3110Init()
{
    // sanity check that we are talking    
    while (magregs[MAG_WHO_AM_I] != 0xC4)
      MasterI2C0Read(MAG3110_SLAVE_ADDRESS,&magregs[MAG_WHO_AM_I],MAG_WHO_AM_I,1);    // read ITG3200_WHO_AM_I

//    MasterI2C0Read(MAG3110_SLAVE_ADDRESS,&magregs[CTRL_REG2],CTRL_REG1,1);    // read ITG3200_WHO_AM_I
//    MasterI2C0Write(MAG3110_SLAVE_ADDRESS,&magregs[CTRL_REG2], (unsigned long)0x80,1L);
//    MasterI2C0Read(MAG3110_SLAVE_ADDRESS,&magregs[CTRL_REG2],CTRL_REG2,1);    // read ITG3200_WHO_AM_I

//    MasterI2C0Read(MAG3110_SLAVE_ADDRESS,&magregs[CTRL_REG1],CTRL_REG1,1);    // read ITG3200_WHO_AM_I
//    MasterI2C0Write(MAG3110_SLAVE_ADDRESS,&magregs[CTRL_REG1], (unsigned long)0x1,1L);
//    MasterI2C0Read(MAG3110_SLAVE_ADDRESS,&magregs[CTRL_REG1],CTRL_REG1,1);    // read ITG3200_WHO_AM_I

	magregs[CTRL_REG2] &= ((~0x80) | 0x80);
	magregs[CTRL_REG2] |= (0x80 & 0x80);
	//magregs[CTRL_REG2] = 0x80;
    MasterI2C0Write(MAG3110_SLAVE_ADDRESS,&magregs[CTRL_REG2], (unsigned long)CTRL_REG2,1L);

	microsecdelay(100000000);

    MasterI2C0Read(MAG3110_SLAVE_ADDRESS,&magregs[CTRL_REG1],CTRL_REG1,1);    // read ITG3200_WHO_AM_I
	magregs[CTRL_REG1] &= ((~0x1) | 0x1);
	magregs[CTRL_REG1] |= (0x1A & 0x1);  
    MasterI2C0Write(MAG3110_SLAVE_ADDRESS,&magregs[CTRL_REG1], (unsigned long)CTRL_REG1,1L);

    //MasterI2C0Read(MAG3110_SLAVE_ADDRESS,&magregs[CTRL_REG2],CTRL_REG2,1);    // read ITG3200_WHO_AM_I	
    //MasterI2C0Read(MAG3110_SLAVE_ADDRESS,&magregs[CTRL_REG1],CTRL_REG1,1);    // read ITG3200_WHO_AM_I

	#if 0
    // set full scale and low pass filter
    magregs[DLPF_FS] = FS_SEL + Hz188;
    MasterI2C0Write(MAG3110_SLAVE_ADDRESS,&magregs[DLPF_FS], (unsigned long)DLPF_FS,1L);
    // congfigure ITG3200 to interrupt at desired rate
    magregs[SMPLRT_DIV] = (1000/ITG3200_SAMPLE_RATE)-1;
    MasterI2C0Write(MAG3110_SLAVE_ADDRESS,&magregs[SMPLRT_DIV], (unsigned long)SMPLRT_DIV,1L);
    // sanity check that I wrote OK
    MasterI2C0Read(MAG3110_SLAVE_ADDRESS,&magregs[SMPLRT_DIV],SMPLRT_DIV,1);
    while (magregs[SMPLRT_DIV] != ((1000/ITG3200_SAMPLE_RATE)-1))
      MasterI2C0Read(MAG3110_SLAVE_ADDRESS,&magregs[0],SMPLRT_DIV,1);

	
    magregs[INT_CFG] = RAW_RDY_EN;
    MasterI2C0Write(MAG3110_SLAVE_ADDRESS,&magregs[INT_CFG], (unsigned long)INT_CFG,1L);
    MasterI2C0Read(MAG3110_SLAVE_ADDRESS,&magregs[INT_CFG],INT_CFG,1);
    while (magregs[INT_CFG] != RAW_RDY_EN)
    	 MasterI2C0Read(MAG3110_SLAVE_ADDRESS,&magregs[INT_CFG],INT_CFG,1);
	#endif
#ifdef USE_MAG3110_INTERRUPT
    magready = false;
    GPIOPinInit(MAG3110_INTERRUPT_PORT,MAG3110_INTERRUPT_PIN,true,true);
	//microsecdelay(100000000);	
	MAG3110getXYZ();
#endif
}

void MAG3110Action()
{
  magready = true;
}

// read X/Y/Z registers and swap endian
void MAG3110getXYZ()
{
  int i;
#ifndef USE_MAG3110_INTERRUPT
  // poll status until raw data ready
  magregs[DR_STATUS] = 0;
  while (!(magregs[DR_STATUS] & ZYXDR))
    MasterI2C0Read(MAG3110_SLAVE_ADDRESS,&magregs[DR_STATUS],DR_STATUS,1);  // read status
#endif
  MasterI2C0Read(MAG3110_SLAVE_ADDRESS,&magregs[OUT_X_MSB],OUT_X_MSB,6);    // read X,Y,Z
  memcpy(MAG3110values,&magregs[OUT_X_MSB],6);
  for (i=0; i<3; i++)
    swapb((unsigned short *)&MAG3110values[i]);
}

/*
I calculated the heading with:
float heading = atan2(readx(), ready());
float headingDegrees = heading * 180/PI;
and i get these values:
x=3398,y=-5064,z=-769
heading=146.10.
what do you think it could be the problem?

I tried to use this to calculate heading and i have for a full rotation 5 degrees of difference, between 55 and 60 degree
*/
int getHeading(float x, float y, float z){  
  float heading=0;  
  if ((x == 0)&&(y < 0))  
    heading= PI/2.0;  
  if ((x == 0)&&(y > 0))  
    heading=3.0*PI/2.0;  
  if (x < 0)  
    heading = PI - atan(y/x);  
  if ((x > 0)&&(y < 0))  
    heading = -atan(y/x);  
  if ((x > 0)&&(y > 0))  
    heading = 2.0*PI - atan(y/x); 
  
  return  (int)(heading*RAD2DEG);  
  	
  //return  int(degrees(heading));  
}


// read X/Y/Z registers and swap endian
void MAG3110getX()
{
  int i;
#ifndef USE_MAG3110_INTERRUPT
  // poll status until raw data ready
  magregs[DR_STATUS] = 0;
  while (!(magregs[DR_STATUS] & XDR))
    MasterI2C0Read(MAG3110_SLAVE_ADDRESS,&magregs[DR_STATUS],DR_STATUS,1);  // read status
#endif
  MasterI2C0Read(MAG3110_SLAVE_ADDRESS,&magregs[OUT_X_MSB],OUT_X_MSB,2);    // read X
  memcpy(MAG3110values,&magregs[OUT_X_MSB],2);
  for (i=0; i<1; i++)
    swapb((unsigned short *)&MAG3110values[i]);
}
#endif

#if 0
/*
  MAG3110 Breakout Example Code
  
  by: Aaron Weiss, aaron at sparkfun dot com
      SparkFun Electronics 2011
  date: 9/6/11
  license: beerware, if you use this code and happen to meet me, you
           can by me a beer

  The code reads the raw 16-bit x, y, and z values and prints them 
  out. This sketch does not use the INT1 pin, nor does it poll for
  new data.

*/

#include <Wire.h>

#define MAG_ADDR  0x0E //7-bit address for the MAG3110, doesn't change

void setup()
{
  Wire.begin();        // join i2c bus (address optional for master)
  Serial.begin(9600);  // start serial for output
  config();            // turn the MAG3110 on
}

void loop()
{
  print_values();
  delay(5);
}

void config(void)
{
  Wire.beginTransmission(MAG_ADDR); // transmit to device 0x0E
  Wire.send(0x11);              // cntrl register2
  Wire.send(0x80);              // send 0x80, enable auto resets
  Wire.endTransmission();       // stop transmitting
  
  delay(15);
  
  Wire.beginTransmission(MAG_ADDR); // transmit to device 0x0E
  Wire.send(0x10);              // cntrl register1
  Wire.send(1);                 // send 0x01, active mode
  Wire.endTransmission();       // stop transmitting
}

void print_values(void)
{
  Serial.print("x=");
  Serial.print(readx()); 
  Serial.print(",");  
  Serial.print("y=");    
  Serial.print(ready());
  Serial.print(",");       
  Serial.print("z=");    
  Serial.println(readz());      
}

int readx(void)
{
  int xl, xh;  //define the MSB and LSB
  
  Wire.beginTransmission(MAG_ADDR); // transmit to device 0x0E
  Wire.send(0x01);              // x MSB reg
  Wire.endTransmission();       // stop transmitting
 
  delayMicroseconds(2); //needs at least 1.3us free time between start and stop
  
  Wire.requestFrom(MAG_ADDR, 1); // request 1 byte
  while(Wire.available())    // slave may send less than requested
  { 
    xh = Wire.receive(); // receive the byte
  }
  
  delayMicroseconds(2); //needs at least 1.3us free time between start and stop
  
  Wire.beginTransmission(MAG_ADDR); // transmit to device 0x0E
  Wire.send(0x02);              // x LSB reg
  Wire.endTransmission();       // stop transmitting
 
  delayMicroseconds(2); //needs at least 1.3us free time between start and stop
  
  Wire.requestFrom(MAG_ADDR, 1); // request 1 byte
  while(Wire.available())    // slave may send less than requested
  { 
    xl = Wire.receive(); // receive the byte
  }
  
  int xout = (xl|(xh << 8)); //concatenate the MSB and LSB
  return xout;
}

int ready(void)
{
  int yl, yh;  //define the MSB and LSB
  
  Wire.beginTransmission(MAG_ADDR); // transmit to device 0x0E
  Wire.send(0x03);              // y MSB reg
  Wire.endTransmission();       // stop transmitting
 
  delayMicroseconds(2); //needs at least 1.3us free time between start and stop
  
  Wire.requestFrom(MAG_ADDR, 1); // request 1 byte
  while(Wire.available())    // slave may send less than requested
  { 
    yh = Wire.receive(); // receive the byte
  }
  
  delayMicroseconds(2); //needs at least 1.3us free time between start and stop
  
  Wire.beginTransmission(MAG_ADDR); // transmit to device 0x0E
  Wire.send(0x04);              // y LSB reg
  Wire.endTransmission();       // stop transmitting
 
  delayMicroseconds(2); //needs at least 1.3us free time between start and stop
  
  Wire.requestFrom(MAG_ADDR, 1); // request 1 byte
  while(Wire.available())    // slave may send less than requested
  { 
    yl = Wire.receive(); // receive the byte
  }
  
  int yout = (yl|(yh << 8)); //concatenate the MSB and LSB
  return yout;
}

int readz(void)
{
  int zl, zh;  //define the MSB and LSB
  
  Wire.beginTransmission(MAG_ADDR); // transmit to device 0x0E
  Wire.send(0x05);              // z MSB reg
  Wire.endTransmission();       // stop transmitting
 
  delayMicroseconds(2); //needs at least 1.3us free time between start and stop
  
  Wire.requestFrom(MAG_ADDR, 1); // request 1 byte
  while(Wire.available())    // slave may send less than requested
  { 
    zh = Wire.receive(); // receive the byte
  }
  
  delayMicroseconds(2); //needs at least 1.3us free time between start and stop
  
  Wire.beginTransmission(MAG_ADDR); // transmit to device 0x0E
  Wire.send(0x06);              // z LSB reg
  Wire.endTransmission();       // stop transmitting
 
  delayMicroseconds(2); //needs at least 1.3us free time between start and stop
  
  Wire.requestFrom(MAG_ADDR, 1); // request 1 byte
  while(Wire.available())    // slave may send less than requested
  { 
    zl = Wire.receive(); // receive the byte
  }
  
  int zout = (zl|(zh << 8)); //concatenate the MSB and LSB
  return zout;
}

/*
I calculated the heading with:
float heading = atan2(readx(), ready());
float headingDegrees = heading * 180/PI;
and i get these values:
x=3398,y=-5064,z=-769
heading=146.10.
what do you think it could be the problem?

I tried to use this to calculate heading and i have for a full rotation 5 degrees of difference, between 55 and 60 degree
*/
int getHeading(float x, float y, float z){  
  float heading=0;  
  if ((x == 0)&&(y < 0))  
    heading= PI/2.0;  
  if ((x == 0)&&(y > 0))  
    heading=3.0*PI/2.0;  
  if (x < 0)  
    heading = PI - atan(y/x);  
  if ((x > 0)&&(y < 0))  
    heading = -atan(y/x);  
  if ((x > 0)&&(y > 0))  
    heading = 2.0*PI - atan(y/x);  
  return  int(degrees(heading));  
}

/*

Your problem is due to calibration.
You need to correct the offset values, either by writing to the offset registers of the device
(OFF_X_MSB ,etc) or by subtracting the offset from the measurements you get. 
This is caused by hard iron effects.
Freescale has an application note (AN4246, i think) which explains this effect and how to correct it

*/

/*
You can correct the measurements by subtracting an offset from the measurements
( the offset for each axis is (max_measurement + min_measurement)/2 ).
This should give you good enough readings.
But if you want to be more accurate then you can collect readings while rotating the magnetometer in every direction and fit the measurements to a ellipse.
An ellipse is described by the equation x'Ax = 1, where x are your measurements.
You need to find, using least squares, the values for the matrix A. After that,
you can find the transformation that rotates and scales the ellipse
into a circle by taking the Cholesky decomposition of A = L'L.
With that, for every measurement x you get, you can get x_corrected = L*(x-offset_vector),
and your measurements of the heading will be much more accurate.
The only thing that is missing is to add an accelerometer to compensate for tilt


*/
#endif
