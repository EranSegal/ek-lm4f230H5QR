/*************************************************************
 * 			On IOxDirection == Input
 * 					IOxMask 1 == Interrupt Disabled
 * 					IOxMask 0 == Interrupt Enabled
 *
 *			On IOxDirection == Output
 *  					1 == IO2 OutPut Reflects IO2Polarity
 *                                0 == Reserved
 **************************************************************/           
#include <sys/hyrtk.h>

#ifdef _DAVICOM_CHIP_
#define HY_E1_IO_WAIT(VALUE)       HY_E1_IO_WAIT##_##VALUE
#define HY_E1_IO_WAIT_DISABLED     (0x00000000LL)
#define HY_E1_IO_WAIT_ENABLED      (0x00000800LL)

#define HY_E1_IO_STROBE(VALUE)     HY_E1_IO_STROBE##_##VALUE
#define HY_E1_IO_STROBE_IOCONTROL  (0x00000000LL)
#define HY_E1_IO_STROBE_RWCONTROL  (0x00000400LL)

#define HY_E1_IO_SETUP(VALUE)      HY_E1_IO_SETUP##_##VALUE
#define HY_E1_IO_SETUP_0CYCLES     (0x00000000LL)
#define HY_E1_IO_SETUP_2CYCLES     (0x00000100LL)
#define HY_E1_IO_SETUP_4CYCLES     (0x00000200LL)
#define HY_E1_IO_SETUP_6CYCLES     (0x00000300LL)

#define HY_E1_IO_ACCESS(VALUE)     HY_E1_IO_ACCESS##_##VALUE
#define HY_E1_IO_ACCESS_2CYCLES    (0x00000000LL)
#define HY_E1_IO_ACCESS_4CYCLES    (0x00000020LL)
#define HY_E1_IO_ACCESS_6CYCLES    (0x00000040LL)
#define HY_E1_IO_ACCESS_8CYCLES    (0x00000060LL)
#define HY_E1_IO_ACCESS_10CYCLES   (0x00000080LL)
#define HY_E1_IO_ACCESS_12CYCLES   (0x000000A0LL)
#define HY_E1_IO_ACCESS_14CYCLES   (0x000000C0LL)
#define HY_E1_IO_ACCESS_16CYCLES   (0x000000E0LL)

#define HY_E1_IO_HOLD(VALUE)       HY_E1_IO_HOLD##_##VALUE
#define HY_E1_IO_HOLD_0CYCLES      (0x00000000LL)
#define HY_E1_IO_HOLD_2CYCLES      (0x00000008LL)
#define HY_E1_IO_HOLD_4CYCLES      (0x00000010LL)
#define HY_E1_IO_HOLD_6CYCLES      (0x00000018LL)

#define NSR_REG 	0x01
#endif


#define HW_INT_UART_CS	   0x003D0000u			   // ADB addr-space !
#define HW_EXT_UART_CS	   0x003B0000u			   // ADB addr-space !

/*
 * Interrupt Disabled
 */
#define IO1INTERRUPTDES()	UpdateFCR( 	1<<1,	0		)
#define IO2INTERRUPTDES()	UpdateFCR(	1 <<4,   0		)
#define IO3INTERRUPTDES() 	UpdateFCR( 	1<<8,	0		)
/*
 * Interrupt Enabled
 */
#define IO1INTERRUPTENA()	 UpdateFCR(0, 1<<1)
#define IO2INTERRUPTENA()	 UpdateFCR(0, 1<<4)
#define IO3INTERRUPTENA()	 UpdateFCR(0, 1<<8)

/*
 * IOxDirection Input
 */
#define IO1INPUTDIRECTION()		UpdateFCR(	1<<2,	0		)
#define IO2INPUTDIRECTION()		UpdateFCR(	1<<6,	0		)
#define IO3INPUTDIRECTION()		UpdateFCR(	1<<10,	0		)
/*
 * IOxDirection Output
 */
#define IO1OUTPUTDIRECTION()		UpdateFCR(	0,		1<<2	)
#define IO2OUTPUTDIRECTION()		UpdateFCR(	0,		1<<6	)
#define IO3OUTPUTDIRECTION()		UpdateFCR(	0,		1<<10	)


#define WRITE_SDA_High() {UpdateFCR(0,1<<10);UpdateFCR(1<<9,0);} 
#define WRITE_SDA_Low()  {UpdateFCR(0,1<<10);UpdateFCR(0,1<<9);}

#define I2C_Clock_High()	{UpdateFCR(0,1<<6);UpdateFCR(1<<5,0);}
#define I2C_Clock_Low() 	{UpdateFCR(0,1<<6);UpdateFCR(0,1<<5);}

  
#define DALLAS_HIGH() 	UpdateFCR(0x3,0x4)
#define DALLAS_LOW()	UpdateFCR(0x1,0x6)

#define DALLAS_WRITE(bit) ( (bit) ? DALLAS_HIGH() : DALLAS_LOW() )
 
/*
 * Reflects IO1 Pin in FCR
 */
#define DALLAS_READ()	  ( GetISR() & 0x10)   

/*
 * Reflects IO2 Pin in FCR
 */
#define IO2SCL() 				( GetISR() & 0x20)   
/*
 * Reflects IO3 Pin in FCR
 */
#define IO3SDA() 				(GetISR() & 0x40)   

/*
 * Reflects INT1 Pin in FCR
 */
#define INTERRUPT1()               ( GetISR() & 0x01)
/*
 * Reflects INT2 Pin in FCR
 */
#define INTERRUPT2()			( GetISR() & 0x02)
/*
 * Reflects INT3 Pin in FCR
 */
#define INTERRUPT3()			( GetISR() & 0x04)
/*
 * Reflects INT4 Pin in FCR
 */
#define INTERRUPT4()		       ( GetISR() & 0x08)

/*
 * Write bit 1 to Dallas  
 */
#define WRITE_DALLAS_HIGH()   	{ IO1OUTPUTDIRECTION(); UpdateFCR(1<<1,0);}
/*
  * Write bit 0  to Dallas
  */
#define WRITE_DALLAS_LOW()    	{ IO1OUTPUTDIRECTION(); UpdateFCR(0,1<<1);}

/*
  * Write bit 1  to SDA
  */
#define WRITE_SDA_HIGH()     {UpdateFCR(1<<9, 1<<10);}
 
/*
 * Write bit 0  to SDA
 */
#define WRITE_SDA_LOW()      {UpdateFCR(0, 1<<9|1<<10);}

/*
 * Write bit 1  to SCL
 */
#define I2C_CLOCK_HIGH()     {UpdateFCR(1<<5, 1<<6);}
	
/*
 * Write bit 0  to SCL
 */
#define I2C_CLOCK_LOW()      {UpdateFCR(0, 1<<5|1<<6);}


#define E1XS_IO_WAIT(VALUE)       E1XS_IO_WAIT##_##VALUE
#define E1XS_IO_WAIT_DISABLED     (0x00000000LL)
#define E1XS_IO_WAIT_ENABLED      (0x00000800LL)

#define E1XS_IO_STROBE(VALUE)     E1XS_IO_STROBE##_##VALUE
#define E1XS_IO_STROBE_IOCONTROL  (0x00000000LL)
#define E1XS_IO_STROBE_RWCONTROL  (0x00000400LL)

#define E1XS_IO_SETUP(VALUE)      E1XS_IO_SETUP##_##VALUE
#define E1XS_IO_SETUP_0CYCLES     (0x00000000LL)
#define E1XS_IO_SETUP_2CYCLES     (0x00000100LL)
#define E1XS_IO_SETUP_4CYCLES     (0x00000200LL)
#define E1XS_IO_SETUP_6CYCLES     (0x00000300LL)

#define E1XS_IO_ACCESS(VALUE)     E1XS_IO_ACCESS##_##VALUE
#define E1XS_IO_ACCESS_2CYCLES    (0x00000000LL)
#define E1XS_IO_ACCESS_4CYCLES    (0x00000020LL)
#define E1XS_IO_ACCESS_6CYCLES    (0x00000040LL)
#define E1XS_IO_ACCESS_8CYCLES    (0x00000060LL)
#define E1XS_IO_ACCESS_10CYCLES   (0x00000080LL)
#define E1XS_IO_ACCESS_12CYCLES   (0x000000A0LL)
#define E1XS_IO_ACCESS_14CYCLES   (0x000000C0LL)
#define E1XS_IO_ACCESS_16CYCLES   (0x000000E0LL)

#define E1XS_IO_HOLD(VALUE)       E1XS_IO_HOLD##_##VALUE
#define E1XS_IO_HOLD_0CYCLES      (0x00000000LL)
#define E1XS_IO_HOLD_2CYCLES      (0x00000008LL)
#define E1XS_IO_HOLD_4CYCLES      (0x00000010LL)
#define E1XS_IO_HOLD_6CYCLES      (0x00000018LL)



