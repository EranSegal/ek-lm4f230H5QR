/*************************************************************************
 *                             FILE: i2c.c
 *  I2C Protocol Functions
 *************************************************************************/

#include <sys/hyrtk.h>
#include "dmcdef.h"
#include "mtype.h"
#include "tw2824.h"
#include "Hy16xs.h"

void I2CSendByte(BYTE data);
void WasteTime(HWORD l);

BYTE I2C_Read_Bit(void)
{
   return IO3SDA();
}

void I2C_Write_Bit(BYTE bit)
{
   if (bit){WRITE_SDA_HIGH();}
   else{WRITE_SDA_LOW()};
}


void I2C_Start()
{
   I2C_Write_Bit(1);
   I2C_Clock_High();
   I2C_Write_Bit(0);
   I2C_Clock_Low();
}

void I2C_Stop()
{
   I2C_Clock_Low();
   I2C_Write_Bit(0);
   I2C_Clock_High();
   I2C_Write_Bit(1);
}

BYTE Transmitter_I2C_Ack()
{
   BYTE i;

   I2C_Clock_Low();
   I2C_Write_Bit(1);
   I2C_Clock_High();
   i = I2C_Read_Bit();      //-- Read data
   I2C_Clock_Low();
   return i;
}

void Receiver_I2C_Ack()
{
   I2C_Clock_Low();
   I2C_Write_Bit(0);
   I2C_Clock_High();
   I2C_Clock_Low();
   I2C_Write_Bit(1);
}

void I2CSendByte(BYTE data)
{
   HWORD i;

   for(i=0; i<8; i++)
   {
      I2C_Clock_Low();

      if(data & 0x80)
         I2C_Write_Bit(1);
      else
         I2C_Write_Bit(0);
      I2C_Clock_High();
      data <<= 1;
   }
   Transmitter_I2C_Ack();
}

BYTE I2CReceiveByte(void)
{
   HWORD i;
   BYTE data = 0;

   for(i=0; i < 8 ; i++)
   {
    I2C_Clock_Low();
    I2C_Clock_High();

    data <<= 1;
    if (I2C_Read_Bit())
     data |= 0x01;
   }
   Receiver_I2C_Ack();

   return data;
}


BYTE I2CReceiveLastByte(void)
{
   HWORD i;
   BYTE data = 0;

   for(i=0; i < 8 ; i++)
   {
    I2C_Clock_Low();
    I2C_Clock_High();

    data <<= 1;
    if (I2C_Read_Bit())
     data |= 0x01;
   }
   return data;
}



void WasteTime(HWORD l)
{
   volatile HWORD j;
   for(j=0; j<l; j++) ;
}




