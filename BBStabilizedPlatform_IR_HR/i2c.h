#include "mtype.h"

BYTE I2C_Read_Bit(void);
void I2C_Write_Bit(BYTE bit);
void I2C_Start();
void I2C_Stop();
BYTE I2C_Ack();
void I2CSendByte(BYTE data);
BYTE I2CReceiveByte(void);
BYTE I2CReceiveLastByte(void);
void WasteTime(HWORD l);




