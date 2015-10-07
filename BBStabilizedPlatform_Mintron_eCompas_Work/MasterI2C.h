/*
  Copyright Bluebird Aero Systems Engineering 2012

  $Id: MasterI2C.h,v 1.1 2011/03/15 12:23:45 EranS Exp $
  $Log: MasterI2C.h,v $
  Revision 1.1  2011/03/15 12:23:45  EranS
  First save


 */
void MasterI2C0Init(int ratehigh);

void MasterI2C0Read(unsigned char address, unsigned char *pucData, unsigned long ulOffset,
          unsigned long ulCount);
void MasterI2C0Write(unsigned char address,unsigned char *pucData, unsigned long ulOffset,
           unsigned long ulCount);
