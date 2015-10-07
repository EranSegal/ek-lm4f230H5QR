/*
  Copyright Bluebird Aero Systems Engineering 2011

  $Id: GPIOhandler.h,v 1.1 2011/04/17 08:44:11 EranS Exp $
  $Log: GPIOhandler.h,v $
  Revision 1.1  2011/04/17 08:44:11  EranS
  add proper reset of encoders via GPIO


 */
void GPIOPinInit(unsigned char port,unsigned char pin,tBoolean input,tBoolean hasinterrupt);
