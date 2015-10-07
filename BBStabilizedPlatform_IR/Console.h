/*
  Copyright Bluebird Aero Systems Engineering 2011

  $Id: Console.h,v 1.3 2011/03/30 06:58:34 EranS Exp $
  $Log: Console.h,v $
  Revision 1.3  2011/03/30 06:58:34  EranS
  added more telemetry
  simplified and improved motor driver

  Revision 1.2  2011/03/21 13:42:03  EranS
  fix adc code, add telemtry

  Revision 1.1  2011/03/15 12:23:45  EranS
  First save


*/

void ConsoleInit(unsigned long baud);
void UART0Send(char *pucBuffer, unsigned ulCount);
