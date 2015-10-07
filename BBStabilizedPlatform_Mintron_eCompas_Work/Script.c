/*
  Copyright Bluebird Aero Systems Engineering 2012

  General Purpose Scripting Engine

  $Id: Script.c,v 1.4 2011/07/21 10:08:21 EranS Exp $
  $Log: Script.c,v $
  Revision 1.4  2011/07/21 10:08:21  EranS
  changed script from time based to cycle based

  Revision 1.3  2011/07/21 06:31:25  EranS
  Improved script engine to allow zero elapsed time scripts

  Revision 1.2  2011/07/20 06:49:47  EranS
  changed ITG rate to 100
  encoder init ar higher PWM
  telemetry changed

  Revision 1.1  2011/07/14 07:19:48  EranS
  added script engine

 
*/

#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "bluebird.h"
#include "inc/hw_ints.h"
#include "inc/hw_types.h"
#include "SysTick.h"
#include "A3906.h"
#include "ITG3200.h"

tBoolean scriptactive;
int scriptindex;
unsigned long interationcount;

// set pitch motor, oscillate rate

extern int oscillaterate;
extern tBoolean testpitch;
extern tBoolean stabilizeaxis[];  // separate stabilise switch for each axis
extern unsigned localdcycle[];
extern tBoolean oscillateswitch;
extern enum MOTOR oscillatemotor;
extern int oscillatecount;
extern tBoolean traceflag;
extern float omega_vertical,omega_horizontal;  

void incrementdcycle(int value,int spare)
{
  localdcycle[testpitch ? PITCH_MOTOR : ROLL_MOTOR] += value;
  // make sure not higher than max allowed
  if (localdcycle[testpitch ? PITCH_MOTOR : ROLL_MOTOR] > 90)
    localdcycle[testpitch ? PITCH_MOTOR : ROLL_MOTOR] = 90;
}

void incrementoscillaterate(int value,int spare)
{
  oscillaterate += value;
  if (oscillaterate > 30)
    oscillaterate = 30;
}

void doOscillate(int value,int spare)
{
  oscillateswitch = true;
  oscillatecount = value;
  oscillatemotor = testpitch ? PITCH_MOTOR : ROLL_MOTOR;
  stabilizeaxis[testpitch ? PITCH_MOTOR : ROLL_MOTOR] = false;
  localdcycle[oscillatemotor] = 70;
  traceflag = true;
}

void ScriptStop(int value,int spare)
{
  scriptactive = false;
}

void bringToPosition(int v,int h)
{
  omega_vertical = v;
  omega_horizontal = h;
  stabilizeaxis[PITCH_MOTOR] = true;
  stabilizeaxis[ROLL_MOTOR] = true;
}

// this is a trick to make a loop, set marker and loop count
int loopMarker[5],loopCount[5];  // maintain up to 5 loop counter
void loopBegin(int index,int p2)
{
  loopMarker[index] = scriptindex;
}
void loopSetCount(int index,int value)
{
  loopCount[index] = value;
}
void loopEnd(int index,int p2)
{
  if (--loopCount[index] > 0)
    scriptindex = loopMarker[index];
}

/*
  call this before using sinetopwm
*/
float sinangle; // save as radians
void sineinit(int p1,int p2)
{
  sinangle = (float)p1;
}
/*
    Go through 0->90->0->-90->0 in ITG3200_SAMPLE_RATE/oscillaterate cycles
*/
void sinetopwm(int p1, int p2)
{
  float sina;
  sinangle += PI/(float)(ITG3200_SAMPLE_RATE/oscillaterate);
  sina = sinf(sinangle)/1.5;
  GeneralPitchRoll(testpitch ? PITCH_MOTOR : ROLL_MOTOR,
                 sina <= 0.0F ? A3906_FORWARD : A3906_REVERSE,
                 abs((short)(sina*50.0F)));
}

void donothing(int p1, int p2)
{
  GeneralPitchRoll(testpitch ? PITCH_MOTOR : ROLL_MOTOR,A3906_BRAKE, 50);
}

enum ConvertAction { NO_ACTION, DIV_BY_OSCRATE, SET_HZ_RATE };
struct script {
  unsigned long iterations;  // how many times to do this 
  void (*action)(int,int); // what to do
  enum ConvertAction factor;
  int parameter1;  // parameter for action
  int parameter2;  // parameter for action
};

#define NUM_CYCLES 4
#define NUM_INCREMENTS 10 // of oscillaterate
// all scripts take 2 in parameters, even if not actually used
struct script scripts [][2] = {
  {0,sineinit,NO_ACTION,0,0},
  {0,loopSetCount,NO_ACTION,2,NUM_INCREMENTS}, // outer loop  oscrate 2-> 20 in 10 steps
  {0,loopBegin,NO_ACTION,2,0},     // start outer loop
  {0,loopSetCount,NO_ACTION,1,NUM_CYCLES},  // middle loop
  {0,loopBegin,NO_ACTION,1,0},     // start middle loop
  {0,loopSetCount,SET_HZ_RATE,0,ITG3200_SAMPLE_RATE}, // inner loop
  {0,loopBegin,NO_ACTION,0,0},    // start inner loop
  {1,sinetopwm,NO_ACTION,0,0},    // do this every cycle
  {0,loopEnd,NO_ACTION,0,0},      // end inner loop
  {0,loopEnd,NO_ACTION,1,0},      // end middle loop
  {ITG3200_SAMPLE_RATE/2,donothing,NO_ACTION,  0,0},
  {0,incrementoscillaterate,NO_ACTION,2,0},
  {0,sineinit,NO_ACTION,0,0},
  {0,loopEnd,NO_ACTION,2,0},      // end outer loop
  {0,ScriptStop,NO_ACTION,0,0},   // not really necessary but do it anyway
};


/*
  Scripts is a 2-dimensional array. Row 0 is the original version, Row 1
  is an updated version made by copying and modifying the original as 
  necessary.
  For example, where factor is SET_HZ_RATE is set, the original elapsed time is
  based upon a cycle of 1Hz, whereas I want to slice it up and repeat based upon
  oscillaterate.
  Execution is based on the modified version
*/
void ScriptStart()
{
  int i;
  //historyindex = 0;
  for (i=0; i< (sizeof(scripts)/sizeof(struct script))/2; i++)
  {
    scripts[i][1].action = scripts[i][0].action;
    scripts[i][1].parameter1 = scripts[i][0].parameter1;
    scripts[i][1].parameter2 = scripts[i][0].parameter2;
    switch (scripts[i][0].factor)
    {
      case DIV_BY_OSCRATE:
        scripts[i][1].iterations = (int)((oscillatecount * 1000.0)/oscillaterate) + 500;
        break;
      case SET_HZ_RATE:
        scripts[i][1].parameter2 = scripts[i][0].parameter2/oscillaterate;
        break;
      case NO_ACTION:
        scripts[i][1].iterations = scripts[i][0].iterations;
        break;
    }
  }
  // start the first script
  scriptactive = true;
  scriptindex = 0;
  interationcount = 0;
  (scripts[0][1].action)(scripts[0][1].parameter1,scripts[0][1].parameter2);
}

// called from main loop after every gyro interrupt

void ScriptExecute()
{
  tBoolean donext = true;
  // if elapsed time is zero, do it right away with no time penalty
  while (donext && scriptactive)
  {
    // if time to do next script, do it and increment index
    if (interationcount >= scripts[scriptindex][1].iterations)
    {
      scriptindex++;
      // may need need to update elapased time
      switch (scripts[scriptindex][0].factor)
      {
        case SET_HZ_RATE:
          scripts[scriptindex][1].parameter2 = scripts[scriptindex][0].parameter2/oscillaterate;
          break;
      }
      (scripts[scriptindex][1].action)(scripts[scriptindex][1].parameter1,scripts[scriptindex][1].parameter2);
      interationcount = 0;
      // if a script has zero elapsed time let the next script run right away
      donext = (scripts[scriptindex][1].iterations == 0);
      // turn off when we get to the end
      if ((1+scriptindex) == (sizeof(scripts)/sizeof(struct script))/2)
        scriptactive = false;
    }
    else
      donext = false;  // force exit
    interationcount++;
  }
}

/*
  {1,rmotor,NO_ACTION},        // select pitch
  {1,resetsetdcycle,NO_ACTION}, // dcycle 10
  {1,incrementdcycle,NO_ACTION}, // dcycle 20
  {1,incrementdcycle,NO_ACTION}, // dcycle 30
  {1,incrementdcycle,NO_ACTION}, // dcycle 40
//  {3000,bringToPosition,NO_ACTION},  // allow 3 seconds
  {10000,doOscillate,DIV_BY_OSCRATE,10}, // takes 10 secs at rate 1
//  {3000,bringToPosition,NO_ACTION}, 
  {1,incrementdcycle,NO_ACTION}, // dcycle 50
  {10000,doOscillate,DIV_BY_OSCRATE}, 
//  {3000,bringToPosition,NO_ACTION}, 
  {1,incrementdcycle,NO_ACTION}, // dcycle 60
  {10000,doOscillate,DIV_BY_OSCRATE},
//  {3000,bringToPosition,NO_ACTION}, 
  {1,incrementdcycle,NO_ACTION}, // dcycle 70
  {10000,doOscillate,DIV_BY_OSCRATE}, 
//  {3000,bringToPosition,NO_ACTION}, 
  {1,incrementdcycle,NO_ACTION}, // dcycle 80
  {10000,doOscillate,DIV_BY_OSCRATE}, 
//  {3000,bringToPosition,NO_ACTION}, 

=====================

  {10000,doOscillate,DIV_BY_OSCRATE}, // takes 10 secs at rate 1
  {1,decrementoscillaterate,NO_ACTION},
  {10000,doOscillate,DIV_BY_OSCRATE}, // takes 10 secs at rate 1
  {1,decrementoscillaterate,NO_ACTION},
  {10000,doOscillate,DIV_BY_OSCRATE}, // takes 10 secs at rate 1
  {1,decrementoscillaterate,NO_ACTION},
  {10000,doOscillate,DIV_BY_OSCRATE}, // takes 10 secs at rate 1
  {1,decrementoscillaterate,NO_ACTION},
  {10000,doOscillate,DIV_BY_OSCRATE}, // takes 10 secs at rate 1
  {1,decrementoscillaterate,NO_ACTION},
  {10000,doOscillate,DIV_BY_OSCRATE}, // takes 10 secs at rate 1
  {1,decrementoscillaterate,NO_ACTION},
  {10000,doOscillate,DIV_BY_OSCRATE}, // takes 10 secs at rate 1
  {1,decrementoscillaterate,NO_ACTION},
  {10000,doOscillate,DIV_BY_OSCRATE}, // takes 10 secs at rate 1
  {1,decrementoscillaterate,NO_ACTION},
  {10000,doOscillate,DIV_BY_OSCRATE}, // takes 10 secs at rate 1
  {1,decrementoscillaterate,NO_ACTION},
*/
