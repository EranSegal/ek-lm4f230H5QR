//*****************************************************************************
//
// hbridge.c - Driver for the H-bridge.
//
// Copyright (c) 2008-2009 Luminary Micro, Inc.  All rights reserved.
// Software License Agreement
// 
// Luminary Micro, Inc. (LMI) is supplying this software for use solely and
// exclusively on LMI's microcontroller products.
// 
// The software is owned by LMI and/or its suppliers, and is protected under
// applicable copyright laws.  All rights are reserved.  You may not combine
// this software with "viral" open-source software in order to form a larger
// program.  Any use in violation of the foregoing restrictions may subject
// the user to criminal sanctions under applicable laws, as well as to civil
// liability for the breach of the terms and conditions of this license.
// 
// THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED
// OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
// MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.
// LMI SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR
// CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
// 
// This is part of revision 4652 of the RDK-BDC Firmware Package.
//
//*****************************************************************************

#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_pwm.h"
#include "inc/hw_types.h"
#include "driverlib/gpio.h"
#include "driverlib/pwm.h"
#include "driverlib/rom.h"
//#include "constants.h"
//#include "hbridge.h"
//#include "limit.h"
#include "pins.h"

//*****************************************************************************
//
// The PWMxGENy value that results in the output signal being a PWM pulse
// stream.  This is used for the CTRL input to the gate drivers.
//
//*****************************************************************************
#define PULSE                   (PWM_X_GENA_ACTLOAD_ONE |                     \
                                 PWM_X_GENA_ACTCMPAD_ZERO)

//*****************************************************************************
//
// The PWMxGENy value that results in the output signal being off all the time.
// This is used for the CTRL input to the gate drivers.
//
//*****************************************************************************
#define OFF                     (PWM_X_GENA_ACTZERO_ZERO |                    \
                                 PWM_X_GENA_ACTLOAD_ZERO |                    \
                                 PWM_X_GENA_ACTCMPAU_ZERO |                   \
                                 PWM_X_GENA_ACTCMPAD_ZERO |                   \
                                 PWM_X_GENA_ACTCMPBU_ZERO |                   \
                                 PWM_X_GENA_ACTCMPBD_ZERO)

//*****************************************************************************
//
// The PWMxGENy value that results in the output signal being on all the time.
// This is used for the CTRL input to the gate drivers.
//
//*****************************************************************************
#define ON                      (PWM_X_GENA_ACTZERO_ONE |                     \
                                 PWM_X_GENA_ACTLOAD_ONE |                     \
                                 PWM_X_GENA_ACTCMPAU_ONE |                    \
                                 PWM_X_GENA_ACTCMPAD_ONE |                    \
                                 PWM_X_GENA_ACTCMPBU_ONE |                    \
                                 PWM_X_GENA_ACTCMPBD_ONE)

//*****************************************************************************
//
// The PWMxGENy value that results in the output signal being low all the time.
// This is used for the PWM input to the gate drivers.
//
//*****************************************************************************
#define LO                      (PWM_X_GENB_ACTZERO_ZERO |                    \
                                 PWM_X_GENB_ACTLOAD_ZERO |                    \
                                 PWM_X_GENB_ACTCMPAU_ZERO |                   \
                                 PWM_X_GENB_ACTCMPAD_ZERO |                   \
                                 PWM_X_GENB_ACTCMPBU_ZERO |                   \
                                 PWM_X_GENB_ACTCMPBD_ZERO)

//*****************************************************************************
//
// The PWMxGENy value that results in the output signal being high all the
// time.  This is used for the PWM input to the gate drivers.
//
//*****************************************************************************
#define HI                      (PWM_X_GENB_ACTZERO_ONE |                     \
                                 PWM_X_GENB_ACTLOAD_ONE |                     \
                                 PWM_X_GENB_ACTCMPAU_ONE |                    \
                                 PWM_X_GENB_ACTCMPAD_ONE |                    \
                                 PWM_X_GENB_ACTCMPBU_ONE |                    \
                                 PWM_X_GENB_ACTCMPBD_ONE)

//*****************************************************************************
//
// This function converts an ADC reading value into current in Amperes, as an
// 8.8 fixed-point value.
//
//*****************************************************************************
#define ADC_TO_CURRENT(ulA)     ((unsigned long)((((ulA) * 4972) + 42018) /   \
                                                 256))

//*****************************************************************************
//
// The width of the plateau at the full reverse end of the voltage curve.  Any
// voltage command within this plateau will result in the motor operating in
// full reverse.
//
//*****************************************************************************
#define REVERSE_PLATEAU         1024

//*****************************************************************************
//
// The width of the plateau around neutral in the voltage curve.  Any voltage
// command within this plateau will result in the motor staying in neutral.
//
//*****************************************************************************
#define NEUTRAL_PLATEAU         2048

//*****************************************************************************
//
// The width of the plateau at the full forward end of the voltage curve.  Any
// voltage command within this plateau will result in the motor operating in
// full forward.
//
//*****************************************************************************
#define FORWARD_PLATEAU         1024

//*****************************************************************************
//
// The current output voltage to the H-bridge.  The volatile is not strictly
// required but is used in EW-ARM to workaround some versions of the compiler
// trying to be too clever (resulting in incorrectly generated machine code).
//
//*****************************************************************************
#ifdef ewarm
static volatile long g_lHBridgeV = 0;
#else
static long g_lHBridgeV = 0;
#endif

//*****************************************************************************
//
// The soft limit switch flags.  The first enables the soft limit switches, the
// second indicates that the forward soft limit switch is a less than
// comparison instead of a greater than comparison, and the third indicates the
// same for the reverse soft limit switch.
//
//*****************************************************************************
#define LIMIT_FLAG_POSITION_EN  2
#define LIMIT_FLAG_FORWARD_LT   3
#define LIMIT_FLAG_REVERSE_LT   4
unsigned long g_ulLimitFlags;

//*****************************************************************************
//
// The position values for the forward and reverse soft limit switches.
//
//*****************************************************************************
static long g_lLimitForward;
static long g_lLimitReverse;

//*****************************************************************************
//
// The bit positions of the flags in g_ulLimitFlags.
//
//*****************************************************************************
#define LIMIT_FLAG_FORWARD_OK   0
#define LIMIT_FLAG_REVERSE_OK   1

//*****************************************************************************
//
// This function determines if the state of the forward limit switch allows the
// motor to operate in the forward direction.
//
//*****************************************************************************
#define LimitForwardOK()                                                      \
        (HWREGBITW(&g_ulLimitFlags, LIMIT_FLAG_FORWARD_OK) == 1)

//*****************************************************************************
//
// This function determines if the state of the reverse limit switch allows the
// motor to operate in the reverse direction.
//
//*****************************************************************************
#define LimitReverseOK()                                                      \
        (HWREGBITW(&g_ulLimitFlags, LIMIT_FLAG_REVERSE_OK) == 1)

//*****************************************************************************
//
// The averaged winding current.
//
//*****************************************************************************
static unsigned short g_usCurrent = 0;

//*****************************************************************************
//
// The averaged winding current when the motor is not being driven.  This is
// used to cancel out any zero-current error that may be present (due to
// component inaccuracies).
//
//*****************************************************************************
static unsigned short g_usCurrentZero;

//*****************************************************************************
//
// The maximum output voltage to the H-bridge.
//
//*****************************************************************************
static long g_lHBridgeVMax = 32767;

//*****************************************************************************
//
// The target current for current control mode.
//
//*****************************************************************************
static long g_lCurrentTarget = 0;

//*****************************************************************************
//
// The current drive voltage.  This is determined by the current control mode
// (voltage, current, or speed).
//
//*****************************************************************************

static long g_lVoltage = 0;

//*****************************************************************************
//
// The settings for the brake/coast configuration.
//
//*****************************************************************************
#define HBRIDGE_JUMPER          0
#define HBRIDGE_BRAKE           1
#define HBRIDGE_COAST           2

//*****************************************************************************
//
// The configuration of the brake/coast setting.  By default, the state of the
// jumper is used to determine brake/coast.
//
//*****************************************************************************
static unsigned long g_ulHBridgeBrakeCoast = HBRIDGE_JUMPER;

//*****************************************************************************
//
// Sets the H-bridge into either brake or coast mode, based on the current
// configuration.  Note that coast is also referred to as fast decay and brake
// is also referred to as slow decay.
//
//*****************************************************************************
static void
HBridgeBrakeCoast(void)
{
    //
    // See if the drive is configured for braking or coasting.
    //
    if((g_ulHBridgeBrakeCoast == HBRIDGE_COAST) ||
       ((g_ulHBridgeBrakeCoast == HBRIDGE_JUMPER) &&
        (GPIOPinRead(BRAKECOAST_PORT, BRAKECOAST_PIN) ==
         BRAKECOAST_COAST)))
    {
        //
        // Place the H-bridge into coast mode.
        //
        HWREG(PWM_BASE + M_MINUS_CTRL_GEN) = LO;
        HWREG(PWM_BASE + M_MINUS_PWM_GEN) = OFF;
        HWREG(PWM_BASE + M_PLUS_CTRL_GEN) = LO;
        HWREG(PWM_BASE + M_PLUS_PWM_GEN) = OFF;
    }
    else
    {
        //
        // Place the H-bridge into brake mode.
        //
        HWREG(PWM_BASE + M_MINUS_CTRL_GEN) = LO;
        HWREG(PWM_BASE + M_MINUS_PWM_GEN) = ON;
        HWREG(PWM_BASE + M_PLUS_CTRL_GEN) = LO;
        HWREG(PWM_BASE + M_PLUS_PWM_GEN) = ON;
    }
}


//*****************************************************************************
//
// This function returns the motor winding current, specified as an unsigned
// 16.16 fixed-point value that represents the current in Amperes.
//
//*****************************************************************************
unsigned long
ADCCurrentGet(void)
{
    unsigned long ulRet;

    //
    // If the ADC reading is less than the zero value, then the current reading
    // is zero.
    //
    if(g_usCurrent < g_usCurrentZero)
    {
        ulRet = 0;
    }
    else
    {
        //
        // Convert the ADC reading into Amperes.
        //
        ulRet = ADC_TO_CURRENT(g_usCurrent - g_usCurrentZero);

        //
        // If the current is less than 1 Amp, or is negative, then set the
        // current reading to zero.
        //
        if((ulRet < 256) || (ulRet & 0x80000000))
        {
            ulRet = 0;
        }
    }

    //
    // Return the computed winding current.
    //
    return(ulRet);
}

//*****************************************************************************
//
// This function handles the periodic processing for current control mode.
//
//*****************************************************************************
static void
ControllerCurrentMode(void)
{
    long lTemp;

    //
    // If the target current is zero, then the output voltage is zero also.
    //
    if(g_lCurrentTarget == 0)
    {
        g_lVoltage = 0;
    }
    else
    {
        //
        // Get the winding current.
        //
        lTemp = ADCCurrentGet();
        if(g_lVoltage < 0)
        {
            lTemp = 0 - lTemp;
        }

        //
        // Compute the error between the target current and the winding
        // current.
        //
        lTemp = g_lCurrentTarget - lTemp;

        //
        // Run the PID controller, with the output being the output voltage
        // that should be driven to the motor.
        //
        //lTemp = PIDUpdate(&g_sCurrentPID, lTemp * 256) / 256;

        //
        // Limit the output voltage to the valid values.
        //
        if(lTemp < -32768)
        {
            lTemp = -32768;
        }
        if(lTemp > 32767)
        {
            lTemp = 32767;
        }

        //
        // Do not drive the motor if the desired output voltage is the opposite
        // polarity of the target current (in other words, do not drive the
        // motor forward if the target current is reverse), or if the desired
        // output voltage is within the neutral plateau.
        //
        if(((g_lCurrentTarget < 0) && (lTemp > 0)) ||
           ((g_lCurrentTarget > 0) && (lTemp < 0)) ||
           ((lTemp >= (0 - NEUTRAL_PLATEAU)) && (lTemp <= NEUTRAL_PLATEAU)))
        {
            lTemp = 0;
        }

        //
        // Set the new output voltage.
        //
        g_lVoltage = lTemp;
    }

    //
    // Send the new output voltage to the H-bridge.
    //
    HBridgeVoltageSet(g_lVoltage);
}



//*****************************************************************************
//
// Sets the H-bridge output voltage.
//
//*****************************************************************************
void
HBridgeVoltageSet(long lVoltage)
{
    //
    // Save the new output voltage.  This will be applied to the H-bridge on
    // the next H-bridge tick.
    //
    g_lHBridgeV = lVoltage;
}


//*****************************************************************************
//
// Sets the H-bridge brake/coast configuration.
//
//*****************************************************************************
void
HBridgeBrakeCoastSet(unsigned long ulState)
{
    //
    // Save the new brake/coast configuration.  This will be applied on the
    // next H-bridge tick in which the motor is in neutral.
    //
    g_ulHBridgeBrakeCoast = ulState;
}

//*****************************************************************************
//
// Handles the periodic update to the H-bridge output.
//
//*****************************************************************************
void
HBridgeTick(void)
{
    unsigned long ulCompare;

    //
    // See if the output voltage should be zero (in other words, neutral).
    //
    if(g_lHBridgeV == 0)
    {
        //
        // Set the H-bridge to brake or coast mode.  This is done every
        // interrupt to handle the cases where the jumper is being used and it
        // is being driven by an external source (such as another
        // microcontroller).
        //
        HBridgeBrakeCoast();
    }

    //
    // See if the output voltage is negative (in other words, reverse).
    //
    else if(g_lHBridgeV < 0)
    {
        //
        // Make sure that the limit switches allow the motor to run in reverse.
        //
        if(LimitReverseOK())
        {
            //
            // Compute the PWM compare value from the desired output voltage.
            // Limit the compare value such that the output pulse doesn't
            // become too short or too long.
            //
            ulCompare = (((((g_lHBridgeV * g_lHBridgeVMax) / 32767) + 32768) *
                          SYSCLK_PER_PWM_PERIOD) / 32768);
            if(ulCompare < 24)
            {
                ulCompare = 24;
            }
            if(ulCompare > (SYSCLK_PER_PWM_PERIOD - 24))
            {
                ulCompare = SYSCLK_PER_PWM_PERIOD - 24;
            }

            //
            // Update the compare registers with the encoded pulse width.
            //
            HWREG(PWM_BASE + M_MINUS_CMP) = ulCompare;
            HWREG(PWM_BASE + M_PLUS_CMP) = ulCompare;
            HWREG(PWM_BASE + ADC_CMP) =
                (SYSCLK_PER_PWM_PERIOD + ulCompare) / 2;

            //
            // Update the generator registers with the required drive pattern.
            //
            HWREG(PWM_BASE + M_MINUS_CTRL_GEN) = HI;
            HWREG(PWM_BASE + M_MINUS_PWM_GEN) = PULSE;
            HWREG(PWM_BASE + M_PLUS_CTRL_GEN) = LO;
            HWREG(PWM_BASE + M_PLUS_PWM_GEN) = ON;
        }
        else
        {
            //
            // The limit switches do not allow the motor to be driven in
            // reverse, so set the H-bridge to brake or coast mode.
            //
            HBridgeBrakeCoast();
        }
    }

    //
    // Otherwise, the output voltage is positive (in other words, forward).
    //
    else
    {
        //
        // Make sure that the limit switches allow the motor to run forward.
        //
        if(LimitForwardOK())
        {
            //
            // Compute the PWM compare value from the desired output voltage.
            // Limit the compare value such that the output pulse doesn't
            // become too short or too long.
            //
            ulCompare = (((32767 - ((g_lHBridgeV * g_lHBridgeVMax) / 32767)) *
                          SYSCLK_PER_PWM_PERIOD) / 32767);
            if(ulCompare < 24)
            {
                ulCompare = 24;
            }
            if(ulCompare > (SYSCLK_PER_PWM_PERIOD - 24))
            {
                ulCompare = SYSCLK_PER_PWM_PERIOD - 24;
            }

            //
            // Update the compare registers with the encoded pulse width.
            //
            HWREG(PWM_BASE + M_MINUS_CMP) = ulCompare;
            HWREG(PWM_BASE + M_PLUS_CMP) = ulCompare;
            HWREG(PWM_BASE + ADC_CMP) =
                (SYSCLK_PER_PWM_PERIOD + ulCompare) / 2;

            //
            // Update the generator registers with the required drive pattern.
            //
            HWREG(PWM_BASE + M_MINUS_CTRL_GEN) = LO;
            HWREG(PWM_BASE + M_MINUS_PWM_GEN) = ON;
            HWREG(PWM_BASE + M_PLUS_CTRL_GEN) = HI;
            HWREG(PWM_BASE + M_PLUS_PWM_GEN) = PULSE;
        }
        else
        {
            //
            // The limit switches do not allow the motor to be driven forward,
            // so set the H-bridge to brake or coast mode.
            //
            HBridgeBrakeCoast();
        }
    }

    //
    // Force a global sync so that any pending updates to CMPA, GENA, or GENB
    // are handled.
    //
    PWMSyncUpdate(PWM_BASE, GEN_M_MINUS_BIT | GEN_M_PLUS_BIT);
}

