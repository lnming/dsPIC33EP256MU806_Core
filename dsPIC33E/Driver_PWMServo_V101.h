// Author			: Fabian Kung
// Date				: 28 Oct 2016
// Filename			: Driver_PWMServo_V101.h

#ifndef _DRIVER_PWMSERVO_dsPIC33E_H
#define _DRIVER_PWMSERVO_dsPIC33E_H

// Include common header to all drivers and sources.  Here absolute path is used.
// To edit if one change folder
#include "osmain.h"

//
// --- PUBLIC VARIABLES ---
//
// RC servo motor driver control variables
typedef struct StructProcePWMDriver
{
    UINT16   NumChannel;            // No. of outputs, valid values are 4, 6 and 8.  Any
                                    // other values not of these will be assume as 8 channel
                                    // of outputs.
    UINT16   unEnPWM;               // Set greater than 0 to enable the PWM module.
} DSPIC33E_PWM_DRIVER;

extern DSPIC33E_PWM_DRIVER    gobjDriverPWM;

extern  UINT16 gunRCServoPW_us[8];                             // Public date. RC servo motor pulse width in usec.
                                                                // For version 1.00 only 8 variables are used, the last two are ignored.
extern  UINT16 gunPWStep_us[8];                                // Public data. RC servo motor pulse width increment/decrement
                                                                // step in usec.  Again for version 1.00 only 8 variables are used,
                                                                // the last two are ignored.
extern  UINT16 gnCurrentMotorIDA;                               // Public data.  RC servo motor ID.  Default = 0.

//
// --- PUBLIC FUNCTION PROTOTYPE ---
//
void Proce_PWMServo_Driver(TASK_ATTRIBUTE *);                   // PWM servo motor driver.

#endif