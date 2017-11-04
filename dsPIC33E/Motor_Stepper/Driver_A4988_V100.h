// Author			: Fabian Kung
// Date				: 19 Sep 2017
// Filename			: Driver_A4988_V100.h

#ifndef _DRIVER_A4988_dsPIC33_H
#define _DRIVER_A4988_dsPIC33_H

// Include common header to all drivers and sources.  Here absolute path is used.
// To edit if one change folder
#include "../osmain.h"

//
// --- PUBLIC VARIABLES ---
//
typedef struct StructProceA4988Driver
{
    UINT16  unEn4988;           // Set greater than 0 to enable A4988 module.
                                // Note: User process can reset the driver by setting this
                                // global variable to 0.  Reseting the driver also initialize
                                // all the velocity and distance variables.      
    INT16   nSpeed1;            // Speed settings, from 0 (stop the motor) to
    INT16   nSpeed2;            // 400 (maximum speed).  Positive rotate clockwise,
                                // negative rotate anti-clockwise.
} DSPIC33E_A4988_DRIVER;

extern  DSPIC33E_A4988_DRIVER    gobjDriverA4988;
extern  long        gnDistanceMoveLW;   // Distance traveled by left wheel in no. of steps, 32 bits integer.
extern  long        gnDistanceMoveRW;   // Distance traveled by right wheel in no. of steps, 32 bits integer.
extern  long        gnDistanceMoveW;    // Average distance traveled by both wheels in no. of steps, 32 bits integer.
extern  int         gnHeading;          // This indicates the direction, essentially the difference between right and left 
                                        // wheel distance.  Facing right if > 0, and facing left if < 0.	
extern  unsigned int    gunDeadBandThres;  // Deadband threshold, speed setting less than this magnitude will be ignored.

//
// --- PUBLIC FUNCTION PROTOTYPE ---
//
void Proce_A4988_Driver(TASK_ATTRIBUTE *);		// A4988 stepper motor driver.
													
#endif