// Author			: Fabian Kung
// Date				: 14 June 2015
// Filename			: Driver_G15_V100.h

#ifndef _DRIVER_G15_2_dsPIC33_H
#define _DRIVER_G15_2_dsPIC33_H

// Include common header to all drivers and sources.  Here absolute path is used.
// To edit if one change folder
#include "../osmain.h"
#include "./Driver_G15_Common.h"
// 
//
// --- PUBLIC VARIABLES ---
//
extern  BYTE        gbytG15_Error2;          // 0 = no motor error.
                                            // 1 = one of the connected motors is faulty.
extern  int         gnMotorCount2;           // Indicate the number of motor connected to the controller.

extern  int         gnG15_Function2;         // To set the function of the driver.
#define     _G15_IDLE               -1
#define     _G15_INIT               0
#define     _G15_SET_LED            1
#define     _G15_READ_STATUS        2
#define     _G15_SET_WHEEL_MODE     3
#define     _G15_SET_SERVO_MODE     4
#define     _G15_WHEEL_MOVE         5
#define     _G15_SERVO_MOVE         6


extern  G15_MOTOR_ATTRIBUTE gobjG15Motor2[_G15_MAX_MOTOR - 1];


//
// --- PUBLIC FUNCTION PROTOTYPE ---
//
void Proce_Motor_G15_Driver2(TASK_ATTRIBUTE *);
G15_MOTOR_ATTRIBUTE *InitG15Motor2(unsigned char);
void ClearAllG15MotorMode2(void);
int SetG15MotorAngleDegree2(unsigned int, unsigned int, unsigned char);
int SetG15ServoMode2(unsigned int, unsigned int, unsigned int, unsigned char);
int SetG15MotorLED2(unsigned int, unsigned char);
#endif
