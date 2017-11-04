// Author			: Fabian Kung
// Date				: 9 August 2015
// Filename			: Driver_G15_V100.h

#ifndef _DRIVER_G15_dsPIC33_H
#define _DRIVER_G15_dsPIC33_H

// Include common header to all drivers and sources.  Here absolute path is used.
// To edit if one change folder
#include "../osmain.h"
#include "./Driver_G15_Common.h"
// 
//
// --- PUBLIC VARIABLES ---
//
extern  BYTE        gbytG15_Error;          // 0 = no motor error.
                                            // 1 = one of the connected motors is faulty.
extern  int         gnMotorCount;           // Indicate the number of motor connected to the controller.

extern  int         gnG15_Function;         // To set the function of the driver.
#define     _G15_IDLE               -1
#define     _G15_INIT               0
#define     _G15_SET_LED            1
#define     _G15_READ_STATUS        2
#define     _G15_SET_WHEEL_MODE     3
#define     _G15_SET_SERVO_MODE     4
#define     _G15_WHEEL_MOVE         5
#define     _G15_SERVO_MOVE         6
#define     _G15_SET_TORQUE         7

extern  G15_MOTOR_ATTRIBUTE gobjG15Motor[_G15_MAX_MOTOR - 1];

//
// --- PUBLIC FUNCTION PROTOTYPE ---
//
void Proce_Motor_G15_Driver(TASK_ATTRIBUTE *);
G15_MOTOR_ATTRIBUTE *InitG15Motor(unsigned char);
void ClearAllG15MotorMode(void);
int SetG15MotorAngleDegree(unsigned int, unsigned int, unsigned char);
int SetG15ServoMode(unsigned int, unsigned int, unsigned int, unsigned char);
int SetG15MotorLED(unsigned int, unsigned char);
int SetG15MotorTorque(unsigned int, unsigned char);
#endif
