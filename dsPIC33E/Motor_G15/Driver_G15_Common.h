// Author			: Fabian Kung
// Date				: 4 June 2015
// Filename			: Driver_G15_Common.h

#ifndef _DRIVER_G15_COMMON_dsPIC33_H
#define _DRIVER_G15_COMMON_dsPIC33_H

#define     _G15_MAX_MOTOR          10          // Maximum no. of motors supported for each bank.
#define     _G15_MODE_MASK          0x01        // AND mask to extract LSb.
#define     _G15_READY_MASK         0x80        // AND mask to extract MSb.

typedef struct StructG15Motor   
{
	unsigned char    bytID;          //  The motor ID, 0-254.
	unsigned char    bytStatus;      //  The error status of the motor.
        unsigned char    bytMode;        //  LSb: 0 - Servo Mode, 1 = Wheel Mode.
                                //  MSb: 0 - Motor not ready (e.g. not initialized and set mode).
                                //       1 - Motor ready.
        unsigned int  unCWLimit;      //  CW rotation limit, 0-360 degrees.   This will be ignored in wheel mode.
        unsigned int  unCCWLimit;     //  CCW rotation limit, 0-360 degrees.  This will be ignored in wheel mode.
        unsigned int  unSetSpeed;     //  Speed setting, 1-1023.
	unsigned int  unPosition;     //  The present position of the motor output axle, 0-1087.
} G15_MOTOR_ATTRIBUTE;

#define     _G15_ON                 1
#define     _G15_OFF                0

#endif
