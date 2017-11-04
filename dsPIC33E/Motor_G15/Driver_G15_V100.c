//////////////////////////////////////////////////////////////////////////////////////////////
//
//	USER DRIVER ROUTINES DECLARATION (PROCESSOR DEPENDENT)
//
//  (c) Copyright 2015, Fabian Kung Wai Lee, Selangor, MALAYSIA
//  All Rights Reserved
//
//////////////////////////////////////////////////////////////////////////////////////////////
//
// File			: Drivers_G15_V100.c
// Author(s)		: Fabian Kung
// Last modified	: 11 August 2015
// Toolsuites		: Microchip MPLAB X IDE v2.20 or above
//                	  MPLAB XC16 C-Compiler v1.22 or above

// --- Engineering change notice ---
// 14 June 2015 - Basic working version.
// 7 July 2015 - Reduced the updating duration during angle and speed settings
//               in servo mode.
// 9 August 2015 - Add in mode to control the motor output torque.

// Include common header to all drivers and sources.  Here absolute path is used.
// To edit if one change folder.
#include "../osmain.h"
#include "../Driver_UART2_V100.h"
//#include "./Driver_G15_Common.h"
#include "Driver_G15_V100.h"
#include "C:\FabianKung\Projects\Embedded_Systems\C_Library\dsPIC33E\Driver_Audio_V101.h"

// --- Public Variables ---
BYTE        gbytG15_Error;          // 0 = no motor error.
                                    // 1 = one of the connected motors is faulty.
int         gnG15_Function;         // To set the function of the driver.
#define     _G15_IDLE               -1
#define     _G15_INIT               0
#define     _G15_SET_LED            1
#define     _G15_READ_STATUS        2
#define     _G15_SET_WHEEL_MODE     3
#define     _G15_SET_SERVO_MODE     4
#define     _G15_WHEEL_MOVE         5
#define     _G15_SERVO_MOVE         6
#define     _G15_SET_TORQUE         7

int         gnMotorCount;               // Indicate the number of motor connected to the controller.
G15_MOTOR_ATTRIBUTE gobjG15Motor[_G15_MAX_MOTOR - 1];   // Object to store motor attributes

BYTE        gbytG15_Status;             // Error status flags for G15 motor.
UINT16      gunG15_Param1;              // Parameter 1.
UINT16      gunG15_Param2;              // Parameter 2.
UINT16      gunG15_Param3;              // Parameter 3.


// --- File Level Variables ---
BYTE        bytG15_ID;                  // ID for G15 motor for general usage.
BYTE        bytCurrentID;               // Temporary variable to store current motor ID.
UINT16      unPresentPosition;          // Temporary variable to store current motor angle/position.
int         nCurrentMotorPtr;           // Index to current motor data structure.


#define     _G15_ON                     1
#define     _G15_OFF                    0

#define     _G15_TX_MODE                1
#define     _G15_RX_MODE                0

#define     _BROADCAST_ID               0xFE            // 254

#define     _HEADER                    	0xFF            // 255

#define     _STATUS_PACKET_TIMEOUT	1000		// in no. of system ticks.

///
/// Process name	: Proce_Motor_G15_Driver - Driver for Cytron Technologies G15 smart servo.
///
/// Author		: Fabian Kung
///
/// Last modified	: 11 August 2015
///
/// Code version	: 0.96
///
/// Processor		: dsPIC33EP256MU80X family.
///
/// Processor/System Resource
/// PIN			:  PIN_TX_RX_SEL (RB4)
///
/// MODULE		:  UART2 (Internal)
///
/// DRIVER		:  Proce_UART2_Driver
///
/// RTOS		:  Ver 1 or above, round-robin scheduling.
///
/// Global variable	: gbytTXbuffer2, gbytRXbuffer2 and others.
///

#ifdef 				  __OS_VER		// Check RTOS version compatibility.
	#if 			  __OS_VER < 1
		#error "Proce__Motor_G15_Driver: Incompatible OS version"
	#endif
#else
	#error "Proce__Motor_G15_Driver: An RTOS is required with this function"
#endif

#define PIN_TX_RX_SEL         _RB4                      // Pin RB4 = pin to select TX or RX
                                                        // operations on the half-duplex
                                                        // UART communication.

///
/// Description		:
/// This is a low-level process to set and control the parameters of Cytron Technologies's G15 smart servo.
/// This smart servo is similar to Robotis's Dynamixel family of smart servo where instead of PWM digital
/// pulse, a series of digital data packets control the motor properties, such as rotation angle, rotation
/// speed and output torque.  The smart servo can also report back it's up-to-date status such as current
/// speed, output shaft angle, motor temperature, current (or loading) consumption and other status.
///
/// The global variable gnG15_Function indicates the current operation of this process.  If it is _G15_IDLE,
/// the process is waiting for further instruction and we can change this variable to initaite new operation
/// on the smart servo.
///
/// gnG15_Function value:
///                     _G15_IDLE = Smart servo motor is idle.
///                     _G15_INIT  = Initialize the smart servo motor.
///                     _G15_SET_LED = Set the smart servo indicator LED (on or off).
///                     _G15_READ_STATUS = Read the status of the smart servo.
///                     _G15_SET_WHEEL_MODE = Set the smart servo to continous rotation (wheel) mode.
///                                           Note, this is not yet implemented.
///                     _G15_SET_SERVO_MODE = Set the smart servo to position mode, e.g. we can
///                                           rotate the output shaft to a required angle and then
///                                           hold at the angle.
///                     _G15_WHEEL_MOVE = Rotate the output shaft continuously at a pre-defined speed
///                                       and direction.  The motor must be in Wheel Mode for this to
///                                       be effective.
///                     _G15_SERVO_MOVE = Move the output shaft to a pre-defined angle and hold.  The
///                                       motor must be in Servo Mode for this to be effective.
///
/// The driver provides the low-level details of controlling the motor, instead of setting the
/// global variable gnG15_Function and associated parameters gunG15_Param1 to gunG15_Param3,
/// the user should use the wrapper functions provided with this driver to setup the motor.
///
/// --- Usage examples ---
/// A typical sequence upon power up is as follows:
///
/// InitG15Motor(2);                // This will initialize the smart servo with ID = 2.
/// DelayMS(30);                    // Delay 30 msec for action to complete.  Else we can
///                                 // monitor gnG15_Function and proceed when it is equal
///                                 // to _G15_IDLE.
/// SetG15ServoMode(15,225,51,2)    // This will set the smart servo with ID = 2 into Servo Mode,
///                                 // with CW limit = 15 degree, CCW limit = 225 degree,
///                                 // and RPM limit = 51 (see function description).
/// DelayMS(15);
/// SetG15MotorAngleDegree(45,0,2); // Rotate the output shaft smart servo with ID = 2 to 45 degrees,
///                                 // speed = default RPM (e.g. 51 in this case)
///                                 // Note that the angle = 45 degree must be within the CW and
///                                 // CCW limits.
/// DelayMS(5);

#define	_G15_UART_BAUDRATE_kBPS	200.0           // Nominal UART datarate in kilobits-per-second.

//#########################################################################
//################ define - G15 Hex code table ######################

// EEPROM AREA (G15)
#define G15_EEPROM_MODEL_NUMBER_L               0x00
#define G15_EEPROM_MODEL_NUMBER_H               0x01
#define G15_EEPROM_VERSION                      0x02
#define G15_EEPROM_ID                           0x03
#define G15_EEPROM_BAUD_RATE                    0x04
#define G15_EEPROM_RETURN_DELAY_TIME            0x05
#define G15_EEPROM_CW_ANGLE_LIMIT_L             0x06
#define G15_EEPROM_CW_ANGLE_LIMIT_H             0x07
#define G15_EEPROM_CCW_ANGLE_LIMIT_L            0x08
#define G15_EEPROM_CCW_ANGLE_LIMIT_H            0x09
#define G15_EEPROM_LIMIT_TEMPERATURE            0x0B
#define G15_EEPROM_LOW_LIMIT_VOLTAGE            0x0C
#define G15_EEPROM_HIGN_LIMIT_VOLTAGE           0x0D
#define G15_EEPROM_MAX_TORQUE_L                 0x0E
#define G15_EEPROM_MAX_TORQUE_H                 0x0F
#define G15_EEPROM_RETURN_LEVEL                 0x10
#define G15_EEPROM_ALARM_LED                    0x11
#define G15_EEPROM_ALARM_SHUTDOWN               0x12
#define G15_EEPROM_DOWN_CALIBRATION_L           0x14
#define G15_EEPROM_DOWN_CALIBRATION_H           0x15
#define G15_EEPROM_UP_CALIBRATION_L             0x16
#define G15_EEPROM_UP_CALIBRATION_H             0x17

// RAM AREA (G15)
#define G15_RAM_TORQUE_ENABLE            	0x18
#define G15_RAM_LED                      	0x19
#define G15_RAM_CW_COMPLIANCE_MARGIN     	0x1A
#define G15_RAM_CCW_COMPLIANCE_MARGIN           0x1B
#define G15_RAM_CW_COMPLIANCE_SLOPE     	0x1C
#define G15_RAM_CCW_COMPLIANCE_SLOPE     	0x1D
#define G15_RAM_GOAL_POSITION_L          	0x1E
#define G15_RAM_GOAL_POSITION_H          	0x1F
#define G15_RAM_MOVING_SPEED_L             	0x20
#define G15_RAM_MOVING_SPEED_H             	0x21
#define G15_RAM_TORQUE_LIMIT_L           	0x22
#define G15_RAM_TORQUE_LIMIT_H           	0x23
#define G15_RAM_PRESENT_POSITION_L       	0x24
#define G15_RAM_PRESENT_POSITION_H       	0x25
#define G15_RAM_PRESENT_SPEED_L          	0x26
#define G15_RAM_PRESENT_SPEED_H          	0x27
#define G15_RAM_PRESENT_LOAD_L           	0x28
#define G15_RAM_PRESENT_LOAD_H           	0x29
#define G15_RAM_PRESENT_VOLTAGE          	0x2A
#define G15_RAM_PRESENT_TEMPERATURE      	0x2B
#define G15_RAM_REGISTER                 	0x2C
#define G15_RAM_MOVING                   	0x2E
#define G15_RAM_LOCK                     	0x2F
#define G15_RAM_PUNCH_L                  	0x30
#define G15_RAM_PUNCH_H                  	0x31


//#########################################################################
//################ Instruction commands Set ###############################
#define G15_COMMAND_PING                	0x01
#define G15_COMMAND_READ_DATA                   0x02
#define G15_COMMAND_WRITE_DATA          	0x03
#define G15_COMMAND_REG_WRITE_DATA      	0x04
#define G15_COMMAND_ACTION              	0x05
#define G15_COMMAND_RESET               	0x06
#define G15_COMMAND_SYNC_WRITE          	0x83

#define G15_BROADCAST_ID                	0xFE            // 254

void Proce_Motor_G15_Driver(TASK_ATTRIBUTE *ptrTask)
{
    unsigned char bytTemp;
    static int nCount = 0;
    static int nTimeOutCounter = 0;

    if (ptrTask->nTimer == 0)
    {
        switch (ptrTask->nState)
	{
            case 0: // State 0 - Initialization.
                gbytG15_Error = 0;                   // Initialize motor error flag.
                nCount = 0;
                gnMotorCount = 0;
                nCurrentMotorPtr = 0;               // Initialize current motor pointer.
                TRISBbits.TRISB4 = 0;               // Set RB4 as output.
                PIN_TX_RX_SEL = _G15_TX_MODE;       // Set to TX mode
                gnG15_Function = _G15_INIT;         // Set gnG15_Function to a value other than
                                                    // _G15_IDLE to indicate that driver is not
                                                    // ready or busy.
                OSSetTaskContext(ptrTask, 1, 1000*__NUM_SYSTEMTICK_MSEC); // Next state = 1,
                                                    // timer = 1000msec. A sufficient long
                                                    // period for the HC-05 bluetooth module
                                                    // to initialize.
                //OSSetTaskContext(ptrTask, 100, 1000000/__SYSTEMTICK_US); // Routines to reprogram new G15 servo.
            break;

            case 1: // State 1 - Change baud rate for UART2 module.
                U2BRG = (__FOSC_MHz/8)*(1000/_G15_UART_BAUDRATE_kBPS)-1; // Set baud rate to 200kbps.
                gnG15_Function = _G15_IDLE;         // Indicate that driver is ready.
                OSSetTaskContext(ptrTask, 2, 1);
                break;

            case 2: // State 2 - Check for functional mode.
                switch (gnG15_Function)
                {
                    case  _G15_INIT:
                        OSSetTaskContext(ptrTask, 10, 1);       // Next to state = 10, timer = 1.
                        break;

                    case _G15_SET_LED:
                        OSSetTaskContext(ptrTask, 24, 1);       // Next to state = 24, timer = 1.
                        break;

                    case _G15_SET_SERVO_MODE:
                        OSSetTaskContext(ptrTask, 30, 1);       // Next to state = 30, timer = 1.
                        break;

                    case _G15_SERVO_MOVE:
                        OSSetTaskContext(ptrTask, 40, 1);       // Next to state = 40, timer = 1.
                        break;

                    case _G15_SET_TORQUE:
                        OSSetTaskContext(ptrTask, 45, 1);       // Next to state = 45, timer = 1.
                        break;

                    //case _G15_SET_WHEEL_MODE:
                    //    PIN_TX_RX_SEL = _G15_TX_MODE;           // Set to TX mode
                    //    OSSetTaskContext(ptrTask, 50, 1);       // Next to state = 50, timer = 1.
                    //    break;

                    //case _G15_WHEEL_MOVE:
                        
                    //    OSSetTaskContext(ptrTask, 60, 1);       // Next to state = 60, timer = 1.
                    //    break;

                    default:                                    // gnG15_Function = _G15_IDLE.
                        nCount++;                               // Increment counter
                        if (nCount == 140)                      // About every 23.33 msec.
                        {                            
                            nCount = 0;
                            if (gnMotorCount > 0)                // If at least one motor already setup.
                            {

                                bytCurrentID = gobjG15Motor[nCurrentMotorPtr].bytID;
                                gnG15_Function = _G15_READ_STATUS;      // Indicate read motor status mode.
                                OSSetTaskContext(ptrTask, 90, 1);       // Next to state = 90, timer = 1.
                            }
                            else
                            {
                                OSSetTaskContext(ptrTask, 2, 1);       // Next to state = 2, timer = 1.
                            }
                        }
                        else
                        {
                            OSSetTaskContext(ptrTask, 2, 1);       // Next to state = 2, timer = 1.
                        }
                        break;
                }
                break;

            case 10: // State 10 - Initialize motor.
                     // Here we comment out the RESET command, according to the G15 datasheet,
                     // issuing this command will restore all the EEPROM of the G15 to the
                     // default values, including the device ID, which is set to 1.
                     // If multiple motors are linked in series, all the motors will
                     // default to ID = 1 !!! if this command is issued.
                PIN_TX_RX_SEL = _G15_TX_MODE;           // Set to TX mode
                //gbytTXbuffer2[0] = 0xFF;                 // Header 1.
                //gbytTXbuffer2[1] = 0xFF;                 // Header 2.
                //gbytTXbuffer2[2] = bytG15_ID;           // Device ID.
                //gbytTXbuffer2[3] = 2;                    // Packet length.
                //gbytTXbuffer2[4] = G15_COMMAND_RESET;    // Action.
                //gbytTXbuffer2[5] = ~(bytG15_ID + 2 + G15_COMMAND_RESET); //Checksum.
                //gbytTXbuflen2 = 6;                       // Set TX frame length.
                //gSCIstatus2.bTXRDY = 1;                  // Initiate TX.
                OSSetTaskContext(ptrTask, 11, 5*__NUM_SYSTEMTICK_MSEC);    // Next state = 11, timer = 5 msec.
                break;

             case 11: // State 11 - Set motor status packet return delay.
                 // Note: 1 May 2015, F.Kung:
                 // From probing, I noticed that the packet return delay is around 200 usec when
                 // the motor is responding to Read command, regardless of the value here.
                 // Perhaps it is an error in the motor firmware, or this parameter only
                 // affects normal response to Write command.
                gbytTXbuffer2[0] = 0xFF;                 // Header 1.
                gbytTXbuffer2[1] = 0xFF;                 // Header 2.
                gbytTXbuffer2[2] = bytG15_ID;           // Device ID.
                gbytTXbuffer2[3] = 4;                    // Packet length.
                gbytTXbuffer2[4] = G15_COMMAND_WRITE_DATA; // Action.
                gbytTXbuffer2[5] = G15_EEPROM_RETURN_DELAY_TIME; // Memory location.
                gbytTXbuffer2[6] = 100;                  // Return delay, 2usec x 100 = 200 usec.
                                                        // This should be greater than one clock tick.
                                                        // Recommends 2 clock ticks so that the controller has
                                                        // sufficient time to toggle the TX/RX select pin of
                                                        // the half-duplex transceiver.
                gbytTXbuffer2[7] = ~(bytG15_ID + 4 + G15_COMMAND_WRITE_DATA + G15_EEPROM_RETURN_DELAY_TIME + 100);
                gbytTXbuflen2 = 8;                       // Set TX frame length.
                gSCIstatus2.bTXRDY = 1;                  // Initiate TX.
                OSSetTaskContext(ptrTask, 12, 10*__NUM_SYSTEMTICK_MSEC);     // Next state = 12, timer = 10 msec.
                break;

             case 12: // State 12 - Set motor status packet return level.
                      // We will set the return level to 1 (default is 2).
                gbytTXbuffer2[0] = 0xFF;                 // Header 1.
                gbytTXbuffer2[1] = 0xFF;                 // Header 2.
                gbytTXbuffer2[2] = bytG15_ID;           // Device ID.
                gbytTXbuffer2[3] = 4;                    // Packet length.
                gbytTXbuffer2[4] = G15_COMMAND_WRITE_DATA;// Action.
                gbytTXbuffer2[5] = G15_EEPROM_RETURN_LEVEL;// Memory location.
                gbytTXbuffer2[6] = 1;                    // Only for PING and Read command.
                gbytTXbuffer2[7] = ~(bytG15_ID + 4 + G15_COMMAND_WRITE_DATA + G15_EEPROM_RETURN_LEVEL + 1);
                gbytTXbuflen2 = 8;                       // Set TX frame length.
                gSCIstatus2.bTXRDY = 1;                  // Initiate TX.
                OSSetTaskContext(ptrTask, 13, 5*__NUM_SYSTEMTICK_MSEC);      // Next state = 13, timer = 5 msec.
                break;

             case 13: // State 13 - Set alarm LED.
                gbytTXbuffer2[0] = 0xFF;                 // Header 1.
                gbytTXbuffer2[1] = 0xFF;                 // Header 2.
                gbytTXbuffer2[2] = bytG15_ID;           // Device ID.
                gbytTXbuffer2[3] = 4;                    // Packet length.
                gbytTXbuffer2[4] = G15_COMMAND_WRITE_DATA;// Action.
                gbytTXbuffer2[5] = G15_EEPROM_ALARM_LED;// Memory location.
                gbytTXbuffer2[6] = 0x25;                 // Activate alarm LED for
                                                         // 1) input voltage fault.
                                                         // 2) Overload fault.
                                                         // 3) Overheating fault.
                gbytTXbuffer2[7] = ~(bytG15_ID + 4 + G15_COMMAND_WRITE_DATA + G15_EEPROM_ALARM_LED + 0x25);
                gbytTXbuflen2 = 8;                       // Set TX frame length.
                gSCIstatus2.bTXRDY = 1;                  // Initiate TX.
                OSSetTaskContext(ptrTask, 14, 5*__NUM_SYSTEMTICK_MSEC);      // Next state = 14, timer = 5 msec.
                break;

             case 14: // State 14 - Set alarm shutdown.
                gbytTXbuffer2[0] = 0xFF;                 // Header 1.
                gbytTXbuffer2[1] = 0xFF;                 // Header 2.
                gbytTXbuffer2[2] = bytG15_ID;           // Device ID.
                gbytTXbuffer2[3] = 4;                    // Packet length.
                gbytTXbuffer2[4] = G15_COMMAND_WRITE_DATA;// Action.
                gbytTXbuffer2[5] = G15_EEPROM_ALARM_SHUTDOWN;// Memory location.
                gbytTXbuffer2[6] = 0x25;                 // Shutdown motor for
                                                         // 1) input voltage fault.
                                                         // 2) Overload fault.
                                                         // 3) Overheating fault.
                gbytTXbuffer2[7] = ~(bytG15_ID + 4 + G15_COMMAND_WRITE_DATA + G15_EEPROM_ALARM_SHUTDOWN + 0x25);
                gbytTXbuflen2 = 8;                       // Set TX frame length.
                gSCIstatus2.bTXRDY = 1;                  // Initiate TX.
                OSSetTaskContext(ptrTask, 15, 5*__NUM_SYSTEMTICK_MSEC);      // Next state = 15, timer = 5 msec.
                break;

            case 15: // State 15 - Clear receive buffer in preparation for normal operation.
                gSCIstatus2.bRXRDY = 0;                  // Reset valid data flag.
                gbytRXbufptr2 = 0;                       // Reset receive buffer pointer.
                PIN_ILED2 = 0;
                                                         // Set up the motor structure.
                OSSetTaskContext(ptrTask, 80, 1);       // Next state = 80, timer = 1.
                break;
                
            case 24: // State 24 - Set motor LED state.
                     // Parameter 1 = LED state, 1 = ON, 0 or otherwise = OFF.
                PIN_TX_RX_SEL = _G15_TX_MODE;           // Set to TX mode
                gbytTXbuffer2[0] = 0xFF;                 // Header 1.
                gbytTXbuffer2[1] = 0xFF;                 // Header 2.
                gbytTXbuffer2[2] = bytG15_ID;           // Device ID.
                gbytTXbuffer2[3] = 4;                    // Packet length.
                gbytTXbuffer2[4] = G15_COMMAND_WRITE_DATA;// Action.
                gbytTXbuffer2[5] = G15_RAM_LED;          // Memory location.
                if (gunG15_Param1 == _G15_ON)
                {
                    gbytTXbuffer2[6] = _G15_ON;          // Parameter 1.
                    gbytTXbuffer2[7] = ~(bytG15_ID + 4 + G15_COMMAND_WRITE_DATA + G15_RAM_LED + _G15_ON);    // Checksum.
                }
                else
                {
                    gbytTXbuffer2[6] = _G15_OFF;          // Parameter 1.
                    gbytTXbuffer2[7] = ~(bytG15_ID + 4 + G15_COMMAND_WRITE_DATA + G15_RAM_LED + _G15_OFF);    // Checksum.
                }
                gbytTXbuflen2 = 8;                       // Set TX frame length.
                gSCIstatus2.bTXRDY = 1;                  // Initiate TX.
                OSSetTaskContext(ptrTask, 80, 10*__NUM_SYSTEMTICK_MSEC);     // Next state = 80, timer = 10 msec.
                break;

            case 30: // State 30 - Set to servo mode, set angle limits.
                     // Parameter1 = CW limit, 0-1087, or 0.33 deg per unit.
                     // Parameter2 = CCW limit, 0-1087, or 0.33 deg per unit.
                     // Both CW and CCW limits will be limit to 1087.

                     // Parameter3 = Moving speed, 1-1023.  The speed resolution is
                     // roughly 0.098 RPM/unit, with a maximum speed of 100 RPM.
                     // If this parameter is > 1023, the codes will limit this to 0,
                     // whereby the motor will rotate at the highest RPM affored by the
                     // power supply without any speed control.
                PIN_TX_RX_SEL = _G15_TX_MODE;           // Set to TX mode
                gbytTXbuffer2[0] = 0xFF;                 // Header 1.
                gbytTXbuffer2[1] = 0xFF;                 // Header 2.
                gbytTXbuffer2[2] = bytG15_ID;           // Device ID.
                gbytTXbuffer2[3] = 7;                    // Packet length.
                gbytTXbuffer2[4] = G15_COMMAND_WRITE_DATA;
                gbytTXbuffer2[5] = G15_EEPROM_CW_ANGLE_LIMIT_L;

                // Servo mode.
                if (gunG15_Param1 > 1087)               // Limit parameter 1 to 1087.
                {
                    gunG15_Param1 = 1087;
                }
                if (gunG15_Param2 > 1087)               // Limit parameter 2 to 1087.
                {
                    gunG15_Param2 = 1087;
                }
                gbytTXbuffer2[6] = gunG15_Param1;        // CW limit L.
                gbytTXbuffer2[7] = gunG15_Param1>>8;     // CW limit H.
                gbytTXbuffer2[8] = gunG15_Param2;        // CCW limit L.
                gbytTXbuffer2[9] = gunG15_Param2>>8;     // CCW limit H.
                bytTemp = bytG15_ID + 7 + G15_COMMAND_WRITE_DATA + G15_EEPROM_CW_ANGLE_LIMIT_L;
                bytTemp = bytTemp + gbytTXbuffer2[6] + gbytTXbuffer2[7];
                bytTemp = bytTemp + gbytTXbuffer2[8] + gbytTXbuffer2[9];
                gbytTXbuffer2[10] = ~bytTemp;            // Checksum.
                gbytTXbuflen2 = 11;                      // Set TX frame length.
                gSCIstatus2.bTXRDY = 1;                  // Initiate TX.
                gobjG15Motor[nCurrentMotorPtr].bytMode = 0; // Indicate servo mode.
                OSSetTaskContext(ptrTask, 31, 10*__NUM_SYSTEMTICK_MSEC);      // Next state = 31, timer = 10 msec.
                break;

           case 31: // State 31 - Set moving speed under servo mode.
                    // NOTE: this state can also be called stand-alone.
                    // Parameter3 = Moving speed, 1-1023.  The speed resolution is
                    // roughly 0.098 RPM/unit, with a maximum speed of 100 RPM.
                    // If this parameter is > 1023, the codes will limit this to 0,
                    // whereby the motor will rotate at the highest RPM affored by the
                    // power supply without any speed control.
               if (gunG15_Param3 > 1023)                // Limit parameter 3 to 1023.
               {
                   gunG15_Param3 = 1023;
               }
               gbytTXbuffer2[0] = 0xFF;                 // Header 1.
               gbytTXbuffer2[1] = 0xFF;                 // Header 2.
               gbytTXbuffer2[2] = bytG15_ID;           // Device ID.
               gbytTXbuffer2[3] = 5;                    // Packet length.
               gbytTXbuffer2[4] = G15_COMMAND_WRITE_DATA;
               gbytTXbuffer2[5] = G15_RAM_MOVING_SPEED_L;
               gbytTXbuffer2[6] = gunG15_Param3;          // Speed L.
               gbytTXbuffer2[7] = gunG15_Param3>>8;       // Speed H.
               gbytTXbuffer2[8] = ~(bytG15_ID + 5 + G15_COMMAND_WRITE_DATA + G15_RAM_MOVING_SPEED_L + gbytTXbuffer2[6] + gbytTXbuffer2[7]);
               gbytTXbuflen2 = 9;                       // Set TX frame length.
               gSCIstatus2.bTXRDY = 1;                  // Initiate TX.
               gobjG15Motor[nCurrentMotorPtr].unSetSpeed = gunG15_Param3;   // Update the speed setting.
               
               OSSetTaskContext(ptrTask, 80, 2*__NUM_SYSTEMTICK_MSEC); // Next state = 80, timer = 2 msec.
               break;

            case 40: // State 40 - Move in servo mode.
                     // Angle is stored in parameter 1, 0-1087, or 0.33 deg / unit.
                PIN_TX_RX_SEL = _G15_TX_MODE;           // Set to TX mode
                if (gunG15_Param1 > 1087)               // Limit parameter 1 to 1087.
                {
                    gunG15_Param1 = 1087;
                }
                gbytTXbuffer2[0] = 0xFF;                 // Header 1.
                gbytTXbuffer2[1] = 0xFF;                 // Header 2.
                gbytTXbuffer2[2] = bytG15_ID;           // Device ID.
                gbytTXbuffer2[3] = 5;                    // Packet length.
                gbytTXbuffer2[4] = G15_COMMAND_WRITE_DATA;
                gbytTXbuffer2[5] = G15_RAM_GOAL_POSITION_L;
                gbytTXbuffer2[6] = gunG15_Param1;        // Angle L.
                gbytTXbuffer2[7] = gunG15_Param1>>8;     // Angle H.
                gbytTXbuffer2[8] = ~(bytG15_ID + 5 + G15_COMMAND_WRITE_DATA + G15_RAM_GOAL_POSITION_L + gbytTXbuffer2[6] + gbytTXbuffer2[7]);
                gbytTXbuflen2 = 9;                       // Set TX frame length.
                gSCIstatus2.bTXRDY = 1;                  // Initiate TX.
                if (gunG15_Param3 == 0)                  // If gunG15_Param3 = 0, no need to change
                {                                        // the moving speed.
                    OSSetTaskContext(ptrTask, 80, 10); // Next state = 80, timer = 10.
                }
                else
                {                                           // Update the moving speed too.
                    OSSetTaskContext(ptrTask, 41, 2*__NUM_SYSTEMTICK_MSEC);      // Next state = 41, timer = 2 msec.
                }
                break;

           case 41: // State 41 - Set moving speed under servo mode.
                    // Parameter3 = Moving speed, 1-1023.  The speed resolution is
                    // roughly 0.098 RPM/unit, with a maximum speed of 100 RPM.
                    // If this parameter is > 1023, the codes will limit this to 0,
                    // whereby the motor will rotate at the highest RPM affored by the
                    // power supply without any speed control.

               if (gunG15_Param3 > 1023)                // Limit parameter 3 to 1023.
               {
                   gunG15_Param3 = 1023;
               }
               gbytTXbuffer2[0] = 0xFF;                 // Header 1.
               gbytTXbuffer2[1] = 0xFF;                 // Header 2.
               gbytTXbuffer2[2] = bytG15_ID;           // Device ID.
               gbytTXbuffer2[3] = 5;                    // Packet length.
               gbytTXbuffer2[4] = G15_COMMAND_WRITE_DATA;
               gbytTXbuffer2[5] = G15_RAM_MOVING_SPEED_L;
               gbytTXbuffer2[6] = gunG15_Param3;          // Speed L.
               gbytTXbuffer2[7] = gunG15_Param3>>8;       // Speed H.
               gbytTXbuffer2[8] = ~(bytG15_ID + 5 + G15_COMMAND_WRITE_DATA + G15_RAM_MOVING_SPEED_L + gbytTXbuffer2[6] + gbytTXbuffer2[7]);
               gbytTXbuflen2 = 9;                       // Set TX frame length.
               gSCIstatus2.bTXRDY = 1;                  // Initiate TX.
               OSSetTaskContext(ptrTask, 80, 1*__NUM_SYSTEMTICK_MSEC); // Next state = 80, timer = 1 msec.
               break;

               // Note: 12 Aug 2015, this mode is still under test, not successful in reducing the motor torque.
            case 45: // State 45 - Set motor output torque.
                     // Parameter 1 = 0 to 1023.  If value = 0, motor will be shut down (e.g. they can be turned).
                     // 1023 corresponds to maximum torque available to the motor by the power supply.
               if (gunG15_Param1 > 1023)                // Limit parameter 1 to 1023.
               {
                   gunG15_Param1 = 1023;
               }
               gbytTXbuffer2[0] = 0xFF;                 // Header 1.
               gbytTXbuffer2[1] = 0xFF;                 // Header 2.
               gbytTXbuffer2[2] = bytG15_ID;            // Device ID.
               
               //gbytTXbuffer2[3] = 4;                    // Packet length.
               //gbytTXbuffer2[4] = G15_COMMAND_WRITE_DATA;
               //gbytTXbuffer2[5] = G15_RAM_TORQUE_ENABLE;
               //gbytTXbuffer2[6] = 0;                    // Disable torque.
               //gbytTXbuffer2[8] = ~(bytG15_ID + 4 + G15_COMMAND_WRITE_DATA + G15_RAM_TORQUE_ENABLE + gbytTXbuffer2[6]);
               //gbytTXbuflen2 = 8;                       // Set TX frame length.

               gbytTXbuffer2[3] = 5;                    // Packet length.
               gbytTXbuffer2[4] = G15_COMMAND_WRITE_DATA;
               gbytTXbuffer2[5] = G15_RAM_TORQUE_LIMIT_L;
               gbytTXbuffer2[6] = gunG15_Param1;          // Torque limit L.
               gbytTXbuffer2[7] = gunG15_Param1>>8;       // Torque limit H.
               gbytTXbuffer2[8] = ~(bytG15_ID + 5 + G15_COMMAND_WRITE_DATA + G15_RAM_TORQUE_LIMIT_L + gbytTXbuffer2[6] + gbytTXbuffer2[7]);
               gbytTXbuflen2 = 9;                       // Set TX frame length.

               gSCIstatus2.bTXRDY = 1;                  // Initiate TX.
               OSSetTaskContext(ptrTask, 80, 2*__NUM_SYSTEMTICK_MSEC); // Next state = 80, timer = 2 msec.
               break;

            case 80: // State 80 - End sequence.
                if (gSCIstatus2.bTXRDY == 0)             // Check if all data been send.
                {
                    gnG15_Function = _G15_IDLE;         // Back to default idle mode.
                    OSSetTaskContext(ptrTask, 2, 1);    // Next state = 2, timer = 1.
                }
                else
                {
                    OSSetTaskContext(ptrTask, 80, 1);   // Next state = 80, timer = 1.
                }
                break;

            case 90: // State 90 - Read motor status sequence.
                PIN_TX_RX_SEL = _G15_TX_MODE;                // Set to TX mode
                gbytTXbuffer2[0] = 0xFF;                     // Header 1.
                gbytTXbuffer2[1] = 0xFF;                     // Header 2.
                gbytTXbuffer2[2] = bytCurrentID;             // Device ID.
                gbytTXbuffer2[3] = 4;                        // Packet length.
                gbytTXbuffer2[4] = G15_COMMAND_READ_DATA;    // Action.
                gbytTXbuffer2[5] = G15_RAM_PRESENT_POSITION_L; // Memory location.
                gbytTXbuffer2[6] = 2;                        // Read 2 bytes.
                                                        
                gbytTXbuffer2[7] = ~(bytCurrentID + 4 + G15_COMMAND_READ_DATA + G15_RAM_PRESENT_POSITION_L + 2);
                gbytTXbuflen2 = 8;                           // Set TX frame length.
                gSCIstatus2.bTXRDY = 1;                      // Initiate TX.
                OSSetTaskContext(ptrTask, 91, 1);           // Next state = 91, timer = 1.
                break;

            case 91: // State 91 - Check if all transmission is completed.
                if (U2STAbits.TRMT == 1)                    // Check if last transmission is completed.
                {
                    PIN_TX_RX_SEL = _G15_RX_MODE;           // Set to RX mode
                    gSCIstatus2.bRXRDY = 0;                 // Reset valid data flag.
                    gbytRXbufptr2 = 0;                      // Reset pointer.
                    nTimeOutCounter = 0;                    // Clear timeout counter.
                    OSSetTaskContext(ptrTask, 92, 1);       // Next state = 92, timer = 1.
                }
                else
                {
                    OSSetTaskContext(ptrTask, 91, 1);       // Next state = 91, timer = 1.
                }
                break;

            case 92 : // State 92 - Read motor status sequence.
                // Packet:
                //      [0xFF] + [0xFF] + [ID] + [Length] + [Error] + [Position L] + [Position H] + [Checksum]
                // Index   0        1      2        3         4            5              6             7
                
                nTimeOutCounter++;                      // Check for time out.  If the motor does not responds skip,
                if (nTimeOutCounter == _STATUS_PACKET_TIMEOUT)  // and return function to idle.
                {
                    nCurrentMotorPtr++;                     // Point to next motor, reset if maximum no. of motor
                    if (nCurrentMotorPtr == gnMotorCount)              // reached.
                    {
                        nCurrentMotorPtr = 0;
                    }

                    //gnAudioTone[0] = 2;                 // Beep a tone to indicate error has occured, e.g. a motor
                    //gnAudioTone[1] = 0;                 // is missing or deactivated.
                    gnG15_Function = _G15_IDLE;         // Back to default idle mode.
                    OSSetTaskContext(ptrTask, 2, 1);    // Next state = 2, timer = 1.
                    break;
                }

                if (gSCIstatus2.bRXRDY == 1)             // Check if UART receive any data.
		{
                    if (gbytRXbufptr2 >= 8)              // Make sure 8 bytes are received.
                    {
                        if (gSCIstatus2.bRXOVF == 0)     // Make sure no overflow error.
			{
                            gobjG15Motor[nCurrentMotorPtr].bytStatus = gbytRXbuffer2[4];   // Get the error status of the motor.
                            if (gbytRXbuffer2[4] > 0)
                            {
                                gnAudioTone[0] = 4;      // Beep a tone to indicate error has occurred.
                                gnAudioTone[1] = 5;
                                gnAudioTone[2] = 8;
                                gnAudioTone[3] = 0;      //
                                gbytG15_Error = 1;       // Set motor error flag.
                            }
                            unPresentPosition = gbytRXbuffer2[6]; // Get the upper byte of the present position.
                            unPresentPosition = (unPresentPosition << 8) + gbytRXbuffer2[5]; // Get the lower byte of the present position.
                            gobjG15Motor[nCurrentMotorPtr].unPosition = unPresentPosition & 0x07FF;   // Mask out unused bits, only 10 bits are used.
                        }
                        else
                        {
                            gSCIstatus2.bRXOVF = 0;          // Reset overflow error flag.
                        }
                        gSCIstatus2.bRXRDY = 0;              // Reset valid data flag.
                        gbytRXbufptr2 = 0;                   // Reset pointer.
                        gnG15_Function = _G15_IDLE;         // Back to default idle mode.
                        OSSetTaskContext(ptrTask, 2, 1);    // Next state = 2, timer = 1.

                        nCurrentMotorPtr++;                     // Point to next motor, reset if maximum no. of motor
                        if (nCurrentMotorPtr == gnMotorCount)              // reached.
                        {
                            nCurrentMotorPtr = 0;
                        }
                    }
                    else
                    {
                        OSSetTaskContext(ptrTask, 92, 1);   // Next state = 92, timer = 1.
                    }


                }
                else
                {
                    OSSetTaskContext(ptrTask, 92, 1);       // Next state = 91, timer = 1.
                }
                        //gnG15_Function = _G15_IDLE;         // Back to default idle mode.
                        //OSSetTaskContext(ptrTask, 2, 1);    // Next state = 2, timer = 1.
                break;

                // --- Routines to set new G15 smart servo to new ID and new baud rate ---
                //
                // NOTE - IMPORTANT, 22 May 2015.
                // when running this routine, make sure to disable other processes that use this driver!!!
                //
            case 100: // State 100 - This routine is used to set the motor to factory default.
                      // After this the motor ID will be 1.
                #define     __CURRENT_G15_ID            1
                #define     __CURRENT_G15_BAUD_KBPS     200.0 // Set UART1 baud rate, this should be
                U2BRG = (__FOSC_MHz/8)*(1000/__CURRENT_G15_BAUD_KBPS)-1;
                                                            // similar to current motor baud rate.
                                                             // For new G15 smart servo this is 19.2kbps.
                gbytTXbuffer2[0] = 0xFF;                     // Header 1.
                gbytTXbuffer2[1] = 0xFF;                     // Header 2.
                gbytTXbuffer2[2] = __CURRENT_G15_ID;         // Original device ID.
                gbytTXbuffer2[3] = 2;                        // Packet length.
                gbytTXbuffer2[4] = G15_COMMAND_RESET;        // Action.
                gbytTXbuffer2[5] = ~(__CURRENT_G15_ID + 2 + G15_COMMAND_RESET); //Checksum.
                gbytTXbuflen2 = 6;                           // Set TX frame length.
                gSCIstatus2.bTXRDY = 1;                      // Initiate TX.
                OSSetTaskContext(ptrTask, 101, 1000);       // Next state = 1, timer = 1000.
                break;

            case 101: // State 101 - Change motor default baud rate to 200 kbps.
                #define     __NEW_G15_BAUD_KBPS     9       // THis will set G15 smart servo baud rate to 200 kbps.
                                                            // 4 for 400 kbps.
                                                            // 7 for 250 kbps.
                                                            // 9 for 200 kbps.
                                                            // 34 for 19.2 kbps.

                gbytTXbuffer2[0] = 0xFF;                     // Header 1.
                gbytTXbuffer2[1] = 0xFF;                     // Header 2.
                gbytTXbuffer2[2] = 1;                        // Default Device ID.
                gbytTXbuffer2[3] = 4;                        // Packet length.
                gbytTXbuffer2[4] = G15_COMMAND_WRITE_DATA;   // Action.
                gbytTXbuffer2[5] = G15_EEPROM_BAUD_RATE;     // Memory location.
                gbytTXbuffer2[6] = __NEW_G15_BAUD_KBPS;      // New baud rate setting.
 
                gbytTXbuffer2[7] = ~(1 + 4 + G15_COMMAND_WRITE_DATA + G15_EEPROM_BAUD_RATE +  __NEW_G15_BAUD_KBPS);
                gbytTXbuflen2 = 8;                           // Set TX frame length.
                gSCIstatus2.bTXRDY = 1;                      // Initiate TX.
                OSSetTaskContext(ptrTask, 102, 200);        // Next state = 2, timer = 200.
                break;

             case 102: // State 102 - Change motor default ID.
                #define     __NEW_G15_ID            18
                U1BRG = (__FOSC_MHz/8)*(1000/ _G15_UART_BAUDRATE_kBPS)-1;      // Set UART1 baud rate to 200 kbps.
                
                gbytTXbuffer2[0] = 0xFF;                     // Header 1.
                gbytTXbuffer2[1] = 0xFF;                     // Header 2.
                gbytTXbuffer2[2] = 1;                        // Default Device ID.
                gbytTXbuffer2[3] = 4;                        // Packet length.
                gbytTXbuffer2[4] = G15_COMMAND_WRITE_DATA;   // Action.
                gbytTXbuffer2[5] = G15_EEPROM_ID;            // Memory location.
                gbytTXbuffer2[6] = __NEW_G15_ID;             // New motor ID.
                gbytTXbuffer2[7] = ~(1 + 4 + G15_COMMAND_WRITE_DATA + G15_EEPROM_ID + __NEW_G15_ID);
                gbytTXbuflen2 = 8;                           // Set TX frame length.
                gSCIstatus2.bTXRDY = 1;                      // Initiate TX.
                OSSetTaskContext(ptrTask, 1, 200);          // Next state = 1, timer = 200.
                break;

            default:
               gnG15_Function = _G15_IDLE;                  // Back to default idle mode.
               OSSetTaskContext(ptrTask, 0, 1);             // Next to state = 0, timer = 1.
               break;
        }
    }
}

// Initialize the G15 smart servo and it's associated object.
// Return: 0 = operation fail.
//         valid address = operation success.
G15_MOTOR_ATTRIBUTE *InitG15Motor(unsigned char bytID)
{
    
    if (gnG15_Function == _G15_IDLE)    // Make sure system is not communicating with any motor.
    {
        if (gnMotorCount <  _G15_MAX_MOTOR)
        {
            gnG15_Function = _G15_INIT;
            bytG15_ID = bytID;
            gobjG15Motor[gnMotorCount].bytID = bytG15_ID;  // Assign ID.
            gobjG15Motor[gnMotorCount].bytStatus = 0;
            gobjG15Motor[gnMotorCount].bytMode = 0;         // Default servo mode.
            gobjG15Motor[gnMotorCount].unPosition = 0;
            gobjG15Motor[gnMotorCount].unSetSpeed = 0;
            return &gobjG15Motor[gnMotorCount];              // Return valid address to object.
        }
        else
        {
            return 0;                   // Operation fail, maximum no. of motor reached.
        }

    }
    else
    {
        return 0;                       // Indicate operation fail, system busy.
    }
}

// Set G15 smart servo to a required angle and the angular speed.
// nAngle = the required angle in degrees, from 0 to 360.
// nSpeed = The angular velocity in moving to the required output angle.
//  The speed resolution is roughly 0.098 RPM/unit, with a maximum
//  speed of 100 RPM.  A value of 0 will use the previous speed setting.
// bytID = Motor ID, 0-254.
// Return: 1 or >0 if operation is succesfull.
//         0 if operation fails.
int SetG15MotorAngleDegree(unsigned int nAngle, unsigned int nSpeed, unsigned char bytID)
{
    float fltAngle;
    unsigned int unAngle;

    if (gnG15_Function == _G15_IDLE)    // Make sure system is not communicating with any motor.
    {
        if (nAngle < 0)
        {
            nAngle = 0;
        }
        if (nAngle > 360)
        {
            nAngle = 360;
        }
        fltAngle = nAngle /0.33;    // 1 unit is around 0.33 degree.
        unAngle = fltAngle;         // Convert back to unsigned integer.
        gnG15_Function = _G15_SERVO_MOVE;
        bytG15_ID = bytID;
        gunG15_Param1 = unAngle;
        gunG15_Param3 = nSpeed;     // 1 unit is around 0.098 rpm or 0.588 deg/sec.
                                    // 17 units is roughly 10 deg/sec.
        return 1;                   // Indicate success.
    }
    else
    {
        return 0;                   // Indicate operation fail.
    }
}

// Set G15 smart servo to servo mode.
// nCWAngleLimit = Clockwise (CW) angle limit in degrees, 0 - 360.
// nCCWAngleLimit = Anti-clockwise (CW) angle limit in degrees, 0 - 360.
// nSpeed = The angular velocity.  The speed resolution is
// roughly 0.098 RPM/unit, with a maximum speed of 100 RPM.
// bytID = motor ID.
// Return: 1 or >0 if operation is succesfull.
//         0 if operation fails.
int SetG15ServoMode(unsigned int nCWAngleLimit, unsigned int nCCWAngleLimit, unsigned int nSpeed, unsigned char bytID)
{
    float fltAngle;
    unsigned int unAngle;

    if (gnG15_Function == _G15_IDLE)    // Make sure system is not communicating with any motor.
    {
        if (nCWAngleLimit < 0)
        {
            nCWAngleLimit = 0;
        }
        if (nCWAngleLimit > 360)
        {
            nCWAngleLimit = 360;
        }
        if (nCCWAngleLimit < 0)
        {
            nCCWAngleLimit = 0;
        }
        if (nCCWAngleLimit > 360)
        {
            nCCWAngleLimit = 360;
        }
        gnG15_Function = _G15_SET_SERVO_MODE;
        bytG15_ID = bytID;
        fltAngle = nCWAngleLimit /0.33;     // 1 unit is around 0.33 degree.
        unAngle = fltAngle;                 // Convert back to unsigned integer.
        gunG15_Param1 = unAngle;
        fltAngle = nCCWAngleLimit /0.33;    // 1 unit is around 0.33 degree.
        unAngle = fltAngle;                 // Convert back to unsigned integer.
        gunG15_Param2 = unAngle;
        gunG15_Param3 = nSpeed;             // 1 unit is around 0.098 rpm or 0.588 deg/sec.

        gobjG15Motor[gnMotorCount].bytMode = 0x80;         // Servo mode, indicate motor ready.
                                                          // MSb = 1: Motor ready.
                                                          //       0: Motor not fully initialized.
                                                          // LSb = 1: Wheel mode.
                                                          //       0: Servo mode.
        gobjG15Motor[gnMotorCount].unCWLimit = nCWAngleLimit;    // Store minimum and maximum angle
        gobjG15Motor[gnMotorCount].unCCWLimit = nCCWAngleLimit;  // limits in degrees.
        gobjG15Motor[gnMotorCount].unSetSpeed = nSpeed;
        gnMotorCount++;                      // Point to next motor.  Note that we cannot put this code
                                            // in the initialization routines as it takes a few hundread
                                            // microseconds to send the packet to the motor, so during
                                            // this period the motor concerned is essentially not
                                            // initialized.  If we tried to poll it, the motor will not
                                            // reply.
        return 1;                           // Indicate success.
    }
    else
    {
        return 0;                   // Indicate operation fail.
    }
}

// Turn on or off G15 smart servo LED.
// unLED: 1 = On LED.
//        otherwise = Off LED.
// Return: 1 or >0 if operation is succesfull.
//         0 if operation fails.
int SetG15MotorLED(unsigned int unLED, unsigned char bytID)
{

    if (gnG15_Function == _G15_IDLE)    // Make sure system is not communicating with any motor.
    {
        gnG15_Function = _G15_SET_LED;
        bytG15_ID = bytID;
        gunG15_Param1 = unLED;
        return 1;
    }
    else
    {
        return 0;
    }
}

// Set G15 smart servo output torque limit.
// unTorqueLimit: 0 - 1023. 0 = motor off, 1023 = maximum torque afforded by the
//                power supply.
// Return: 1 or >0 if operation is succesfull.
//         0 if operation fails.
int SetG15MotorTorque(unsigned int unTorqueLimit, unsigned char bytID)
{
    if (gnG15_Function == _G15_IDLE)    // Make sure system is not communicating with any motor.
    {
        if (unTorqueLimit > 1023)
        {
            unTorqueLimit = 1023;
        }
        gnG15_Function = _G15_SET_TORQUE;
        bytG15_ID = bytID;
        gunG15_Param1 = unTorqueLimit;
        return 1;
    }
    else
    {
        return 0;
    }
}

// Clear all the G15 smart servo mode settings.
void ClearAllG15MotorMode()
{
    gnMotorCount = 0;
    nCurrentMotorPtr = 0;
}