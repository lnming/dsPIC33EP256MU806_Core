//////////////////////////////////////////////////////////////////////////////////////////////
//
//	USER DRIVER ROUTINES DECLARATION (PROCESSOR DEPENDENT)
//
//  (c) Copyright 2015, Fabian Kung Wai Lee, Selangor, MALAYSIA
//  All Rights Reserved
//
//////////////////////////////////////////////////////////////////////////////////////////////
//
// File			: Drivers_G15_2_V100.c
// Author(s)		: Fabian Kung
// Last modified	: 11 August 2015
// Toolsuites		: Microchip MPLAB X IDE v2.20 or above
//                	  MPLAB XC16 C-Compiler v1.22 or above

// --- Engineering change notice ---
// 14 June 2015 - Basic working version.
// 7 July 2015 - Reduced the updating duration during angle and speed settings
//               in servo mode.

// Include common header to all drivers and sources.  Here absolute path is used.
// To edit if one change folder.
#include "../osmain.h"
#include "../Driver_UART3_V100.h"
#include "Driver_G15_2_V100.h"
#include "C:\FabianKung\Projects\Embedded_Systems\C_Library\dsPIC33E\Driver_Audio_V101.h"

// --- Public Variables ---
BYTE        gbytG15_Error2;          // 0 = no motor error.
                                    // 1 = one of the connected motors is faulty.
int         gnG15_Function2;             // To set the function of the driver.
#define     _G15_IDLE               -1
#define     _G15_INIT               0
#define     _G15_SET_LED            1
#define     _G15_READ_STATUS        2
#define     _G15_SET_WHEEL_MODE     3
#define     _G15_SET_SERVO_MODE     4
#define     _G15_WHEEL_MOVE         5
#define     _G15_SERVO_MOVE         6

int         gnMotorCount2;               // Indicate the number of motor connected to the controller.
G15_MOTOR_ATTRIBUTE gobjG15Motor2[_G15_MAX_MOTOR - 1];

BYTE        gbytG15_Status2;             // Error status flags for G15 motor.
UINT16      gunG15_Param12;              // Parameter 1.
UINT16      gunG15_Param22;              // Parameter 2.
UINT16      gunG15_Param32;              // Parameter 3.


// --- File Level Variables ---
BYTE        bytG15_ID2;                 // ID for G15 motor for general usage.
BYTE        bytCurrentID2;               // Temporary variable to store current motor ID.
UINT16      unPresentPosition2;          // Temporary variable to store current motor angle/position.
int         nCurrentMotorPtr2;           // Index to current motor data structure.


#define     _G15_ON                     1
#define     _G15_OFF                    0

#define     _G15_TX_MODE                1
#define     _G15_RX_MODE                0

#define     _BROADCAST_ID               0xFE            // 254

#define     _HEADER                    	0xFF            // 255

#define     _STATUS_PACKET_TIMEOUT	1000		// in no. of system ticks.

///
/// Process name	: Proce_Motor_G15_Driver2 - Driver for Cytron Technologies G15 smart servo.
///
/// Author		: Fabian Kung
///
/// Last modified	: 11 August 2015
///
/// Code version	: 0.91
///
/// Processor		: dsPIC33EP256MU80X family.
///
/// Processor/System Resource
/// PIN			:  PIN_TX_RX_SEL2 (RG8)
///
/// MODULE		:  UART3 (Internal)
///
/// DRIVER		:  Proce_UART3_Driver
///
/// RTOS		:  Ver 1 or above, round-robin scheduling.
///
/// Global variable	: gbytTXbuffer3, gbytRXbuffer3 and others.
///

#ifdef 				  __OS_VER		// Check RTOS version compatibility.
	#if 			  __OS_VER < 1
		#error "Proce__Motor_G15_Driver2: Incompatible OS version"
	#endif
#else
	#error "Proce__Motor_G15_Driver2: An RTOS is required with this function"
#endif

#define PIN_TX_RX_SEL2         _RG8                     // Pin RG8 = pin to select TX or RX
                                                        // operations on the half-duplex
                                                        // UART communication.

///
/// Description		:  See description for Driver_G15_V100


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

void Proce_Motor_G15_Driver2(TASK_ATTRIBUTE *ptrTask)
{
    
    unsigned char bytTemp;
    static int nCount = 0;
    static int nTimeOutCounter = 0;

    if (ptrTask->nTimer == 0)
    {
        switch (ptrTask->nState)
	{
            case 0: // State 0 - Initialization.
                gbytG15_Error2 = 0;                   // Initialize motor error flag.
                nCount = 0;
                gnMotorCount2 = 0;
                nCurrentMotorPtr2 = 0;               // Initialize current motor pointer.
                TRISGbits.TRISG8 = 0;               // Set RG8 as output.
                PIN_TX_RX_SEL2 = _G15_TX_MODE;       // Set to TX mode
                gnG15_Function2 = _G15_INIT;         // Set gnG15_Funcion2 to a value other than
                                                    // _G15_IDLE to indicate that driver is not
                                                    // ready or busy.
                OSSetTaskContext(ptrTask, 1, 1000*__NUM_SYSTEMTICK_MSEC); // Next state = 1,
                                                    // timer = 1000msec. A sufficient long
                                                    // period for the HC-05 bluetooth module
                                                    // to initialize.
            break;

            case 1: // State 1 - Change baud rate for UART3 module.
                U3BRG = (__FOSC_MHz/8)*(1000/_G15_UART_BAUDRATE_kBPS)-1; // Set baud rate to 200kbps.
                gnG15_Function2 = _G15_IDLE;         // Indicate that driver is ready.
                OSSetTaskContext(ptrTask, 2, 1);
                break;

            case 2: // State 2 - Check for functional mode.
                switch (gnG15_Function2)
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

                    //case _G15_SET_WHEEL_MODE:
                    //    PIN_TX_RX_SEL = _G15_TX_MODE;           // Set to TX mode
                    //    OSSetTaskContext(ptrTask, 50, 1);       // Next to state = 50, timer = 1.
                    //    break;

                    //case _G15_WHEEL_MOVE:
                        
                    //    OSSetTaskContext(ptrTask, 60, 1);       // Next to state = 60, timer = 1.
                    //    break;

                    default:                                    // gnG15_Funcion2 = _G15_IDLE.
                        nCount++;                               // Increment counter
                        if (nCount == 140)                      // About every 25 msec.
                        {                            
                            nCount = 0;
                            if (gnMotorCount2 > 0)                // If at least one motor already setup.
                            {

                                bytCurrentID2 = gobjG15Motor2[nCurrentMotorPtr2].bytID;
                                gnG15_Function2 = _G15_READ_STATUS;      // Indicate read motor status mode.
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
                PIN_TX_RX_SEL2 = _G15_TX_MODE;           // Set to TX mode
                //gbytTXbuffer3[0] = 0xFF;                 // Header 1.
                //gbytTXbuffer3[1] = 0xFF;                 // Header 2.
                //gbytTXbuffer3[2] = bytG15_ID2;           // Device ID.
                //gbytTXbuffer3[3] = 2;                    // Packet length.
                //gbytTXbuffer3[4] = G15_COMMAND_RESET;    // Action.
                //gbytTXbuffer3[5] = ~(bytG15_ID2 + 2 + G15_COMMAND_RESET); //Checksum.
                //gbytTXbuflen3 = 6;                       // Set TX frame length.
                //gSCIstatus3.bTXRDY = 1;                  // Initiate TX.
                OSSetTaskContext(ptrTask, 11, 5*__NUM_SYSTEMTICK_MSEC);    // Next state = 11, timer = 5 msec.
                break;

             case 11: // State 11 - Set motor status packet return delay.
                 // Note: 1 May 2015, F.Kung:
                 // From probing, I noticed that the packet return delay is around 200 usec when
                 // the motor is responding to Read command, regardless of the value here.
                 // Perhaps it is an error in the motor firmware, or this parameter only
                 // affects normal response to Write command.
                gbytTXbuffer3[0] = 0xFF;                 // Header 1.
                gbytTXbuffer3[1] = 0xFF;                 // Header 2.
                gbytTXbuffer3[2] = bytG15_ID2;           // Device ID.
                gbytTXbuffer3[3] = 4;                    // Packet length.
                gbytTXbuffer3[4] = G15_COMMAND_WRITE_DATA; // Action.
                gbytTXbuffer3[5] = G15_EEPROM_RETURN_DELAY_TIME; // Memory location.
                gbytTXbuffer3[6] = 100;                  // Return delay, 2usec x 100 = 200 usec.
                                                        // This should be greater than one clock tick.
                                                        // Recommends 2 clock ticks so that the controller has
                                                        // sufficient time to toggle the TX/RX select pin of
                                                        // the half-duplex transceiver.
                gbytTXbuffer3[7] = ~(bytG15_ID2 + 4 + G15_COMMAND_WRITE_DATA + G15_EEPROM_RETURN_DELAY_TIME + 100);
                gbytTXbuflen3 = 8;                       // Set TX frame length.
                gSCIstatus3.bTXRDY = 1;                  // Initiate TX.
                OSSetTaskContext(ptrTask, 12, 10*__NUM_SYSTEMTICK_MSEC);     // Next state = 12, timer = 10 msec.
                break;

             case 12: // State 12 - Set motor status packet return level.
                      // We will set the return level to 1 (default is 2).
                gbytTXbuffer3[0] = 0xFF;                 // Header 1.
                gbytTXbuffer3[1] = 0xFF;                 // Header 2.
                gbytTXbuffer3[2] = bytG15_ID2;           // Device ID.
                gbytTXbuffer3[3] = 4;                    // Packet length.
                gbytTXbuffer3[4] = G15_COMMAND_WRITE_DATA;// Action.
                gbytTXbuffer3[5] = G15_EEPROM_RETURN_LEVEL;// Memory location.
                gbytTXbuffer3[6] = 1;                    // Only for PING and Read command.
                gbytTXbuffer3[7] = ~(bytG15_ID2 + 4 + G15_COMMAND_WRITE_DATA + G15_EEPROM_RETURN_LEVEL + 1);
                gbytTXbuflen3 = 8;                       // Set TX frame length.
                gSCIstatus3.bTXRDY = 1;                  // Initiate TX.
                OSSetTaskContext(ptrTask, 13, 5*__NUM_SYSTEMTICK_MSEC);      // Next state = 13, timer = 5 msec.
                break;

             case 13: // State 13 - Set alarm LED.
                gbytTXbuffer3[0] = 0xFF;                 // Header 1.
                gbytTXbuffer3[1] = 0xFF;                 // Header 2.
                gbytTXbuffer3[2] = bytG15_ID2;           // Device ID.
                gbytTXbuffer3[3] = 4;                    // Packet length.
                gbytTXbuffer3[4] = G15_COMMAND_WRITE_DATA;// Action.
                gbytTXbuffer3[5] = G15_EEPROM_ALARM_LED;// Memory location.
                gbytTXbuffer3[6] = 0x25;                 // Activate alarm LED for
                                                         // 1) input voltage fault.
                                                         // 2) Overload fault.
                                                         // 3) Overheating fault.
                gbytTXbuffer3[7] = ~(bytG15_ID2 + 4 + G15_COMMAND_WRITE_DATA + G15_EEPROM_ALARM_LED + 0x25);
                gbytTXbuflen3 = 8;                       // Set TX frame length.
                gSCIstatus3.bTXRDY = 1;                  // Initiate TX.
                OSSetTaskContext(ptrTask, 14, 5*__NUM_SYSTEMTICK_MSEC);      // Next state = 14, timer = 5 msec.
                break;

             case 14: // State 14 - Set alarm shutdown.
                gbytTXbuffer3[0] = 0xFF;                 // Header 1.
                gbytTXbuffer3[1] = 0xFF;                 // Header 2.
                gbytTXbuffer3[2] = bytG15_ID2;           // Device ID.
                gbytTXbuffer3[3] = 4;                    // Packet length.
                gbytTXbuffer3[4] = G15_COMMAND_WRITE_DATA;// Action.
                gbytTXbuffer3[5] = G15_EEPROM_ALARM_SHUTDOWN;// Memory location.
                gbytTXbuffer3[6] = 0x25;                 // Shutdown motor for
                                                         // 1) input voltage fault.
                                                         // 2) Overload fault.
                                                         // 3) Overheating fault.
                gbytTXbuffer3[7] = ~(bytG15_ID2 + 4 + G15_COMMAND_WRITE_DATA + G15_EEPROM_ALARM_SHUTDOWN + 0x25);
                gbytTXbuflen3 = 8;                       // Set TX frame length.
                gSCIstatus3.bTXRDY = 1;                  // Initiate TX.
                OSSetTaskContext(ptrTask, 15, 5*__NUM_SYSTEMTICK_MSEC);      // Next state = 15, timer = 5 msec.
                break;

            case 15: // State 15 - Clear receive buffer in preparation for normal operation.
                gSCIstatus3.bRXRDY = 0;                  // Reset valid data flag.
                gbytRXbufptr3 = 0;                       // Reset receive buffer pointer.
                PIN_ILED2 = 0;
                                                         // Set up the motor structure.
                OSSetTaskContext(ptrTask, 80, 1);       // Next state = 80, timer = 1.
                break;
                
            case 24: // State 24 - Set motor LED state.
                     // Parameter 1 = LED state, 1 = ON, 0 or otherwise = OFF.
                PIN_TX_RX_SEL2 = _G15_TX_MODE;           // Set to TX mode
                gbytTXbuffer3[0] = 0xFF;                 // Header 1.
                gbytTXbuffer3[1] = 0xFF;                 // Header 2.
                gbytTXbuffer3[2] = bytG15_ID2;           // Device ID.
                gbytTXbuffer3[3] = 4;                    // Packet length.
                gbytTXbuffer3[4] = G15_COMMAND_WRITE_DATA;// Action.
                gbytTXbuffer3[5] = G15_RAM_LED;          // Memory location.
                if (gunG15_Param12 == _G15_ON)
                {
                    gbytTXbuffer3[6] = _G15_ON;          // Parameter 1.
                    gbytTXbuffer3[7] = ~(bytG15_ID2 + 4 + G15_COMMAND_WRITE_DATA + G15_RAM_LED + _G15_ON);    // Checksum.
                }
                else
                {
                    gbytTXbuffer3[6] = _G15_OFF;          // Parameter 1.
                    gbytTXbuffer3[7] = ~(bytG15_ID2 + 4 + G15_COMMAND_WRITE_DATA + G15_RAM_LED + _G15_OFF);    // Checksum.
                }
                gbytTXbuflen3 = 8;                       // Set TX frame length.
                gSCIstatus3.bTXRDY = 1;                  // Initiate TX.
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
                PIN_TX_RX_SEL2 = _G15_TX_MODE;           // Set to TX mode
                gbytTXbuffer3[0] = 0xFF;                 // Header 1.
                gbytTXbuffer3[1] = 0xFF;                 // Header 2.
                gbytTXbuffer3[2] = bytG15_ID2;           // Device ID.
                gbytTXbuffer3[3] = 7;                    // Packet length.
                gbytTXbuffer3[4] = G15_COMMAND_WRITE_DATA;
                gbytTXbuffer3[5] = G15_EEPROM_CW_ANGLE_LIMIT_L;

                // Servo mode.
                if (gunG15_Param12 > 1087)               // Limit parameter 1 to 1087.
                {
                    gunG15_Param12 = 1087;
                }
                if (gunG15_Param22 > 1087)               // Limit parameter 2 to 1087.
                {
                    gunG15_Param22 = 1087;
                }
                gbytTXbuffer3[6] = gunG15_Param12;        // CW limit L.
                gbytTXbuffer3[7] = gunG15_Param12>>8;     // CW limit H.
                gbytTXbuffer3[8] = gunG15_Param22;        // CCW limit L.
                gbytTXbuffer3[9] = gunG15_Param22>>8;     // CCW limit H.
                bytTemp = bytG15_ID2 + 7 + G15_COMMAND_WRITE_DATA + G15_EEPROM_CW_ANGLE_LIMIT_L;
                bytTemp = bytTemp + gbytTXbuffer3[6] + gbytTXbuffer3[7];
                bytTemp = bytTemp + gbytTXbuffer3[8] + gbytTXbuffer3[9];
                gbytTXbuffer3[10] = ~bytTemp;            // Checksum.
                gbytTXbuflen3 = 11;                      // Set TX frame length.
                gSCIstatus3.bTXRDY = 1;                  // Initiate TX.
                gobjG15Motor2[nCurrentMotorPtr2].bytMode = 0; // Indicate servo mode.
                OSSetTaskContext(ptrTask, 31, 10*__NUM_SYSTEMTICK_MSEC);      // Next state = 31, timer = 10 msec.
                break;

           case 31: // State 31 - Set moving speed under servo mode.
                    // NOTE: this state can also be called stand-alone.
                    // Parameter3 = Moving speed, 1-1023.  The speed resolution is
                    // roughly 0.098 RPM/unit, with a maximum speed of 100 RPM.
                    // If this parameter is > 1023, the codes will limit this to 0,
                    // whereby the motor will rotate at the highest RPM affored by the
                    // power supply without any speed control.
               if (gunG15_Param32 > 1023)                // Limit parameter 3 to 1023.
               {
                   gunG15_Param32 = 1023;
               }
               gbytTXbuffer3[0] = 0xFF;                 // Header 1.
               gbytTXbuffer3[1] = 0xFF;                 // Header 2.
               gbytTXbuffer3[2] = bytG15_ID2;           // Device ID.
               gbytTXbuffer3[3] = 5;                    // Packet length.
               gbytTXbuffer3[4] = G15_COMMAND_WRITE_DATA;
               gbytTXbuffer3[5] = G15_RAM_MOVING_SPEED_L;
               gbytTXbuffer3[6] = gunG15_Param32;          // Speed L.
               gbytTXbuffer3[7] = gunG15_Param32>>8;       // Speed H.
               gbytTXbuffer3[8] = ~(bytG15_ID2 + 5 + G15_COMMAND_WRITE_DATA + G15_RAM_MOVING_SPEED_L + gbytTXbuffer3[6] + gbytTXbuffer3[7]);
               gbytTXbuflen3 = 9;                       // Set TX frame length.
               gSCIstatus3.bTXRDY = 1;                  // Initiate TX.
               gobjG15Motor2[nCurrentMotorPtr2].unSetSpeed = gunG15_Param32;   // Update the speed setting.
               
               OSSetTaskContext(ptrTask, 80, 2*__NUM_SYSTEMTICK_MSEC); // Next state = 80, timer = 2 msec.
               break;

            case 40: // State 40 - Move in servo mode.
                     // Angle is stored in parameter 1, 0-1087, or 0.33 deg / unit.
                PIN_TX_RX_SEL2 = _G15_TX_MODE;           // Set to TX mode
                if (gunG15_Param12 > 1087)               // Limit parameter 1 to 1087.
                {
                    gunG15_Param12 = 1087;
                }
                gbytTXbuffer3[0] = 0xFF;                 // Header 1.
                gbytTXbuffer3[1] = 0xFF;                 // Header 2.
                gbytTXbuffer3[2] = bytG15_ID2;           // Device ID.
                gbytTXbuffer3[3] = 5;                    // Packet length.
                gbytTXbuffer3[4] = G15_COMMAND_WRITE_DATA;
                gbytTXbuffer3[5] = G15_RAM_GOAL_POSITION_L;
                gbytTXbuffer3[6] = gunG15_Param12;        // Angle L.
                gbytTXbuffer3[7] = gunG15_Param12>>8;     // Angle H.
                gbytTXbuffer3[8] = ~(bytG15_ID2 + 5 + G15_COMMAND_WRITE_DATA + G15_RAM_GOAL_POSITION_L + gbytTXbuffer3[6] + gbytTXbuffer3[7]);
                gbytTXbuflen3 = 9;                       // Set TX frame length.
                gSCIstatus3.bTXRDY = 1;                  // Initiate TX.
                if (gunG15_Param32 == 0)                  // If gunG15_Param32 = 0, no need to change
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
               if (gunG15_Param32 > 1023)                // Limit parameter 3 to 1023.
               {
                   gunG15_Param32 = 1023;
               }
               gbytTXbuffer3[0] = 0xFF;                 // Header 1.
               gbytTXbuffer3[1] = 0xFF;                 // Header 2.
               gbytTXbuffer3[2] = bytG15_ID2;           // Device ID.
               gbytTXbuffer3[3] = 5;                    // Packet length.
               gbytTXbuffer3[4] = G15_COMMAND_WRITE_DATA;
               gbytTXbuffer3[5] = G15_RAM_MOVING_SPEED_L;
               gbytTXbuffer3[6] = gunG15_Param32;          // Speed L.
               gbytTXbuffer3[7] = gunG15_Param32>>8;       // Speed H.
               gbytTXbuffer3[8] = ~(bytG15_ID2 + 5 + G15_COMMAND_WRITE_DATA + G15_RAM_MOVING_SPEED_L + gbytTXbuffer3[6] + gbytTXbuffer3[7]);
               gbytTXbuflen3 = 9;                       // Set TX frame length.
               gSCIstatus3.bTXRDY = 1;                  // Initiate TX.
               OSSetTaskContext(ptrTask, 80, 1*__NUM_SYSTEMTICK_MSEC); // Next state = 80, timer = 1 msec.
               break;
                
            case 80: // State 80 - End sequence.
                if (gSCIstatus3.bTXRDY == 0)             // Check if all data been send.
                {
                    gnG15_Function2 = _G15_IDLE;         // Back to default idle mode.
                    OSSetTaskContext(ptrTask, 2, 1);    // Next state = 2, timer = 1.
                }
                else
                {
                    OSSetTaskContext(ptrTask, 80, 1);   // Next state = 80, timer = 1.
                }
                break;

            case 90: // State 90 - Read motor status sequence.
                PIN_TX_RX_SEL2 = _G15_TX_MODE;                // Set to TX mode
                gbytTXbuffer3[0] = 0xFF;                     // Header 1.
                gbytTXbuffer3[1] = 0xFF;                     // Header 2.
                gbytTXbuffer3[2] = bytCurrentID2;             // Device ID.
                gbytTXbuffer3[3] = 4;                        // Packet length.
                gbytTXbuffer3[4] = G15_COMMAND_READ_DATA;    // Action.
                gbytTXbuffer3[5] = G15_RAM_PRESENT_POSITION_L; // Memory location.
                gbytTXbuffer3[6] = 2;                        // Read 2 bytes.
                                                        
                gbytTXbuffer3[7] = ~(bytCurrentID2 + 4 + G15_COMMAND_READ_DATA + G15_RAM_PRESENT_POSITION_L + 2);
                gbytTXbuflen3 = 8;                           // Set TX frame length.
                gSCIstatus3.bTXRDY = 1;                      // Initiate TX.
                OSSetTaskContext(ptrTask, 91, 1);           // Next state = 91, timer = 1.
                break;

            case 91: // State 91 - Check if all transmission is completed.
                if (U3STAbits.TRMT == 1)                    // Check if last transmission is completed.
                {
                    PIN_TX_RX_SEL2 = _G15_RX_MODE;           // Set to RX mode
                    gSCIstatus3.bRXRDY = 0;                 // Reset valid data flag.
                    gbytRXbufptr3 = 0;                      // Reset pointer.
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
                    nCurrentMotorPtr2++;                     // Point to next motor, reset if maximum no. of motor
                    if (nCurrentMotorPtr2 == gnMotorCount2)              // reached.
                    {
                        nCurrentMotorPtr2 = 0;
                    }

                    //gnAudioTone[0] = 2;                 // Beep a tone to indicate error has occured, e.g. a motor
                    //gnAudioTone[1] = 0;                 // is missing or deactivated.
                    gnG15_Function2 = _G15_IDLE;         // Back to default idle mode.
                    OSSetTaskContext(ptrTask, 2, 1);    // Next state = 2, timer = 1.
                    break;
                }

                if (gSCIstatus3.bRXRDY == 1)             // Check if UART receive any data.
		{
                    if (gbytRXbufptr3 >= 8)              // Make sure 8 bytes are received.
                    {
                        if (gSCIstatus3.bRXOVF == 0)     // Make sure no overflow error.
			{
                            gobjG15Motor2[nCurrentMotorPtr2].bytStatus = gbytRXbuffer3[4];   // Get the error status of the motor.
                            if (gbytRXbuffer3[4] > 0)
                            {
                                gnAudioTone[0] = 4;      // Beep a tone to indicate error has occurred.
                                gnAudioTone[1] = 5;
                                gnAudioTone[2] = 8;
                                gnAudioTone[3] = 0;      //
                                gbytG15_Error2 = 1;       // Set motor error flag.
                            }
                            unPresentPosition2 = gbytRXbuffer3[6]; // Get the upper byte of the present position.
                            unPresentPosition2 = (unPresentPosition2 << 8) + gbytRXbuffer3[5]; // Get the lower byte of the present position.
                            gobjG15Motor2[nCurrentMotorPtr2].unPosition = unPresentPosition2 & 0x07FF;   // Mask out unused bits, only 10 bits are used.
                        }
                        else
                        {
                            gSCIstatus3.bRXOVF = 0;          // Reset overflow error flag.
                        }
                        gSCIstatus3.bRXRDY = 0;              // Reset valid data flag.
                        gbytRXbufptr3 = 0;                   // Reset pointer.
                        gnG15_Function2 = _G15_IDLE;         // Back to default idle mode.
                        OSSetTaskContext(ptrTask, 2, 1);    // Next state = 2, timer = 1.

                        nCurrentMotorPtr2++;                     // Point to next motor, reset if maximum no. of motor
                        if (nCurrentMotorPtr2 == gnMotorCount2)              // reached.
                        {
                            nCurrentMotorPtr2 = 0;
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
                break;

            default:
               gnG15_Function2 = _G15_IDLE;                  // Back to default idle mode.
               OSSetTaskContext(ptrTask, 0, 1);             // Next to state = 0, timer = 1.
               break;
        }
    }
}

// Initialize the G15 smart servo and it's associated object.
// Return: 0 = operation fail.
//         valid address = operation success.
G15_MOTOR_ATTRIBUTE *InitG15Motor2(unsigned char bytID)
{
    if (gnG15_Function2 == _G15_IDLE)    // Make sure system is not communicating with any motor.
    {
        if (gnMotorCount2 <  _G15_MAX_MOTOR)
        {
            gnG15_Function2 = _G15_INIT;
            bytG15_ID2 = bytID;
            gobjG15Motor2[gnMotorCount2].bytID = bytG15_ID2;  // Assign ID.
            gobjG15Motor2[gnMotorCount2].bytStatus = 0;
            gobjG15Motor2[gnMotorCount2].bytMode = 0;         // Default servo mode.
            gobjG15Motor2[gnMotorCount2].unPosition = 0;
            gobjG15Motor2[gnMotorCount2].unSetSpeed = 0;
            return &gobjG15Motor2[gnMotorCount2];              // Return valid address to object.
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
int SetG15MotorAngleDegree2(unsigned int nAngle, unsigned int nSpeed, unsigned char bytID)
{
    float fltAngle;
    unsigned int unAngle;

    if (gnG15_Function2 == _G15_IDLE)    // Make sure system is not communicating with any motor.
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
        gnG15_Function2 = _G15_SERVO_MOVE;
        bytG15_ID2 = bytID;
        gunG15_Param12 = unAngle;
        gunG15_Param32 = nSpeed;    // 1 unit is around 0.098 rpm or 0.588 deg/sec.
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
int SetG15ServoMode2(unsigned int nCWAngleLimit, unsigned int nCCWAngleLimit, unsigned int nSpeed, unsigned char bytID)
{
    float fltAngle;
    unsigned int unAngle;

    if (gnG15_Function2 == _G15_IDLE)    // Make sure system is not communicating with any motor.
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
        gnG15_Function2 = _G15_SET_SERVO_MODE;
        bytG15_ID2 = bytID;
        fltAngle = nCWAngleLimit /0.33;     // 1 unit is around 0.33 degree.
        unAngle = fltAngle;                 // Convert back to unsigned integer.
        gunG15_Param12 = unAngle;
        fltAngle = nCCWAngleLimit /0.33;    // 1 unit is around 0.33 degree.
        unAngle = fltAngle;                 // Convert back to unsigned integer.
        gunG15_Param22 = unAngle;
        gunG15_Param32 = nSpeed;

        gobjG15Motor2[gnMotorCount2].bytMode = 0x80;         // Servo mode, indicate motor ready.
                                                          // MSb = 1: Motor ready.
                                                          //       0: Motor not fully initialized.
                                                          // LSb = 1: Wheel mode.
                                                          //       0: Servo mode.
        gobjG15Motor2[gnMotorCount2].unCWLimit = nCWAngleLimit;    // Store minimum and maximum angle
        gobjG15Motor2[gnMotorCount2].unCCWLimit = nCCWAngleLimit;  // limits in degrees.
        gobjG15Motor2[gnMotorCount2].unSetSpeed = nSpeed;
        gnMotorCount2++;                      // Point to next motor.  Note that we cannot put this code
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
int SetG15MotorLED2(unsigned int unLED, unsigned char bytID)
{
    if (gnG15_Function2 == _G15_IDLE)    // Make sure system is not communicating with any motor.
    {
        gnG15_Function2 = _G15_SET_LED;
        bytG15_ID2 = bytID;
        gunG15_Param12 = unLED;
        return 1;
    }
    else
    {
        return 0;
    }
}

// Clear all the G15 smart servo mode settings.
void ClearAllG15MotorMode2()
{
    gnMotorCount2 = 0;
    nCurrentMotorPtr2 = 0;
}