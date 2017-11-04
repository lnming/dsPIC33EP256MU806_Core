//////////////////////////////////////////////////////////////////////////////////////////////
//
//	USER DRIVER ROUTINES DECLARATION (PROCESSOR DEPENDENT)
//
//  (c) Copyright 2017, Fabian Kung Wai Lee, Selangor, MALAYSIA
//  All Rights Reserved  
//   
//////////////////////////////////////////////////////////////////////////////////////////////
//
// File             : Drivers_A4988_V100.c
// Author(s)		: Fabian Kung
// Last modified	: 19 Sep 2017
// Toolsuites		: Microchip MPLAB-X IDE v3.60 or above
//                	  MPLAB XC16 C-Compiler v1.31 or above
//			

// Include common header to all drivers and sources.  Here absolute path is used.
// To edit if one change folder
#include "../osmain.h"


// NOTE: Public function prototypes are declared in the corresponding *.h file.

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

DSPIC33E_A4988_DRIVER    gobjDriverA4988;
long        gnDistanceMoveLW = 0;   // Distance traveled by left wheel in no. of steps, 32 bits integer.
long        gnDistanceMoveRW = 0;   // Distance traveled by right wheel in no. of steps, 32 bits integer.
long        gnDistanceMoveW = 0;    // Average distance traveled by both wheels in no. of steps, 32 bits integer.
int         gnHeading = 0;          // This indicates the direction, essentially the difference between right and left 
                                    // wheel distance.  Facing right if > 0, and facing left if < 0.			
unsigned int    gunDeadBandThres = 6; // Deadband threshold, speed setting less than this magnitude will be ignored.
//
// --- PRIVATE FUNCTION PROTOTYPES ---
//

//
// --- PRIVATE VARIABLES ---
//

#define PIN_STEPPER_DIR1                 _RB6 
#define PIN_STEPPER_DIR2                 _RB7 
#define PIN_A4988_ENABLE                 _RB9
#define PIN_STEPPER_STEP1                _RF4
#define PIN_STEPPER_STEP2                _RF5

#define _DISABLE_A4988                   1
#define _ENABLE_A4988                    0

//
// --- Process Level Constants Definition --- 
//


///
/// Process name	: Proce_A4988_Driver
///
/// Author			: Fabian Kung
///
/// Last modified   : 15 Sep 2017
///
/// Code Version	: 0.92
///
/// Processor		: dsPIC33EP256MU80X family.
///                   dsPIC33EP512MC80X family.
///
/// Processor/System Resources 
/// PINS			: 1. RF4 - Step output to A4988 chip1.
///                   2. RB6 - Direction output to A4988 chip1. 
///                   3. RF5 - Step output to A4988 chip2.
///                   4. RB7 - Direction output to A4988 chip2. 
///                   5. RB9 - Enable pin control for both A4988 chips.
///                            Low = disable FET outputs of both chips.
///                            High = enable FET outputs of both chips.
/// 
/// MODULES			: 1. OC7 (Internal) for Motor1.
///                   2. OC8 (Internal) for Motor2.
///
/// RTOS			: Ver 1 or above, round-robin scheduling.
///
/// Global variables	: gobjDriverA4988
///                       gnDistanceMoveLW
///                       gnDistanceMoveRW
///                       gnDistanceMoveW
///                       gnHeading


#ifdef 				  __OS_VER			// Check RTOS version compatibility.
	#if 			  __OS_VER < 1
		#error "Proce_A4988_Driver: Incompatible OS version"
	#endif
#else
	#error "Proce_A4988_Driver: An RTOS is required with this function"
#endif

/// Description	: Subroutine to drive the A4988 or compatible (such as DRV8825) stepper motor driver chips.
///               This driver supports two A4988 chips, e.g. two stepper motors, driven in half-step (0.9 
///               degree) mode.
///               The codes make uses of linearization algorithm between the speed setting and 
///               pulse duration from J. Brokking's codes for arduino,
///               in: http://www.brokking.net/yabr_main.html
///               The process uses the internal Output Compare (OC) of the dsPIC33 processors in PWM
///               mode to generate the step pulses.  Another I/O pin is used to control the DIR (direction).
///               In addition to driving the stepper motor, this process also calculates the instantaneous
///               rotational velocity of the motor shaft by sampling
///                    
/// Example of usage: 
/// 1) To start the sampling and conversion, we first set gobjDriverA4988.unEn4988 to 1. 
/// 2) Then we set nSpeed1 or nSpeed2 to between -400 and +400.  Values between -4 to +4
///    will stop the motor.  The driver codes has a linearization routines, which will
///    generate almost linear angular velocity output. 
///    For nSpeedx = 5, angular speed = 0.115 revolution/sec.
///        nSpeedx = 400, angular speed = 3.346 revolution/sec.
/// 3) The global variables gnDistanceMoveLW, gnDistanceMoveRW, gnDistanceMoveW and gnHeading
///    stores the instantaneous left wheel distance, right wheel distance, average wheel 
///    distance and the heading.  User routines can read this value as frequently as needed.
///    These global variables will be cleared to 0 whenever the driver is disabled.

#if __FOSC_MHz < 116    // Check processor clock frequency, this driver may not work
                        // properly if clock frequency is below 116 MHz.
    #error "Proce_A4988_Driver: Processor frequency is less than 59 MHz, this driver may not work properly"
#endif

#define  T3_PS	64                                                           // TIMER3 Prescalar.

#define	_STEPPER_MOTOR_PULSE_WIDTH      9       // The pulse width for the STEP pulse, in multiples of the period resolution.
#define _STEPPER_MOTOR_MAX_PERIOD_US    50000   // Largest period is 50 msec, or 20 Hz.
#define _STEPPER_MOTOR_MIN_PERIOD_US    200     // The smallest period is 0.2 msec, or 5.00 kHz.
#define _STEPPER_MOTOR_PERIOD_RESOLUTION_US    20      // This is the smallest increment/decrement.
#define _STEPPER_MOTOR_COUNT            _STEPPER_MOTOR_PERIOD_RESOLUTION_US/(__TCLK_US*T3_PS)
#define _STEPPER_MOTOR_MIN_SPEED        0
#define _STEPPER_MOTOR_MAX_SPEED        400


void Proce_A4988_Driver(TASK_ATTRIBUTE *ptrTask)
{
    int nTemp, nTemp2;
    long nlTemp;
    static int  nLeftOutHigh = 0;
    static int  nRightOutHigh = 0;
    
    if (ptrTask->nTimer == 0)
    {
        switch (ptrTask->nState)
        {
            case 0: // State 0 - Initialization OC7 and OC8 to PWM mode, with clock source 
                    // provided by Timer3.
                
                TRISBbits.TRISB6 = 0;                   // Set RB6 as output.
                TRISBbits.TRISB7 = 0;                   // Set RB7 as output.
                TRISBbits.TRISB9 = 0;                   // Set RB9 as output.
                TRISFbits.TRISF4 = 0;                   // Set RF4 as output.
                TRISFbits.TRISF5 = 0;                   // Set RF5 as output.
                
                PIN_A4988_ENABLE = _DISABLE_A4988;      // First disable all the FET outputs of the A4988 chips.
                
                // Settings of remappable output pin. The cluster of bits RPnR
                // control the mapping of the pin internal peripheral output.
                // Note n = integer.  If we refer to the datasheet, pin RF4
                // is also called RP100, and RP100R bits are contained in
                // the special function register RPOR9 in the controller.
                RPOR9bits.RP100R = 0b010110;             // RP100 or RF4 connected to OC7's output.
                RPOR9bits.RP101R = 0b010111;             // RP101 or RF5 connected to OC8's output.
                
                OC7CON1 = 0;                            // Initialize both control registers for OC7.     
                OC7CON2 = 0;
                OC8CON1 = 0;                            // Initialize both control registers for OC8.     
                OC8CON2 = 0;                
                
                OC8CON1bits.OCSIDL = 1;                 // Output Compare 1 halts when in idle mode.
                OC8CON2bits.SYNCSEL = 0b11111;          // 0x1F, No trigger or sync source is selected.  
                                                        // The internal timer OC8TMR resets when it reaches 
                                                        // the value of OC8RS.
                OC8CON1bits.OCTSEL = 0x01;              // Timer3 provides the clock source. 
                //OC8CON1bits.OCM = 0b110;                // Set OC8 to PWM mode.                 
                                                        // Note: 4/7/2017 F.Kung
                                                        // I discovered that this frequency is given by:
                                                        // fclock = fsource / pre-scalar
                                                        // fsource is the input clock source for Timer 3,
                                                        // which is the peripheral clock in this case.
                OC7CON1bits.OCSIDL = 1;                 // Same settings as OC8.
                OC7CON2bits.SYNCSEL = 0b11111;
                OC7CON1bits.OCTSEL = 0x01;
                //OC7CON1bits.OCM = 0b110;
                OC8RS = 0;  
                OC7RS = 0;
                OC8R = 0;
                OC7R = 0;
                OC8TMR = 0x0000;
                OC7TMR = 0x0000;       
                
                T3CONbits.TSIDL = 1;                    // Stop TIMER3 when in idle mode.
                T3CONbits.TCS = 0;                      // Clock source for TIMER3 is peripheral clock (Tclk/2).
                T3CONbits.TCKPS = 0b10;                 // TIMER3 prescalar = 1:64.
                T3CONbits.TON = 1;                      // Turn on TIMER3.
                TMR3 = 0x0000;                          // Reset TIMER.
                
                gobjDriverA4988.nSpeed1 = 0;
                gobjDriverA4988.nSpeed2 = 0;
                OSSetTaskContext(ptrTask, 1, 100*__NUM_SYSTEMTICK_MSEC); // Next state = 1, timer = 100 msec.						 
            break;

            case 1: // State 1 - Wait for module to be enabled.
                if (gobjDriverA4988.unEn4988 == 1)      // Check if module is enabled.
                {
                    OSSetTaskContext(ptrTask, 2, 1);    // Next state = 2, timer = 1.
                    if (OC8CON1bits.OCM != 0b110)       // If the OC8 is not in PWM mode,
                    {                                   // set it to PWM mode.
                        OC8CON1bits.OCM = 0b110;        
                    }
                    if (OC7CON1bits.OCM != 0b110)       // If the OC7 is not in PWM mode,
                    {                                   // set OC7 to PWM mode.
                        OC7CON1bits.OCM = 0b110;        
                    }
                }
                else
                {
                    PIN_A4988_ENABLE = _DISABLE_A4988;  // Cut-off the power to stepper motor.
                    OC7CON1bits.OCM = 0b000;            // Turn off OC7 and OC8.
                    OC8CON1bits.OCM = 0b000;
                    gobjDriverA4988.nSpeed1 = 0;
                    gobjDriverA4988.nSpeed2 = 0;
                    gnDistanceMoveLW = 0;
                    gnDistanceMoveRW = 0;
                    gnDistanceMoveW = 0;
                    gnHeading = 0;
                    OSSetTaskContext(ptrTask, 1, 1);    // Next state = 1, timer = 1.
                }                
                
                // Update distance counters by polling the OC7 and OC8 output pins.  Also compute average
                // distance and heading.
                if ((nLeftOutHigh == 0) && (PIN_STEPPER_STEP1 == 1)) // Check for low-to-high transition on left motor output.
                {
                    if (PIN_STEPPER_DIR1 == 0)          // Check for forward direction.
                    {
                        gnDistanceMoveLW++;
                    }
                    else
                    {
                        gnDistanceMoveLW--;
                    }
                    nLeftOutHigh = 1;                   // Indicate that low-to-high transition has already occur.
                }
                else if ((nLeftOutHigh == 1) && (PIN_STEPPER_STEP1 == 0))
                {
                    nLeftOutHigh = 0;
                }
                if ((nRightOutHigh == 0) && (PIN_STEPPER_STEP2 == 1)) // Check for low-to-high transition on right motor output.
                {
                    if (PIN_STEPPER_DIR2 == 1)              // Check for forward direction.
                    {
                        gnDistanceMoveRW++;
                    }
                    else
                    {
                        gnDistanceMoveRW--;
                    }
                    nRightOutHigh = 1;                   // Indicate that low-to-high transition has already occur.
                }
                else if ((nRightOutHigh == 1) && (PIN_STEPPER_STEP2 == 0))
                {
                    nRightOutHigh = 0;
                } 
                nlTemp = gnDistanceMoveLW + gnDistanceMoveRW;       // Compute the average distance traveled.  Here a long integer
                gnDistanceMoveW = nlTemp >> 1;                      // is used to prevent overflow. Divide by 2.
                gnHeading = gnDistanceMoveRW - gnDistanceMoveLW;    // Compute the direction or heading.
            break;

            case 2: // State 2 - Scan unSpeed1 and set the OC7 module accordingly.  
                nTemp = gobjDriverA4988.nSpeed1;                
                if (nTemp < 0)      
                {
                    PIN_STEPPER_DIR1 = 1;
                    nTemp = -nTemp;
                }
                else
                {
                    PIN_STEPPER_DIR1 = 0; 
                }
                
                if (nTemp > gunDeadBandThres)      // There is a deadband of 4 units.  For two-wheels robot 
                {                   // a small deadband can reduce unwanted oscillation.    
                   // Compensation for non-linear relationship between angular 
                   // velocity and pulse period. Based J. Brokking codes for 
                   // Arduino. For positive speed setting:        
                   // NewSpeedSetting = 405 - 5500/(SpeedSettings + 9)
                   // For negative speed setting:
                   // NewSpeedSetting = -405 - 5500/(SpeedSettings - 9)
                   // Since this is inversely proportional to the step interval, 
                   // we subtract from the maximum time interval.
                   PIN_A4988_ENABLE = _ENABLE_A4988;
                   nTemp2 = 5500/(nTemp+9);
                   //nTemp2 = 405 - nTemp2;  
                   OC7R = _STEPPER_MOTOR_PULSE_WIDTH*_STEPPER_MOTOR_COUNT;      // Set OC7 output pin to high. The width of the 
                   //OC7RS = (405-nTemp2)*_STEPPER_MOTOR_COUNT;                   // the pulse is determined by 
                   OC7RS = nTemp2*_STEPPER_MOTOR_COUNT;                         // _STEPPER_MOTOR_PULSE_WIDTH*_STEPPER_MOTOR_COUNT
                                                                                // while the period is set by OC7RS value.  We must
                                                                                // make sure OC7RS >= OC7R at all times! 
                }
                else
                {
                    PIN_A4988_ENABLE = _DISABLE_A4988;                          // Cut-off the power to stepper motor when
                                                                                // not turning.  This depends on application, 
                                                                                // it means there will be no holding torque
                                                                                // when the motor is not turning.        
                    OC7R = 0;
                    OC7RS = 0;                    
                }                                                               
                                                                                               
                // Note: 30 Aug 2017, the above codes to set the Output Compare unit needs to execute first
                // before we update the distance ticks.
                if ((nLeftOutHigh == 0) && (PIN_STEPPER_STEP1 == 1)) // Check for low-to-high transition on left motor output.
                {
                    if (PIN_STEPPER_DIR1 == 0)          // Check for forward direction.
                    {
                        gnDistanceMoveLW++;
                    }
                    else
                    {
                        gnDistanceMoveLW--;
                    }
                    nLeftOutHigh = 1;                   // Indicate that low-to-high transition has already occur.
                }
                else if ((nLeftOutHigh == 1) && (PIN_STEPPER_STEP1 == 0))
                {
                    nLeftOutHigh = 0;
                }                
                if ((nRightOutHigh == 0) && (PIN_STEPPER_STEP2 == 1))    // Check for low-to-high transition on right motor output.
                {
                    if (PIN_STEPPER_DIR2 == 1)              // Check for forward direction.
                    {
                        gnDistanceMoveRW++;
                    }
                    else
                    {
                        gnDistanceMoveRW--;
                    }
                    nRightOutHigh = 1;                   // Indicate that low-to-high transition has already occur.
                }
                else if ((nRightOutHigh == 1) && (PIN_STEPPER_STEP2 == 0))
                {
                    nRightOutHigh = 0;
                }                           
                OSSetTaskContext(ptrTask, 3, 1);                                // Next state = 3, timer = 1.                
            break;

            case 3: // State 3 - Scan unSpeed2 and set the OC8 module accordingly. 
                nTemp = gobjDriverA4988.nSpeed2;                
                if (nTemp < 0)
                {
                    PIN_STEPPER_DIR2 = 0;
                    nTemp = -nTemp;
                }
                else
                {
                    PIN_STEPPER_DIR2 = 1; 
                }
                if (nTemp > gunDeadBandThres)      // There is a deadband of 4 units.
                {
                   // Compensation for non-linear relationship between angular 
                   // velocity and pulse period. Based J. Brokking codes for 
                   // Arduino.                    
                   PIN_A4988_ENABLE = _ENABLE_A4988;
                   nTemp2 = 5500/(nTemp+9);
                   //nTemp2 = 405 - nTemp2;                  
                   OC8R = _STEPPER_MOTOR_PULSE_WIDTH*_STEPPER_MOTOR_COUNT;      // Set OC8 output pin to high.  The width of the 
                   //OC8RS = (405-nTemp2)*_STEPPER_MOTOR_COUNT;                   // pulse is determined by the value
                   OC8RS = nTemp2*_STEPPER_MOTOR_COUNT;                         // _STEPPER_MOTOR_PULSE_WIDTH*_STEPPER_MOTOR_COUNT
                                                                                // while the value in OC8RS sets the period. We must
                                                                                // make sure OC8RS >= OC8R at all times!    
                }
                else
                {
                    PIN_A4988_ENABLE = _DISABLE_A4988;                          // Cut-off the power to stepper motor when
                                                                                // not turning.  This depends on application,
                                                                                // it means there will be no holding torque
                                                                                // when the motor is not turning.    
                    OC8R = 0;                                                   // By setting OC8R = OC8RS, output pin of OC8
                    OC8RS = 0;                                                  // will be driven to low when not active.
                }                                                                
                                                                                
                // Note: 30 Aug 2017, the above codes to set the Output Compare unit needs to execute first
                // before we update the distance ticks.
                if ((nLeftOutHigh == 0) && (PIN_STEPPER_STEP1 == 1)) // Check for low-to-high transition on left motor output.
                {
                    if (PIN_STEPPER_DIR1 == 0)          // Check for forward direction.
                    {
                        gnDistanceMoveLW++;
                    }
                    else
                    {
                        gnDistanceMoveLW--;
                    }
                    nLeftOutHigh = 1;                   // Indicate that low-to-high transition has already occur.
                }
                else if ((nLeftOutHigh == 1) && (PIN_STEPPER_STEP1 == 0))
                {
                    nLeftOutHigh = 0;
                }                
                if ((nRightOutHigh == 0) && (PIN_STEPPER_STEP2 == 1)) // Check for low-to-high transition on right motor output.
                {
                    if (PIN_STEPPER_DIR2 == 1) 
                    {
                        gnDistanceMoveRW++;
                    }
                    else
                    {
                        gnDistanceMoveRW--;
                    }
                    nRightOutHigh = 1;                   // Indicate that low-to-high transition has already occur.
                }
                else if ((nRightOutHigh == 1) && (PIN_STEPPER_STEP2 == 0))
                {
                    nRightOutHigh = 0;
                }      
                                  
                OSSetTaskContext(ptrTask, 1, 1);                                // Next state = 1, timer = 1.
                break;
                
            case 10: // State 10 - Stop the motor.
                OC7CON1bits.OCM = 0b000;
                OC8CON1bits.OCM = 0b000;
                OSSetTaskContext(ptrTask, 1, 1);       // Next state = 1, timer = 1.
                break;
                
            default:
                OSSetTaskContext(ptrTask, 0, 1); // Back to state = 0, timer = 1.
            break;
        }
    }
}

