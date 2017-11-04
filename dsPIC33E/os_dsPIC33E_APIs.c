///////////////////////////////////////////////////////////////////////////////////////////////////
//
//  APPLICATION PROGRAM INTERFACE ROUTINES FOR dsPIC33EXXXX MICROCONTROLLER
//
//  (c) Copyright 2017, Fabian Kung Wai Lee, Selangor, MALAYSIA
//  All Rights Reserved  
//   
///////////////////////////////////////////////////////////////////////////////////////////////////
//
// Filename         : os_dsPIC33E_APIs.c
// Author           : Fabian Kung
// Last modified	: 4 April 2016
// Version          : 1.01
// Description		: This file contains the implementation of all the important routines
//                    used by the OS and the user routines. Most of the routines deal with
//                    microcontroller specifics resources, thus the functions have to be
//                    rewritten for different microcontroller family. Only the function
//                    prototype (call convention) of the routine is to be maintained.  In this
//                    way the OS can be ported to different microcontroller family.
// Toolsuites		: Microchip MPLAB X IDE v3.10 or above
//                	  MPLAB XC-16 C-Compiler v1.25 or above
// Microcontroller	: dsPIC33E families.

// Include common header to all drivers and sources.  Here absolute path is used.
// To edit if one changes folder
#include "osmain.h"

// --- GLOBAL AND EXTERNAL VARIABLES DECLARATION ---


// --- FUNCTIONS' PROTOTYPES ---
void ClearWatchDog(void);
void dsPIC33E_PORInit(void);

// --- REGISTER FOR STORING PROCESSOR'S INTERRUPT CONTEXT ---
unsigned char bytIEC0bak;
unsigned char bytIEC1bak;
unsigned char bytIEC2bak;

// --- FUNCTIONS' BODY ---

//////////////////////////////////////////////////////////////////////////////////////////////
//  BEGINNING OF CODES SPECIFIC TO dsPIC33EXXXX MICROCONTROLLER	//////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////


// Configuration bits setting for the microcontroller in the header file "p33EP256MU806.h".

_FOSCSEL(FNOSC_PRIPLL); 		// Primary oscillator, XT mode with phase-locked loop.
_FOSC(FCKSM_CSDCMD & IOL1WAY_OFF & OSCIOFNC_OFF & POSCMD_XT);
							   // Disable Fail-safe Clock Monitor and Clock-switching.
							   // Use XT crystal oscillator with internal phase-locked loop (PLL)
							   // OSC2 pin as oscillator output.
							   // Note: the actual clock frequency is determined by setting the 
							   // feedback divider M, pre-divider N1 and post-divider N2 of the 
							   // PLL system in the Power-On Reset initialization routines.
							   // On power-up the processor will runs at the oscillator frequency,
							   // which is 4MHz in this case.
//_FWDT(FWDTEN_OFF);                                // Disable Watch-dog Timer.
_FWDT(FWDTEN_ON);                                   // Enable Watch-dog Timer.
_FPOR(FPWRT_PWR2 & BOREN_ON);                       // Power-on Reset timer set to 2msec, brown-out reset enabled..
_FGS(GSS_ON & GWRP_OFF & GSSK_ON);                  // Code protection enabled, standard, no write-protect.
                                                    // Note: whenever either GSS or GWRP is ON, the GSSK must be ON to
                                                    // ensure proper operation.



// Interrupt Service Routine 
// Author		: Fabian Kung
// Last modified	: 12 Dec 2011
// Purpose		: This routine performs the following important tasks pertaining to the 
//                RTOS:
//                1. Updates the 32-bits global clock tick gulClockTick of the OS when Timer 1 overflows.
//                2. Increments the timer property of each tasks.
//                3. Check for task overflow condition (i.e. the time used to run the tasks is
//                greater the the clock tick.  When this happen the main scheduler will be
//                stalled, with the indicator LED turned on all the time. This option can be
//                disabled by remarking the relevant codes in the routine.
//
// Arguments		: None
// Return		: None
void __attribute__((__interrupt__)) _T1Interrupt( void )
{
    int ni = 0;

    if (IFS0bits.T1IF)                                                          // If it is Timer 1 overflow interrupt
    { 
		OSEnterCritical();

		if (gnRunTask == 1)						// If task overflow occur trap the controller 
		{										// indefinitely and turn on indicator LED1.
			while (1)
			{
				ClearWatchDog();				// Clear the Watch Dog Timer.
				PIN_OSPROCE1 = 1; 				// Turn on indicator LED1.
				PIN_ILED2 = 1;					// Turn on indicator LED2.
			}
		}

		gnRunTask = 1;							// Assert gnRunTask.
		gulClockTick++; 						// Increment RTOS clock tick counter. 	
		for (ni = 0;ni < gnTaskCount;ni++)		// Using for-loop produce more efficient 
												// assembly codes.
		{	
			if (gstrcTaskContext[ni].nTimer > 0) // Only decrement timer if it is greater than zero.
			{
				--(gstrcTaskContext[ni].nTimer); // Decrement timer for each process.
			}
		}
		
		PR1 = __TIMER1COUNT;					// Load Period Register.
 		IFS0bits.T1IF = 0;					// reset Timer 1 interrupt flag.
		OSExitCritical(); 	
    }
}     

// Interrupt Service Routine 
// Author		: Fabian Kung
// Last modified	: 29 March 2012
// Purpose		: This routine handles math routines error.
// Arguments		: None
// Return		: None
void __attribute__((__interrupt__)) _MathError( void )
{
    // Basically we stalled the processor when math erorr occurs.
    while (1)
    {
        ClearWatchDog();						// Clear the Watch Dog Timer.
    //    PIN_OSPROCE1 = 1; 						// Turn on indicator LED1.
    //    PIN_ILED2 = 1;							// Turn on indicator LED2.
        PIN_OSPROCE1 = 0; 						// Turn off indicator LED1.
        PIN_ILED2 = 0;							// Turn off indicator LED2.
    }
}

// Function name	: ClearWatchDog 
// Author           : Fabian Kung
// Last modified	: 24 April 2007
// Purpose          : Reset the Watch Dog Timer
// Arguments		: None
// Return           : None
void ClearWatchDog(void)
{
	asm ("clrwdt");	// Inline assembly instruction to clear the Watch Dog Timer.
}

// Function Name	: dsPIC33E_PORInit
// Author           : Fabian Kung
// Last modified	: 24 Feb 2017
// Description		: Perform power-on reset (POR) initialization on the microcontroller.
//                    Upon completion of this routine, all the microcontroller peripherals and
//                    I/O ports will be set to known state. For I/O ports, all pins will
//                    be set to
//                    (a) Digital mode,
//                    (b) Output and
//                    (c) A logic '0'.
// Arguments		: None
// Return           : None
// 
void dsPIC33E_PORInit()
{

	// Sets the internal phase-locked loop (PLL) for clock generation.
	// Oscillator frequency = Fin = 4MHz.
	// Fclk = Fin (M/(N1XN2).  
	// M = PLL feedback divider = PLLDIV + 2
	// N1 = Pre-divider into PLL = PLLPRE + 2
	// N2 = Post-divider from voltage-controlled oscillator (VCO) of PLL = 2(PLLPOST + 1)
	// If we are setting Fclk to 120MHz for 60MIPS operation (2 clock cycle per instruction).
    // The following selections are used:
    // Option 1: M = 120, N1 = 2, N2 = 2 and Fin = 4 MHz, Fclk = 120 MHz.
    // or PLLDIV = 118 = b'01110110', PLLPRE = 0, PLLPOST = 0.
	// Option 2,3 and 4: For slightly higher clock speed (with slightly higher power dissipation)
    // M = 124, N1 = 2, N2 = 2 and Fin = 4 MHz, Fclk = 124 MHz.
    // or PLLDIV = 122, PLLPRE = 0, PLLPOST = 0.
    // M = 128, N1 = 2, N2 = 2 and Fin = 4 MHz, Fclk = 128 MHz.
    // or PLLDIV = 126, PLLPRE = 0, PLLPOST = 0.
    // M = 132, N1 = 2, N2 = 2 and Fin = 4 MHz, Fclk = 132 MHz.
    // or PLLDIV = 130, PLLPRE = 0, PLLPOST = 0.
    
    //PLLFBD = 0x0076; // Clock option 1, sets PLLDIV = 120. Option 1, Fclk = 120 MHz.
    //PLLFBD = 0x007A;    // Clock option 2, sets PLLDIV = 124. Fclk = 124 MHz.
    //PLLFBD = 126; // Clock option 3, sets PLLDIV = 128, Option 2, Fclk = 128 MHz
	//PLLFBD = 130; // Clock option 3, sets PLLDIV = 132, Option 2, Fclk = 132 MHz
    //PLLFBD = 134; // Clock option 3, sets PLLDIV = 136, Option 2, Fclk = 136 MHz
    PLLFBD = 138; // Clock option 3, sets PLLDIV = 140, Option 2, Fclk = 140 MHz
    
    CLKDIV = 0x0000; // Sets PLLPRE = 0 and PLLPOST = 0.
                     // Peripheral clock = processor clock.
                     // Internal fast RC oscillator post scaler = 1.
                     // Fcy divided by 1, e.g. processor clock = fosc/2.

	// Check for Watchdog Timer timeout.
//	if (RCONbits.WDTO == 1) // Check the flag WDTO in Reset Control Register (RCON).
//	{
//		while (1) 
//		{
//			ClearWatchDog(); // Clear Watchdog Timer.
//			LATBbits.PLED1 = 1; // Turn on indicator LED1.
//		}
//	}

	// I/O port setting:
	// Port B setting, all Port B pins are set to outputs by default to prevent floating inputs.
	LATB = 0x0000;
	TRISB = 0x0000;
    ANSELB = 0x0000;        // Set all pins of Port B to digital by default.  This is important as dsPIC33E
                            // sets all analog capable pins to analog mode on power up.
                            // Port B pins are mapped to ADC inputs.

    TRISBbits.TRISB0 = 1;   // Set RB0 to input.  This is the input from voltage reference IC.

	// Port C setting, all Port C pins are set to outputs by default to prevent floating inputs.
	LATC = 0x0000;	
	TRISC = 0x0000;

    // Port D setting, all Port D pins are set to outputs by default to prevent floating inputs.
	LATD = 0x0000;
	TRISD = 0x0000;

    // Port E setting, all Port E pins are set to digital mode, and outputs by
    // default to prevent floating inputs.
	LATE = 0x0000;
	TRISE = 0x0000;
    ANSELE = 0x0000;        // Set all pins to digital be default.  This is important as dsPIC33E
                            // sets all analog capable pins to analog mode on power up.

    // Port F setting, all Port F pins are set to outputs by default to prevent floating inputs.
	LATF = 0x0000;
	TRISF = 0x0000;

    // Port G setting, all Port g pins are set to outputs by default to prevent floating inputs.
	LATG = 0x0000;
	TRISG = 0x0000;
    ANSELG = 0x0000;        // Set all pins of Port G to digital by default.  Port G pins are
                                // mapped to Analog Comparator inputs.
	// Setup 16-bit Timer1:
	// Note - Timer1 will be driven by internal clock cycle.  It will increase up 
	// to the value set in PR1, then reset back to 0. An interrupt will be triggered
	// during this event, known as Timer1 overflow.  
	// Set Timer1 to default state first.
	T1CON = 0;
	// Prescaler = 1:1, valid value 0-3 (2 bits).
	T1CONbits.TCKPS = 0;    // Clock source from internal oscillator (peripheral clock).
	// Load Period Register.
	PR1 = __TIMER1COUNT;	
	// Reset Timer1 register.		
	TMR1 = 0; 
	// reset Timer 1 interrupt flag.
 	IFS0bits.T1IF = 0;
 	// set Timer1 interrupt priority level to 1 (highest priority).
	IPC0bits.T1IP = 1;
	// enable Timer 1 interrupt.
 	IEC0bits.T1IE = 1;
	// Turn on Timer1 and start counting.
	T1CONbits.TON = 1; // Turn on Timer1.
}

// Function name	: OSEnterCritical
// Author           : Fabian Kung
// Last modified	: 24 April 2007
// Description		: Disable all processor interrupts for important tasks
//                    involving Stacks, Program Counter other critical
//	                  processor registers.
void OSEnterCritical(void)
{
	bytIEC0bak = IEC0;	// Store current processor interrupt settings.
	bytIEC1bak = IEC1;
	bytIEC2bak = IEC2;
	IEC0 = 0x00;		// Disable all processor interrupts.
	IEC1 = 0x00;
	IEC2 = 0x00;
}											

// Function name	: OSExitCritical
// Author           : Fabian Kung
// Last modified	: 24 April 2007
// Description		: Enable all processor interrupts for important tasks.
void OSExitCritical(void)
{
    IEC0 = bytIEC0bak;	// Restore processor interrupt settings.
    IEC1 = bytIEC1bak;
    IEC2 = bytIEC2bak;
}											


// Function name	: OSProce1
// Author           : Fabian Kung
// Last modified	: 4 April 2016
// Description		: Blink an indicator LED1 to show that the microcontroller is 'alive'.
#define _LED1_ON_MS	500		// LED1 on period in msec, i.e. 500 msec.

void OSProce1(TASK_ATTRIBUTE *ptrTask)
{
    switch (ptrTask->nState)
    {
	case 0: // State 0 - On Indicator LED1
            PIN_OSPROCE1 = 1;                                           // Turn on indicator LED1.
            OSSetTaskContext(ptrTask, 1, _LED1_ON_MS*__NUM_SYSTEMTICK_MSEC);    // Next state = 1, timer = _LED_ON_MS.
	break;

	case 1: // State 1 - Off Indicator LED1
            PIN_OSPROCE1 = 0;                                           // Turn off indicator LED1.
            OSSetTaskContext(ptrTask, 0, _LED1_ON_MS*__NUM_SYSTEMTICK_MSEC);    // Next state = 0, timer = _LED_ON_MS
            break;

        default:
            OSSetTaskContext(ptrTask, 0, 0);                                    // Back to state = 0, timer = 0.
    }
}


//////////////////////////////////////////////////////////////////////////////////////////////
//  END OF CODES SPECIFIC TO dsPIC33EXXXX MICROCONTROLLER	   ///////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////
