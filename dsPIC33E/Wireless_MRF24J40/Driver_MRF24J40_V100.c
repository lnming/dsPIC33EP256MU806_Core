//////////////////////////////////////////////////////////////////////////////////////////////
//
//	USER DRIVER ROUTINES DECLARATION (PROCESSOR DEPENDENT)
//
//  (c) Copyright 2013, Fabian Kung Wai Lee, Selangor, MALAYSIA
//  All Rights Reserved  
//   
//////////////////////////////////////////////////////////////////////////////////////////////
//
// File			: Drivers_WirelessComm_V100.c
// Author(s)		: Fabian Kung
// Last modified	: 14 Nov 2014
// Toolsuites		: Microchip MPLAB-X IDE v2.10 or above
//                	  MPLAB XC-16 C-Compiler v1.21 or above

// Include common header to all drivers and sources.  Here absolute path is used.
// To edit if one change folder
#include "osmain.h"

// Declaration for Microchip Technologies MRF24J40MA RF module.
#include "WirelessModule_MRF24J40.h"

// NOTE: Public function prototypes are declared in the corresponding *.h file.


//
// --- PUBLIC VARIABLES ---
//

// Data buffer and address pointers for wireless serial communications (IEEE 802.15.4).
unsigned char gbytRFTXbuffer[__RFSCI_TXBUF_LENGTH-1];   // Transmit buffer.
unsigned char gbytRFTXbufptr;                           // Transmit buffer pointer.
unsigned char gbytRFTXbuflen;                           // Transmit buffer length.
unsigned char gbytRFRXbuffer[__RFSCI_RXBUF_LENGTH-1];   // Receive buffer length.
unsigned char gbytRFRXbufptr;                           // Receive buffer length pointer.
unsigned char gbytRFRXbuflen;                           // Receive buffer length.

//
// --- PRIVATE FUNCTION PROTOTYPES ---
//
// Functions related to MRF24J40MA RF Module Driver 
void SPI_SetRAMShortAdd(BYTE, BYTE );           // Private function, this file only.
void SPI_SetRAMLongAdd(UINT16, BYTE );          // Private function, this file only.
BYTE bytSPI_GetRAMShortAdd(BYTE);		// Private function, this file only.
BYTE bytSPI_GetRAMLongAdd(UINT16);		// Private function, this file only.
INT16 nIsRFModuleBusy(void);			// Public function, whole project (via extern keyword).
INT16 nIsRFModuleTXError(void);			// Public function, whole project (via extern keyword).
INT16 nIsValidDataRFModule(void);		// Public function, whole project (via extern keyword).
void SendDataRFModule(BYTE *, BYTE);            // Public function, whole project (via extern keyword).

//
// --- PRIVATE VARIABLES ---
//

// MRF24J40MA device default device address
BYTE gbytRFAddL = 0x01;						// Public data, whole project (via extern keyword).
//BYTE gbytRFAddL = 0x02;
BYTE gbytRFAddH = 0x00;						// Public data, whole project (via extern keyword).
BYTE gbytRFDesAddL = 0x00;					// Public data, whole project (via extern keyword).
BYTE gbytRFDesAddH = 0x00;					// Public data, whole project (via extern keyword).


//
// --- Microcontroller Pins --- 
//
// Pin Name                         uC Pin                External Module Pin Function             uC Pin Function
// --------                         ------                ----------------------------             --------------
#define PIN_MRF24J40_Driver_RESET	_RF0	// Pin RF0 = &Reset&, output.
#define PIN_MRF24J40_Driver_SCK		_RG6	// Pin RG6 = SCK, output.                           Note: uC pin set to SCK, output.
#define	PIN_MRF24J40_Driver_CS		_RG9	// Pin RG9 = &CS&, output.                          Note: uC pin set to CS, output.
#define PIN_MRF24J40_Driver_SDI		_RG8	// Pin RG8 = SDI, output.                           Note: uC pin set to SDO, output.
#define PIN_MRF24J40_Driver_SDO		_RG7	// Pin RG7 = SDO, input.                            Note: uC pin set to SDI, input.
#define	PIN_MRF24J40_Driver_INT		_RF1	// Pin RF1 = INT, input.                            Note: INT is MRF24J40 interrupt output.
#define	PIN_MRF24J40_Driver_LED2	PIN_ILED2	// Indicator LED2 output.

//
// --- Process Level Constants Definition --- 
//
#define	_RFMODULE_RSTDLY_US	240	// Reset period, in microseconds.
#define _RFMODULE_PANIDL	0x01	// PAN (Personal Area Network) ID, low byte.
#define _RFMODULE_PANIDH	0x00	// PAN ID, high byte.
#define	_MRF24J40_BUSY          1	// MRF module status.
#define	_MRF24J40_IDLE          0
#define _MRF24J40_POWER_DOWN    -1      

///
/// Process name	: Proce_MRF24J40_Driver
///
/// Author		: Fabian Kung
///
/// Last modified	: 14 Nov 2014
///
/// Code version	: 1.00
///
/// Processor		: dsPIC33EP256MU80X family.
///
/// Processor/System Resources 
/// PINS		: See above.
///
/// MODULES		: 1. SPI2 (Internal).
///
/// RTOS		: Ver 1 or above, round-robin scheduling.
///
/// Global variables: gbytRFRXbuffer[]
///                   gbytRFRXbufptr
///                   gbytRFRXbuflen
///                   gbytRFTXbuffer[]
///                   gbytRFTXbufptr
///		      gbytTXbuflen
///                   gSCIstatus
///
/// Description     : Driver for Microchip Technologies MRF24J40MA RF Transceiver Module.
///                   This modules performs a number of functions:
///                    1. Set up the built-in SPI module of the microcontroller (SPI2) for
///                    serial communication with the RF Transeiver.  The microcontroller assumes the
///                    role of master.
///                    2. Initialize the device address and PAN ID for MRF24J40MA.
///                    3. Initialize the MRF24J40MA RF Transeiver Module.
///                    4. Send a packet of data in iEEE 802.15.4 format.
///                    5. Receive a packet of data in IEEE 802.15.4 format.
///
///
/// Example of usage	: The codes example below illustrates how to send 2 bytes of character,
///		          'a' and 'b' via RF Module.
///	if (gSCIstatus.bRFTXRDY == 0)	// Check if RF module is busy.
///	{
///         gSCIstatus.bRFTXRDY = 1; 	// Set transmitt flag.
///         gbytRFTXbuflen = 2;         // Indicate data frame length.
///         gbytRFTXbuffer[0] = 'a'; 	// Load data to be send.
///         gbytRFTXbuffer[1] = 'b'; 	// Load data to be send.
///	}
///
///  The flag bRFTXRDY will be automatically cleared upon completion.  Sometimes the
///  transmission is not successful, due to error such as channel is congested, and
///  the number of retries exceeds the limit.  To check for transmission status, we
///  monitor the bRFTXRDY flag and bRFTXERR flag.
///
///	if (gSCIstatus.bRFTXRDY == 0)  //	
///	{
///         if (gSCIstatus.bRFTXERR == 1) // Indicate error in transmission.
///         {
///		gSCIstatus.bRFTXERR = 0;	// Clear error flag.
///		... Do some actions ...
///         }
///	}
///
/// Example of usage	: The codes example below illustrates how to retrieve 1 byte of data from
///                    the RF Module receive buffer.
///	if (gSCIstatus.bRFRXRDY == 1)
///	{
///         if (gSCIstatus.bRFTXRDY == 0)	// Check if RF module is busy.
///         {
///              gbytData = gbytRFRXbuffer[3]; 	// Load data to be send.
///						// Note: Here we assume the 3rd byte contains
///						// the data.  0 byte is the frame length.
///						// 1st and 2nd bytes are frame header.
///          }
///          gbytRFRXbufptr = 0x00;		// Clear RF module receiver buffer pointer.
///          gSCIstatus.bRFRXRDY = 0; 		// Deassert RF module receive flag.
///	}
///

void Proce_MRF24J40_Driver(TASK_ATTRIBUTE *ptrTask)
{
 static BYTE bytData, bytLQI, bytRSSI;
 static BYTE bytFrameLenRX;
 static UINT16 nRXFIFO_Address, nTXFIFO_Address;
 static UINT16 nMRF24J40State = _MRF24J40_IDLE;
 INT16 nTemp;

    if (ptrTask->nTimer == 0)
    {
	switch (ptrTask->nState)
	{
            case 0: // State 0 - SPI2 Initialization and MRF24J40 global reset.
                    // Setup IO pins mode.
                TRISGbits.TRISG7 = 1;		// Set RF1 and RG7 to inputs.
		TRISFbits.TRISF1 = 1;
                // Note: for dsPIC33EP256MU806 controller, internal module SPI2 pins is automtaically
                // mapped to RG6 (SCK2), RG7 (SDI2) and RG8 (SDO2).  These pins are connected directly
                // to the RF module.  There is not need to configure the remappable IO registers RPORXX
                // and RPINRXX as in dsPIC33F series.

                
                SPI2CON1bits.MSTEN = 1;         // Set to master mode.  This bit needs to be set first
                                                // before setting SMP bit (see datasheet of dsPIC33EP256MU806).
 		//SPI2CON1 = 0x033A;		// SPI Master, 8 bits operation (shift out 8 bits at a time).
                                                // CKE=1, CKP=0, i.e. clock is active high, positive going edge.
						// SSEN=0, SS1 pin is not used.
						// Primary prescalar = 4:1, secondary prescalar = 2:1
                                                // Assuming CPU clock frequency of 60MHz, this will give
                                                // 60/(4x2) = 7.5MHz SPI clock.
						// NOTE:
                                                // Each SPI write and read contains 3 bytes max, thus 24 clock pulses
						// are needed.  This translates into roughly max 5usec per SPI
						// transaction.
                SPI2CON1 = 0x0336;            // Same as above except Primary prescalar = 4:1, secondary = 4:1
                                                // For 60MHz CPU clock this will give 60/16 = 3.75MHz.
		SPI2STAT = 0xA000;		// Enable SPI2 module.
		nTemp = SPI2BUF;		// Read the received buffer, to clear SPIRBF flag.
		PIN_MRF24J40_Driver_CS = 1;	// Manually deassert SS pin.
		PIN_MRF24J40_Driver_RESET = 1;	// Manually deassert Reset pin.

		gSCIstatus.bRFRXRDY = 0;	// Clear all flags in status register.
		gSCIstatus.bRFTXRDY = 0;
		gSCIstatus.bRFTXERR = 0;	
		gbytRFTXbuflen = 0;		// Clear RF TX buffer length register.
		gbytRFTXbufptr = 0;		// Reset RF TX buffer pointer. 
		gbytRFRXbufptr = 0;		// Reset RF RX buffer pointer.
                OSSetTaskContext(ptrTask, 1, 1000); // Next state = 1, timer = 1000.
            break;
			
            case 1: // State 1 - Global reset and long delay for chip to stabilize.
		PIN_MRF24J40_Driver_RESET = 0;      // Reset MRF24J40 chip via external Reset pin.
		OSSetTaskContext(ptrTask, 2, 5000); // Next state = 2, timer = 5000.
            break;

            case 2: // State 2 - Global reset and delay with chip to stabilize.
		PIN_MRF24J40_Driver_RESET = 1;                      // Deasert MRF24J40 external Reset pin.
		OSSetTaskContext(ptrTask, 3, 4000/__SYSTEMTICK_US); // Next state = 3, timer = requires at least 2msec.
		//OSSetTaskContext(ptrTask, 3, 100); 	// Next state = 3, timer = 100 (requires at least 2msec).
            break;

            case 3: // State 3 - Software reset power management, baseband and MAC circuitry.
		SPI_SetRAMShortAdd(SOFTRST, 0x07); 	// Perform a software reset.
		OSSetTaskContext(ptrTask, 4, 1); 	// Next state = 4, timer = 1.
                
            break;

            case 4: // State 4 - Check if software reset done.
                if ((bytSPI_GetRAMShortAdd(SOFTRST)&0x07) != 0x00)
                {
                    OSSetTaskContext(ptrTask, 3, 1); // Next state = 3, timer = 1.
                }
                else
                {
                    OSSetTaskContext(ptrTask, 5, 1); // Next state = 5, timer = 1.
                }
            break;

            case 5: // State 5 - Physical layer initialization.
		// Temp instructions (because I modified the MRF24J40MA module, floating these pins)
		// This instruction is not needed for normal MRF24J40MA module.
		// SPI_SetRAMShortAdd(0x34, 0x0C); // Set GPIO direction, GPIO2 & 3 are outputs.
		// End of temp instructions

		SPI_SetRAMShortAdd(PACON2, 0x98); // Enable FIFO.
		OSSetTaskContext(ptrTask, 6, 2); // Next state = 6, timer = 2.
            break;

            case 6: // State 6 - Physical layer initialization.
		SPI_SetRAMShortAdd(TXSTBL, 0x95); // Set VCO stabilization period and minimum short inter-frame
												  // spacing bits. 
		OSSetTaskContext(ptrTask, 7, 1); // Next state = 7, timer = 1.
            break;

            case 7: // State 7 - Physical layer initialization.
		SPI_SetRAMLongAdd(RFCON1, 0x01); // Set VCO optimization, as per recommendation from datasheet.
		OSSetTaskContext(ptrTask, 8, 1); // Next state = 8, timer = 1.
            break;

            case 8: // State 8 - Physical layer initialization.
		SPI_SetRAMLongAdd(RFCON2, 0x80); // Enable phase-locked loop (PLL).
		OSSetTaskContext(ptrTask, 9, 1); // Next state = 9, timer = 1.
            break;

            case 9: // State 9 - Physical layer initialization.
		SPI_SetRAMLongAdd(RFCON6, 0x90); //	Set TX filter and 20MHz clock recovery bits, as per datasheet.
		OSSetTaskContext(ptrTask, 10, 1); // Next state = 10, timer = 1.
            break;

            case 10: // State 10 - Physical layer initialization.
		SPI_SetRAMLongAdd(RFCON7, 0x80); // Select 100kHz internal oscillator for sleep clock.
		OSSetTaskContext(ptrTask, 11, 1); // Next state = 11, timer = 1.
            break;

            case 11: // State 11 - Physical layer initialization.
                SPI_SetRAMLongAdd(RFCON8, 0x10); // Set RF VCO control, as per datasheet.
                OSSetTaskContext(ptrTask, 12, 1); // Next state = 12, timer = 1.
            break;

            case 12: // State 12 - Physical layer initialization.
                SPI_SetRAMLongAdd(SLPCON1, 0x21); // Disable CLKOUT pin and set sleep clock divisor, as per
	  				      // datasheet.
                OSSetTaskContext(ptrTask, 13, 1); // Next state = 13, timer = 1.
            break;

            case 13: // State 13 - Physical layer initialization.
		SPI_SetRAMShortAdd(BBREG2, 0x80); // Set CCA mode to ED. 
		OSSetTaskContext(ptrTask, 14, 1); // Next state = 14, timer = 1.
            break;

            case 14: // State 14 - Physical layer initialization.
		SPI_SetRAMShortAdd(RSSITHCCA, 0x60); // Set CCA ED threshold.
		OSSetTaskContext(ptrTask, 15, 1); // Next state = 15, timer = 1.
            break;

            case 15: // State 15 - Physical layer initialization.
		SPI_SetRAMShortAdd(BBREG6, 0x40); // Set appended RSSI value to RXFIFO.
		OSSetTaskContext(ptrTask, 16, 1); // Next state = 16, timer = 1.
            break;

            case 16: // State 16 - Set interrupt.
                    // The default edge for INT pin is high-to-low when INT16errupt is generated.  It is
                    // controlled by INTEDGE bit in SLPCON0 (0x211) register.
                    //SPI_SetRAMShortAdd(INTCON, 0x00); // Enable all interrupts.
		SPI_SetRAMShortAdd(INTCON, 0xF6); // TXNIE (TX normal FIFO transmission interrupt) = enabled.
						  // RXIE (RX FIFO reception interrupt) = enabled.
		OSSetTaskContext(ptrTask, 17, 1); // Next state = 17, timer = 1.
            break;

            case 17: // State 17 - Set channel.
                SPI_SetRAMLongAdd(RFCON0, 0x02); // Select Channel 11, as per datasheet.
                OSSetTaskContext(ptrTask, 18, 1); // Next state = 18, timer = 1.
            break;

            // --- Custom initialization steps added by F. Kung ---

            // --- Set Device Address (16 bits) and PAN ID (16 bits) ---
            case 18: // State 18 - Set Short Device Address low byte.
		SPI_SetRAMShortAdd(SADRL, gbytRFAddL); // Low byte.
		OSSetTaskContext(ptrTask, 19, 1); // Next state = 19, timer = 1.
            break;

            case 19: // State 19 - Set Short Device Address high byte.
		SPI_SetRAMShortAdd(SADRH, gbytRFAddH); // High byte.
		OSSetTaskContext(ptrTask, 20, 1); // Next state = 20, timer = 1.
            break;

            case 20: // State 20 - Set PAN ID low byte.
		SPI_SetRAMShortAdd(PANIDL, _RFMODULE_PANIDL); // Low byte.
		OSSetTaskContext(ptrTask, 21, 1); // Next state = 21, timer = 1.
            break;

            case 21: // State 21 - Set PAN ID high byte.
		SPI_SetRAMShortAdd(PANIDH, _RFMODULE_PANIDH); // High byte.
		OSSetTaskContext(ptrTask, 22, 1); // Next state = 22, timer = 1.
                //OSSetTaskContext(ptrTask, 200, 1); // Next state = 200, timer = 1.
            break;

            // --- TX module setting ---
            case 22: // State 22 - Set TX power level.
		SPI_SetRAMLongAdd(RFCON3, 0x00); // Set power level to 0dBm (into 50 ohms load).
		OSSetTaskContext(ptrTask, 23, 1); // Next state = 23, timer = 1.
            break;

            case 23: // State 23 - Set receive MAC control register.
		SPI_SetRAMShortAdd(RXMCR, 0x00);  // 1. Receive mode = Normal (Only packets with good CRC, matched 
						  //	address and PAN ID are accepted).
		//SPI_SetRAMShortAdd(RXMCR, 0x01);  // 1. Receive mode = Promiscuous (all packets with good CRC are 
						  // accepted).	
						  // 2. Enable automatic Acknowledgment response.
						  // 3. Device as PAN coordinator = no.
						  // 4. Device as Coordinator = no.
												   
		OSSetTaskContext(ptrTask, 24, 1); // Next state = 24, timer = 1.
            break;
			
            // --- RX module setting ---
            case 24: // State 24 - Set receive frame format filter.
                SPI_SetRAMShortAdd(RXFLUSH, 0x01); // 1. Receive all valid data frames and put in RX FIFO.
                                                   // 2. Automatically reset RX FIFO address pointer to zero.
                                                   // 3. Disable Wake I/O pin.
                                                   // Also manually set bit0 to clear the internal RX FIFO address
                                                   // pointter to zero.
                 OSSetTaskContext(ptrTask, 25, 1);  // Next state = 25, timer = 1.
            break;

            case 25: // State 25 - Enable receiver.
                SPI_SetRAMShortAdd(BBREG1, 0x00);   // Clear the RXDECINV bit (bit2), enable receiving packets.
                                                    // This bit is set upon receiving a packet, to prevent incoming
                                                    // data from corrupting the received data.  It is only cleared
                                                    // when the old data is retrieved from the RX FIFO.
		PIN_MRF24J40_Driver_LED2 = 0;	  // Off indicator LED2.
		OSSetTaskContext(ptrTask, 26, 1); // Next state = 26, timer = 1.
            break;

            // --- End of custom initialization steps added by F. Kung ---

            case 26: // State 26 - Reset RF state machine.
		SPI_SetRAMShortAdd(RFCTL, 0x04); // Reset RF state machine by toggling RFRST bit.
		OSSetTaskContext(ptrTask, 27, 1); // Next state = 27, timer = 1.
            break;

            case 27: // State 27 - Reset RF state machine.
		SPI_SetRAMShortAdd(RFCTL, 0x00); //
		gSCIstatus.bRFRESET = 0;		 // Deassert reset flag.
		nMRF24J40State = _MRF24J40_IDLE;		 // Indicate RF module is idle.
		OSSetTaskContext(ptrTask, 28, (_RFMODULE_RSTDLY_US/__SYSTEMTICK_US)+1); // Next state = 28, timer = 2 (delay to allow internal calibration).
            break;

            case 28: // State 28 - Clear all interrupt flags.
		bytData = bytSPI_GetRAMShortAdd(INTSTAT); // Read the interrupt status register, to clear all flags.
                OSSetTaskContext(ptrTask, 30, 200); // Next state = 30, timer = 200.
            break;

            case 30: // State 30 - Check for pending data to transmitt or receive.
		bytData = bytSPI_GetRAMShortAdd(INTSTAT); // Read the interrupt status register.
		if ((bytData & 0x01)!=0) // Check bit0, TXNIF flag (End if transmission for Normal Data Frame).
		{	// Data transmitted.
                    gbytRFTXbufptr = 0;				// Reset RF TX buffer pointer.
                    gbytRFTXbuflen = 0;				// Reset RF TX buffer length.
                    gSCIstatus.bRFTXRDY = 0;		// Clear flag.
                    if ((bytSPI_GetRAMShortAdd(TXSTAT) & 0x01) == 0) // Check for transmitt error.
                    {
			gSCIstatus.bRFTXERR = 1; 	// Set RF TX error flag.
                    }
                    nMRF24J40State = _MRF24J40_IDLE;	 	// Indicate RF module is idle.
                    PIN_MRF24J40_Driver_LED2 = 0;			// Off indicator LED2.
		}

		if ((bytData & 0x08)!=0) // Check bit3, RXIF flag.
		{	// Received one data packet.
                    SPI_SetRAMShortAdd(BBREG1, 0x04);  // Set the RXDECINV bit (bit2), disable receiving packets.
                    OSSetTaskContext(ptrTask, 100, 1); // Next state = 100, timer = 1.
		}
		else if (gSCIstatus.bRFTXRDY == 1) // Check if valid data to be transmitted via RF transceiver.
		{	
                    if (nMRF24J40State != _MRF24J40_BUSY)
                    {	// No on-going transmitt activity.
			OSSetTaskContext(ptrTask, 31, 1); // Next state = 31, timer = 1.
                    }
                    else
                    {	// On-going transmitt activity.
			OSSetTaskContext(ptrTask, 30, 1); // Next state = 30, timer = 1.
                    }
		}
                else
		{
			OSSetTaskContext(ptrTask, 30, 1); // Next state = 30, timer = 1.
		}
					
		if (gSCIstatus.bRFRESET == 1)  // Check if reset RF module request.
		{	// Reset the RF module, override all previous settings.
			OSSetTaskContext(ptrTask, 1, 1); // Next state = 1, timer = 1.
		}
		break; 	

		// --- Transmission state machine for DATA packet ---
		case 31: // State 31 - Load Data frame (Load packet into FIFO), Part 1.
                         // According to IEEE 802.15.4 2006 standard.
			 // Only DATA packet according to the above is used.
			 // Use TX Normal FIFO (Long RAM address 0x0000 to 0x007F).
			 // FIFO Address:	Content:		Remarks
                        // 0x0000			9				Header length
			// 0x0001			Variable		Frame length (Header + payload)
			// 0x0002			0x41			Frame Control Field(FCF) 1: Indicate Data Frame.
			// 0x0003			0x88			Frame Control Field 2: Default.
			// 0x0004			0			Frame sequence
			// 0x0005			Variable		Destination PAN ID, low byte.
			// 0x0006			Variable		Destination PAN ID, high byte.
			// 0x0007			gbytRFDesAddL	Destination Address, low byte.
			// 0x0008			gbytRFDesAddH 	Destination Address, high byte.
			// 0x0009			gbytRFAddL		Source Address, low byte.
			// 0x000A			gbytRFAddH		Source Address, high byte.
			// 0x000B			Start of payload.

                    PIN_MRF24J40_Driver_LED2 = 1; 		// lights up Indicator LED2.
                    nMRF24J40State = _MRF24J40_BUSY;            // Indicate RF module is busy.

                    if (gbytRFTXbuflen > __RFSCI_TXBUF_LENGTH)	// Limit the length if necessary.
                    {
			gbytRFTXbuflen = __RFSCI_TXBUF_LENGTH;
                    }
                    nTXFIFO_Address = 0x0000; 			// Reset TX FIFO address pointer.
                    SPI_SetRAMLongAdd(nTXFIFO_Address, 0x09);	// Load header length.
                    nTXFIFO_Address++; 				// Next location.

                    bytData = gbytRFTXbuflen + 9;	// Calculate frame length.
							// Frame length = Header length + payload length.
                    // Begin load TX Normal FIFO procedures.
                    SPI_SetRAMLongAdd(nTXFIFO_Address, bytData);	// Load Frame length.
                    OSSetTaskContext(ptrTask, 32, 1); 	// Next state = 32, timer = 1.
		break;

		case 32: // State 32 - Load header (Load packet into FIFO), Part 2.
                    nTXFIFO_Address++;
                    SPI_SetRAMLongAdd(nTXFIFO_Address, 0x41);	// Load Frame Check Format (FCF) 1.
                    nTXFIFO_Address++;
                    SPI_SetRAMLongAdd(nTXFIFO_Address, 0x88);	// Load Frame Check Format (FCF) 2.
                    OSSetTaskContext(ptrTask, 33, 1); // Next state = 33, timer = 1.
		break;

		case 33: // State 33 - Load header (Load packet into FIFO), Part 3.
                    nTXFIFO_Address++;
                    SPI_SetRAMLongAdd(nTXFIFO_Address, 0x00);	// Load Frame Sequence Number (set to 1 default).
                    OSSetTaskContext(ptrTask, 34, 1); // Next state = 34, timer = 1.
                break;

		case 34: // State 34 - Load header (Load packet into FIFO), Part 4.
                    nTXFIFO_Address++;
                    SPI_SetRAMLongAdd(nTXFIFO_Address, _RFMODULE_PANIDL);	// Load PAN ID, low byte.
                    nTXFIFO_Address++;
                    SPI_SetRAMLongAdd(nTXFIFO_Address, _RFMODULE_PANIDH);	// Load PAN ID, high byte.
                    OSSetTaskContext(ptrTask, 35, 1); // Next state = 35, timer = 1.
		break;

		case 35: // State 35 - Load header (Load packet into FIFO), Part 5.
                    nTXFIFO_Address++;
                    SPI_SetRAMLongAdd(nTXFIFO_Address, gbytRFDesAddL);	// Load Destination Address, low byte.
                    nTXFIFO_Address++;
                    SPI_SetRAMLongAdd(nTXFIFO_Address, gbytRFDesAddH);	// Load Destination Address, high byte.
                    OSSetTaskContext(ptrTask, 36, 1); // Next state = 36, timer = 1.
		break;

		case 36: // State 36 - Load header (Load packet into FIFO), Part 6.
                    nTXFIFO_Address++;
                    SPI_SetRAMLongAdd(nTXFIFO_Address, gbytRFAddL);	// Load Device Address, low byte.
                    nTXFIFO_Address++;
                    SPI_SetRAMLongAdd(nTXFIFO_Address, gbytRFAddH);	// Load Device Address, high byte.
                    OSSetTaskContext(ptrTask, 37, 1); // Next state = 37, timer = 1.
		break;

		case 37: // State 37 - Load data/payload (Load packet into FIFO).
                    nTXFIFO_Address++;
                    SPI_SetRAMLongAdd(nTXFIFO_Address, gbytRFTXbuffer[gbytRFTXbufptr]); // Load data from TX buffer for RF Comm.
                    gbytRFTXbufptr++;	// Pointer to next byte.
                    if (gbytRFTXbufptr < gbytRFTXbuflen)	// All bytes transmitted?
                    {	// No
			OSSetTaskContext(ptrTask, 37, 1); // Next state = 37, timer = 1.
                    }
                    else
                    {	// Yes
			OSSetTaskContext(ptrTask, 38, 1); // Next state = 38, timer = 1.
                    }
                 break;

		case 38:  // State 38 - Send normal data packet.
                    SPI_SetRAMShortAdd(TXNCON, 0x01); // Send packet, no acknowledgment and no encryption.
                    OSSetTaskContext(ptrTask, 30, 1); // Next state = 30, timer = 1.
		break;

			// --- Reception state machine for DATA packet ---
			case 100: // State 100 - Read RX FIFO.
				PIN_MRF24J40_Driver_LED2 = 1; 				  // lights up Indicator LED2.
				bytFrameLenRX = bytSPI_GetRAMLongAdd(0x0300); // Read 1st byte of RX FIFO (frame length).
				nRXFIFO_Address = 0x300; // Reset RX FIFO address pointer to the start of FIFO.
				if (bytFrameLenRX < 9+2) // Length of header is 9 bytes, with one LQI and RSSI byte each.
										 // thus anything less means no meaningful data.
				{
					OSSetTaskContext(ptrTask, 102, 1); // Next state = 102, timer = 1.
				}
				else
				{
					gbytRFRXbuflen = bytFrameLenRX - (9+2);	// Indicate the length of received data in bytes.
					OSSetTaskContext(ptrTask, 101, 1); // Next state = 101, timer = 1.
				}
			break;

			case 101: // State 101 - Transfer RX FIFO data into local RF receive buffer.
				nRXFIFO_Address++;			// Increment RX FIFO address pointer.
				bytFrameLenRX--;			// Decrement length counter.

				// Transfer one byte from RX FIFO to the on-chip receive buffer provided 
				// the receive buffer is not full.
				if (gbytRFRXbufptr < __RFSCI_RXBUF_LENGTH)
				{
					gbytRFRXbuffer[gbytRFRXbufptr] = bytSPI_GetRAMLongAdd(nRXFIFO_Address); 
					gbytRFRXbufptr++;		// Increment RF comm RX data pointer.
				}

				if (bytFrameLenRX == 0) 
				{	// All data read retrived.		
					OSSetTaskContext(ptrTask, 102, 1); // Next state = 102, timer = 1.
				}
				else
				{	// Still got data in RF FIFO.
					OSSetTaskContext(ptrTask, 101, 1); // Next state = 101, timer = 1.
				}
			break;
		
			case 102: // State 102 - Read LOI (link quality indicator) and RSSI (receive signal strength indicator) bytes.
				nRXFIFO_Address++;
				bytLQI = bytSPI_GetRAMLongAdd(nRXFIFO_Address); 
				nRXFIFO_Address++;
				bytRSSI = bytSPI_GetRAMLongAdd(nRXFIFO_Address);
				OSSetTaskContext(ptrTask, 103, 1); // Next state = 102, timer = 1.
			break;

			case 103: // State 103 - End receiving session.
				SPI_SetRAMShortAdd(RXFLUSH, 0x01); // 1. Receive all valid data frames and put in RX FIFO.
												   // 2. Automatically reset RX FIFO address pointer to zero.
												   // 3. Disable Wake I/O pin. 
												   // Also manually set bit0 to clear the internal RX FIFO address
												   // pointer to zero.
				SPI_SetRAMShortAdd(BBREG1, 0x00); // Clear the RXDECINV bit (bit2), enable receiving packets.
												  // This bit is set upon receiving a packet, to prevent incoming
												  // data from corrupting the received data.  It is only cleared 
												  // when the old data is retrieved from the RX FIFO.
				PIN_MRF24J40_Driver_LED2 = 0;	  // Off indicator LED2.
				gSCIstatus.bRFRXRDY = 1;	// Set flag to indicate valid data.
				OSSetTaskContext(ptrTask, 30, 1); // Next state = 30, timer = 1.
			break;

            case 200: // State 200 - Set the module to SLEEP mode (low power).
                OSSetTaskContext(ptrTask, 203, 1); // Next state = 203, timer = 1.
                break;

            case 203: // State 203 - Check the CPU state, wake up the RF module if CPU is backed in NORMAL mode.
                //if (gnProcessorState == __CPU_NORMAL)
                //{
                //    OSSetTaskContext(ptrTask, 203, 1); // Next state = 203, timer = 1.
                //}
                //else
                //{
                    OSSetTaskContext(ptrTask, 203, 1); // Next state = 203, timer = 1.
                //}
                break;

           case 204: // State 204 - Wake up the RF module from SLEEP mode.
                OSSetTaskContext(ptrTask, 206, 1); // Next state = 206, timer = 1.
                break;
                
           case 206: // State 206 - Resume operation.
                OSSetTaskContext(ptrTask, 30, 1); // Next state = 30, timer = 1.
                break;

            default:
		OSSetTaskContext(ptrTask, 0, 1); // Back to state = 0, timer = 1.
            break;
	}
    }
}

///
/// Function name	: SPI_SetRAMShortAdd
///
/// Author		: Fabian Kung
///
/// Last modified	: 10 Jan 2013
///
/// Code version	: 1.00
///
/// Processor		: dsPIC33EP256MU806
///
/// Description		: SPI driver for Microchip Technologies MRF24J40MA RF Transceiver Module.
///                       To set the registers in MRF24J40 chip using short address format.
///                       SPI2 is used for this.
///
/// Arguments		: bytAdd = Register address (only 6 bits is used).
///
///                       bytData = data to be written into register.
///
/// Return		: None.
///
/// Global variable	: None.
void SPI_SetRAMShortAdd(BYTE bytAdd, BYTE bytData)
{
	INT16 nReadData;

	PIN_MRF24J40_Driver_CS = 0;	// Manually assert SS pin. 
					// NOTE: must be careful here to ensure the THL for SS to
					// SCK output exceeds the minimum specs.
	bytAdd = (bytAdd<<1)&0x007F;	// Shift left 1 bit and set bit7=0, bit0=1.
	bytAdd = bytAdd|0x01;		// bit7=0 to indicate short address format, bit0=1 for write operation.
					// (See datasheet of MRF24J40 chip).
	SPI2BUF = bytAdd; 		// Shift out data via SPI. Note that data is shifted out via MSb.
	while (!SPI2STATbits.SPIRBF);	// Wait until all bits are shifted out, this is
					// equivalent to 16 bits being shifted in, triggering
					// the SPI receive buffer full flag, SPIRBF.
	nReadData = SPI2BUF;		// Read the received buffer, to clear SPIRBF flag
	SPI2BUF = bytData; 		// Shift out data via SPI. Note that data is shifted out via MSb.
					// Only first 8 bits are used.
	while (!SPI2STATbits.SPIRBF);	// Wait until all bits are shifted out
	nReadData = SPI2BUF;		// Read the received buffer, to clear SPIRBF flag.
	PIN_MRF24J40_Driver_CS = 1;	// Manually deassert SS pin.
}

///
/// Function name	: bytSPI_GetRAMShortAdd
///
/// Author		: Fabian Kung
///
/// Last modified	: 10 Jan 2013
///
/// Code version	: 1.04
///
/// Processor		: dsPIC33EP256MU806
///
/// Description		: SPI driver for Microchip Technologies MRF24J40MA RF Transceiver Module.
///                       To get the register's content in MRF24J40 chip using short address format.
///
/// Arguments		: bytAdd = Register address (only 6 bits is used).
///
/// Return		: bytData = data read from register.
///
/// Global variable	: None.
BYTE bytSPI_GetRAMShortAdd(BYTE bytAdd)
{
    INT16 nReadData;

    PIN_MRF24J40_Driver_CS = 0;     // Manually assert SS pin.
                                    // NOTE: must be careful here to ensure the THL for SS to
                                    // SCK output exceeds the minimum specs.
    bytAdd = (bytAdd<<1)&0x007E;    // Shift left 1 bit and set bit7=0, bit0=0.
                                    // bit7=0 to indicate short address format, bit0=0 for read operation.
                                    // (See datasheet of MRF24J40 chip).
    SPI2BUF = bytAdd;               // Shift out data via SPI. Note that data is shifted out via MSb.
    while (!SPI2STATbits.SPIRBF);   // Wait until all bits are shifted out, this is
                                    // equivalent to 16 bits being shifted in, triggering
                                    // the SPI receive buffer full flag, SPIRBF.
    nReadData = SPI2BUF;            // Read the received buffer, to clear SPIRBF flag
    SPI2BUF = 0x00;                 // Shift out dummy data via SPI. Note that data is shifted out via MSb.
                                    // Only first 8 bits are used.
    while (!SPI2STATbits.SPIRBF);   // Wait until all bits are shifted out
    nReadData = SPI2BUF;            // Read the received buffer, to clear SPIRBF flag.
    PIN_MRF24J40_Driver_CS = 1;     // Manually deassert SS pin.
    return nReadData;
}

///
/// Function name	: SPI_SetRAMLongAdd
///
/// Author		: Fabian Kung
///
/// Last modified	: 10 Jan 2013
///
/// Code version	: 1.04
///
/// Processor		: dsPIC33EP256MU806
///
/// Description		: SPI driver for Microchip Technologies MRF24J40MA RF Transceiver Module.
///                       To set the registers in MRF24J40 chip using long address format.
///
/// Arguments		: nAdd = Register address (only 10 bits is used).
///			  bytData = data to be written into register.
///
/// Return		: None.
///
/// Global variable	: None.
void SPI_SetRAMLongAdd(UINT16 nAdd, BYTE bytData)
{
	INT16 nReadData;
	
	PIN_MRF24J40_Driver_CS = 0;	// Manually assert SS1 pin. 
					// NOTE: must be careful here to ensure the THL for SS to
					// SCK output exceeds the minimum specs.
	nAdd = (nAdd<<5)&0x7FE0;	// Shift left 5 bit ands set bit15=1, bit4=0, bit3-0 don't care.
	nAdd = nAdd|0x8010;		// bit15=1 to indicate long address format, bit4=1 for write operation.
					// (See datasheet of MRF24J40 chip).
	SPI2BUF = (nAdd>>8); 		// Shift out upper 8 bits of address data via SPI.
					// Note that data is shifted out via MSb.
	while (!SPI2STATbits.SPIRBF);	// Wait until all bits are shifted out, this is
					// equivalent to 16 bits being shifted in, triggering
					// the SPI receive buffer full flag, SPIRBF.
	nReadData = SPI2BUF;		// Read the received buffer, to clear SPIRBF flag
	SPI2BUF = nAdd; 		// Shift out lower 8 bits of address data via SPI.
	while (!SPI2STATbits.SPIRBF);	// Wait until all bits are shifted out
	nReadData = SPI2BUF;		// Read the received buffer, to clear SPIRBF flag
	SPI2BUF = bytData; 		// Shift out data via SPI. Note that data is shifted out via MSb.
					// Only first 8 bits are used.
	while (!SPI2STATbits.SPIRBF);	// Wait until all bits are shifted out
	nReadData = SPI2BUF;		// Read the received buffer, to clear SPIRBF flag.
	PIN_MRF24J40_Driver_CS = 1;	// Manually deassert SS pin.	
}

///
/// Function name	: bytSPI_GetRAMLongAdd
///
/// Author		: Fabian Kung
///
/// Last modified	: 10 Jan 2013
///
/// Code version	: 1.04
///
/// Processor		: dsPIC33EP256MU806
///
/// Description		: SPI driver for Microchip Technologies MRF24J40MA RF Transceiver Module.
///                       To get the register's content in MRF24J40 chip using long address format.
///
/// Arguments		: nAdd = Register address (only 10 bits is used).
///
/// Return	        : bytData = data read from register.
///
/// Global variable	: None.
BYTE bytSPI_GetRAMLongAdd(UINT16 nAdd)
{
	INT16 nReadData;
	
	PIN_MRF24J40_Driver_CS = 0;	// Manually assert SS pin. 
					// NOTE: must be careful here to ensure the THL for SS to
					// SCK output exceeds the minimum specs.
	nAdd = (nAdd<<5)&0x7FE0;	// Shift left 5 bit ands set bit15=1, bit4=1, bit3-0 don't care.
	nAdd = nAdd|0x8000;		// bit15=1 to indicate long address format, bit4=0 for read operation.
					// (See datasheet of MRF24J40 chip).
	SPI2BUF = (nAdd>>8); 		// Shift out upper 8 bits of address data via SPI.
					// Note that data is shifted out via MSb.
	while (!SPI2STATbits.SPIRBF);	// Wait until all bits are shifted out, this is
					// equivalent to 16 bits being shifted in, triggering
					// the SPI receive buffer full flag, SPIRBF.
	nReadData = SPI2BUF;		// Read the received buffer, to clear SPIRBF flag
	SPI2BUF = nAdd; 		// Shift out lower 8 bits of address data via SPI.
	while (!SPI2STATbits.SPIRBF);	// Wait until all bits are shifted out
	nReadData = SPI2BUF;		// Read the received buffer, to clear SPIRBF flag
	SPI2BUF = 0x00; 		// Shift out dummy data via SPI. Note that data is shifted out via MSb.
					// Only first 8 bits are used.
	while (!SPI2STATbits.SPIRBF);	// Wait until all bits are shifted out
	nReadData = SPI2BUF;		// Read the received buffer, to clear SPIRBF flag.
	PIN_MRF24J40_Driver_CS = 1;	// Manually deassert SS pin.	
	return nReadData;				
}

///
/// Function name	: nIsRFModuleBusy
///
/// Author			: Fabian Kung
///
/// Last modified	: 11 Nov 2010
///
/// Code version	: 1.02
///
/// Description		: Check to see if RF Module is busy transmitting data.
///
/// Arguments		: None.
///
/// Return			: 1 if busy, 0 if idle.
///
/// Global variable	: gSCIstatus
INT16 nIsRFModuleBusy(void)
{
    return gSCIstatus.bRFTXRDY;
}

///
/// Function name	: nIsRFModuleTXError
///
/// Author		: Fabian Kung
///
/// Last modified	: 11 Nov 2010
///
/// Code version	: 1.02
///
/// Description		: Check to see if RF Module the last transmit operation is successful.
///                       Transmission failure could be due to the number of retry exceeds limit, due to
///                       congestion in the medium (air).
///
/// Arguments		: None.
///
/// Return		: 1 if success, 0 if failed.
///
/// Global variable	: gSCIstatus
INT16 nIsRFModuleTXError(void)
{
	if (gSCIstatus.bRFTXERR == 1)
	{
		gSCIstatus.bRFTXERR = 0;	// Clear error flag.
		return 0;	// Fail.
	}
	else
	{
		return 1;	// Success.
	}
}

///
/// Function name	: nIsValiddataRFModule
///
/// Author		: Fabian Kung
///
/// Last modified	: 11 Nov 2010
///
/// Code version	: 1.02
///
/// Description		: Check to see if there is valid data frame in the RF Module receive buffer.
///
/// Arguments		: None.
///
/// Return		: 1 if there is valid data frame, 0 if none.
///
/// Global variable	: gSCIstatus
INT16 nIsValidDataRFModule(void)
{
    if (gSCIstatus.bRFRXRDY == 1)
    {
	gSCIstatus.bRFRXRDY = 0; 	// Deassert RF module receive flag.
	return 1;			// Valid data.
    }
    else
    {
	return 0;			// No valid data.
    }
}

///
/// Function name	: SendDataRFModule
///
/// Author		: Fabian Kung
///
/// Last modified	: 11 Nov 2010
///
/// Code version	: 1.02
///
/// Description		: Send a data frame via RF module.
///
/// Arguments		: ptrData = pointer to a character string (not necessary null terminated).
///                       bytLength = number of bytes to send.
///
/// Return		: None.
///
/// Global variable	: gSCIstatus
///			  gbytRFTXbuffer[]
///  			  gbytRFTXbuflen
void SendDataRFModule(BYTE *ptrData, BYTE bytLength)
{
    INT16 i;

    gSCIstatus.bRFTXRDY = 1; 			// Set transmitt flag.
    gbytRFTXbuflen = bytLength;			// Indicate data frame length.
    for (i=0; i<bytLength; i++)
    {
	gbytRFTXbuffer[i] = *ptrData;           // Load data.
	ptrData++;				// Next byte.
    }
}



