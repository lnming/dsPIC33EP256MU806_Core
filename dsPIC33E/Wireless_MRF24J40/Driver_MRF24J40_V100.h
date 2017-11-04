// Author			: Fabian Kung
// Date				: 30 July 2013
// Filename			: Driver_WirelessComm_V100.h

#ifndef _DRIVER_WIRELESSCOMM_dsPIC33E_H
#define _DRIVER_WIRELESSCOMM_dsPIC33E_H

// Include common header to all drivers and sources.  Here absolute path is used.
// To edit if one change folder
#include "osmain.h"

// 
//
// --- PUBLIC VARIABLES ---
//
// MRF24J40MA RF Module device default device address
extern BYTE gbytRFAddL;						
extern BYTE gbytRFAddH;					
extern BYTE gbytRFDesAddL;					
extern BYTE gbytRFDesAddH;					

// Data buffer and address pointers for wireless serial communications.
extern BYTE gbytRFTXbuffer[__RFSCI_TXBUF_LENGTH-1];
extern BYTE gbytRFTXbufptr;
extern BYTE gbytRFTXbuflen;
extern BYTE gbytRFRXbuffer[__RFSCI_RXBUF_LENGTH-1];
extern BYTE gbytRFRXbufptr;
extern BYTE gbytRFRXbuflen;

//
// --- PUBLIC FUNCTION PROTOTYPE ---
//

void Proce_MRF24J40_Driver(TASK_ATTRIBUTE *);

#endif
