package picc

// RxGain godoc
type RxGain byte
// Command godoc
type Command byte
// MIFAREMisc godoc
type MIFAREMisc byte
// Type godoc
type Type byte

// UID is a struct used for passing the UID of a PICC.
type UID struct {
	Size byte			// Number of bytes in the UID. 4, 7 or 10.
	UIDByte []byte
	Sak byte			// The SAK (Select acknowledge) byte returned from the PICC after successful selection.
}

const MaxLength	byte = 16

// Commands sent to the PICC.
const (
	// The commands used by the PCD to manage communication with several PICCs (ISO 14443-3, Type A, section 6.4)
	CMDReqA				Command	= 0x26		// REQuest command, Type A. Invites PICCs in state IDLE to go to READY and prepare for anticollision or selection. 7 bit frame.
	CMDWupA				Command = 0x52		// Wake-UP command, Type A. Invites PICCs in state IDLE and HALT to go to READY(*) and prepare for anticollision or selection. 7 bit frame.
	CMDCT				Command = 0x88		// Cascade Tag. Not really a command, but used during anti collision.
	CMDSelCL1			Command = 0x93		// Anti collision/Select, Cascade Level 1
	CMDSelCL2			Command = 0x95		// Anti collision/Select, Cascade Level 2
	CMDSelCL3			Command = 0x97		// Anti collision/Select, Cascade Level 3
	CMDHltA				Command = 0x50		// HaLT command, Type A. Instructs an ACTIVE PICC to go to state HALT.
	CMDRatS         	Command = 0xE0      // Request command for Answer To Reset.
	// The commands used for MIFARE Classic (from http://www.mouser.com/ds/2/302/MF1S503x-89574.pdf, Section 9)
	// Use PCD_MFAuthent to authenticate access to a sector, then use these commands to read/write/modify the blocks on the sector.
	// The read/write commands can also be used for MIFARE Ultralight.
	CMDMifareAuthKeyA	Command = 0x60		// Perform authentication with Key A
	CMDMifareAuthKeyB	Command = 0x61		// Perform authentication with Key B
	CMDMifareRead		Command = 0x30		// Reads one 16 byte block from the authenticated sector of the PICC. Also used for MIFARE Ultralight.
	CMDMifareWrite		Command = 0xA0		// Writes one 16 byte block to the authenticated sector of the PICC. Called "COMPATIBILITY WRITE" for MIFARE Ultralight.
	CMDMifareDecrement	Command = 0xC0		// Decrements the contents of a block and stores the result in the internal data register.
	CMDMifareIncrement	Command = 0xC1		// Increments the contents of a block and stores the result in the internal data register.
	CMDMifareRestore	Command = 0xC2		// Reads the contents of a block into the internal data register.
	CMDMifareTransfer	Command = 0xB0		// Writes the contents of the internal data register to a block.
	// The commands used for MIFARE Ultralight (from http://www.nxp.com/documents/data_sheet/MF0ICU1.pdf, Section 8.6)
	// The PiccCMDMF_READ and PiccCMDMF_WRITE can also be used for MIFARE Ultralight.
	CMDMifareULWrite	Command = 0xA2		// Writes one 4 byte page to the PICC.
)

const (
	MIOK 		byte = 0x0
    MINotAgErr  byte = 0x1
    MIErr 		byte = 0x2
)

// MIFARE constants that does not fit anywhere else
const (
	MifareACK		MIFAREMisc	= 0xA		// The MIFARE Classic uses a 4 bit ACK/NAK. Any other value than 0xA is NAK.
	MifareKeySize	MIFAREMisc	= 6			// A Mifare Crypto1 key is 6 bytes.
)

// PICC types we can detect. Remember to update PICC_GetTypeName() if you add more.
// last value set to 0xff, then compiler uses less ram, it seems some optimisations are triggered
const (
	TypeUnknown			Type	= 0x00
	TypeISO14443_4  	Type	= 0x01 	// PICC compliant with ISO/IEC 14443-4 
	TypeISO18092		Type	= 0x02 	// PICC compliant with ISO/IEC 18092 (NFC)
	TypeMifareMini		Type	= 0x03	// MIFARE Classic protocol, 320 bytes
	TypeMifare1k		Type	= 0x04  // MIFARE Classic protocol, 1KB
	TypeMifare4k		Type	= 0x05	// MIFARE Classic protocol, 4KB
	TypeMifareUL		Type	= 0x06	// MIFARE Ultralight or Ultralight C
	TypeMifarePlus		Type	= 0x07	// MIFARE Plus
	TypeMifareDESFire   Type	= 0x08	// MIFARE DESFire
	TypeTNP3XXX			Type	= 0x09	// Only mentioned in NXP AN 10833 MIFARE Type Identification Procedure
	TypeNotComplete 	Type	= 0xff	// SAK indicates UID is not complete.
)

// MFRC522 RxGain[2:0] masks, defines the receiver's signal voltage gain factor (on the PCD).
// Described in 9.3.3.6 / table 98 of the datasheet at http://www.nxp.com/documents/data_sheet/MFRC522.pdf
const (
	RxGain18dB		RxGain	= 0x00 << 4	// 000b - 18 dB, minimum
	RxGain23dB		RxGain	= 0x01 << 4	// 001b - 23 dB
	RxGain18dB2		RxGain	= 0x02 << 4	// 010b - 18 dB, it seems 010b is a duplicate for 000b
	RxGain23dB2		RxGain	= 0x03 << 4	// 011b - 23 dB, it seems 011b is a duplicate for 001b
	RxGain33dB		RxGain	= 0x04 << 4	// 100b - 33 dB, average, and typical default
	RxGain38dB		RxGain	= 0x05 << 4	// 101b - 38 dB
	RxGain43dB		RxGain	= 0x06 << 4	// 110b - 43 dB
	RxGain48dB		RxGain	= 0x07 << 4	// 111b - 48 dB, maximum
	RxGainMin		RxGain	= 0x00 << 4	// 000b - 18 dB, minimum, convenience for RxGain_18dB
	RxGainAvg		RxGain	= 0x04 << 4	// 100b - 33 dB, average, convenience for RxGain_33dB
	RxGainMax		RxGain	= 0x07 << 4	// 111b - 48 dB, maximum, convenience for RxGain_48dB
)

