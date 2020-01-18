package mfrc522

import (
	"github.com/stianeikeland/go-rpio/v4"
	"github.com/phires/go-mfrc522/pcd"
	"fmt"
	"log"
	"time"
)

/* **********************************************
	Pin mapping
	
	Name		Physical PIN	BCM PIN (used in rpio)
	3v3			1				--
	GND			6				--
	MOSI		19				10
	MISO		21				9
	SCK			23				11
	SDA (CE0)	24				8
	RST	(GPIO6)	22				25

	Spi PINs
	dev\pin    | CE0 | CE1 | CE2 | SCLK | MOSI | MISO | RST |
	Spi0 (bcm) |   8 |   7 |   - |    9 |   10 |   11 |  25 |
	Spi0 (phy) |  24 |  26 |   - |   23 |   19 |   21 |  22 |
*/

// PCDRxGain godoc
type PCDRxGain byte
// PICCCommand godoc
type PICCCommand byte
// MIFAREMisc godoc
type MIFAREMisc byte
// PICCType godoc
type PICCType byte
// StatusCode godoc
type StatusCode byte
// SpiSlave (CE0, CE1 or CE2)
type SpiSlave uint8

var resetPin rpio.Pin
var spiSlavePin uint8

// UID is a struct used for passing the UID of a PICC.
type UID struct {
	size	byte	// Number of bytes in the UID. 4, 7 or 10.
	uidByte [10]byte
	sak 	byte			// The SAK (Select acknowledge) byte returned from the PICC after successful selection.
}

// MifareKey is a struct used for passing a MIFARE Crypto1 key
type MifareKey struct {
	keyByte	[uint8(MifareKeySize)]byte
}

// MFRC522 RxGain[2:0] masks, defines the receiver's signal voltage gain factor (on the PCD).
// Described in 9.3.3.6 / table 98 of the datasheet at http://www.nxp.com/documents/data_sheet/MFRC522.pdf
const (
		RxGain18dB		PCDRxGain	= 0x00 << 4	// 000b - 18 dB, minimum
		RxGain23dB		PCDRxGain	= 0x01 << 4	// 001b - 23 dB
		RxGain18dB2		PCDRxGain	= 0x02 << 4	// 010b - 18 dB, it seems 010b is a duplicate for 000b
		RxGain23dB2		PCDRxGain	= 0x03 << 4	// 011b - 23 dB, it seems 011b is a duplicate for 001b
		RxGain33dB		PCDRxGain	= 0x04 << 4	// 100b - 33 dB, average, and typical default
		RxGain38dB		PCDRxGain	= 0x05 << 4	// 101b - 38 dB
		RxGain43dB		PCDRxGain	= 0x06 << 4	// 110b - 43 dB
		RxGain48dB		PCDRxGain	= 0x07 << 4	// 111b - 48 dB, maximum
		RxGainMin		PCDRxGain	= 0x00 << 4	// 000b - 18 dB, minimum, convenience for RxGain_18dB
		RxGainAvg		PCDRxGain	= 0x04 << 4	// 100b - 33 dB, average, convenience for RxGain_33dB
		RxGainMax		PCDRxGain	= 0x07 << 4	// 111b - 48 dB, maximum, convenience for RxGain_48dB
)

// Commands sent to the PICC.
const (
	// The commands used by the PCD to manage communication with several PICCs (ISO 14443-3, Type A, section 6.4)
	PiccCMDReqA				PICCCommand	= 0x26		// REQuest command, Type A. Invites PICCs in state IDLE to go to READY and prepare for anticollision or selection. 7 bit frame.
	PiccCMDWupA				PICCCommand = 0x52		// Wake-UP command, Type A. Invites PICCs in state IDLE and HALT to go to READY(*) and prepare for anticollision or selection. 7 bit frame.
	PiccCMDCT				PICCCommand = 0x88		// Cascade Tag. Not really a command, but used during anti collision.
	PiccCMDSelCL1			PICCCommand = 0x93		// Anti collision/Select, Cascade Level 1
	PiccCMDSelCL2			PICCCommand = 0x95		// Anti collision/Select, Cascade Level 2
	PiccCMDSelCL3			PICCCommand = 0x97		// Anti collision/Select, Cascade Level 3
	PiccCMDHltA				PICCCommand = 0x50		// HaLT command, Type A. Instructs an ACTIVE PICC to go to state HALT.
	PiccCMDRatS         	PICCCommand = 0xE0      // Request command for Answer To Reset.
	// The commands used for MIFARE Classic (from http://www.mouser.com/ds/2/302/MF1S503x-89574.pdf, Section 9)
	// Use PCD_MFAuthent to authenticate access to a sector, then use these commands to read/write/modify the blocks on the sector.
	// The read/write commands can also be used for MIFARE Ultralight.
	PiccCMDMifareAuthKeyA	PICCCommand = 0x60		// Perform authentication with Key A
	PiccCMDMifareAuthKeyB	PICCCommand = 0x61		// Perform authentication with Key B
	PiccCMDMifareRead		PICCCommand = 0x30		// Reads one 16 byte block from the authenticated sector of the PICC. Also used for MIFARE Ultralight.
	PiccCMDMifareWrite		PICCCommand = 0xA0		// Writes one 16 byte block to the authenticated sector of the PICC. Called "COMPATIBILITY WRITE" for MIFARE Ultralight.
	PiccCMDMifareDecrement	PICCCommand = 0xC0		// Decrements the contents of a block and stores the result in the internal data register.
	PiccCMDMifareIncrement	PICCCommand = 0xC1		// Increments the contents of a block and stores the result in the internal data register.
	PiccCMDMifareRestore	PICCCommand = 0xC2		// Reads the contents of a block into the internal data register.
	PiccCMDMifareTransfer	PICCCommand = 0xB0		// Writes the contents of the internal data register to a block.
	// The commands used for MIFARE Ultralight (from http://www.nxp.com/documents/data_sheet/MF0ICU1.pdf, Section 8.6)
	// The PiccCMDMF_READ and PiccCMDMF_WRITE can also be used for MIFARE Ultralight.
	PiccCMDMifareULWrite	PICCCommand = 0xA2		// Writes one 4 byte page to the PICC.
)

// MIFARE constants that does not fit anywhere else
const (
	MifareACK		MIFAREMisc	= 0xA		// The MIFARE Classic uses a 4 bit ACK/NAK. Any other value than 0xA is NAK.
	MifareKeySize	MIFAREMisc	= 6			// A Mifare Crypto1 key is 6 bytes.
)

// PICC types we can detect. Remember to update PICC_GetTypeName() if you add more.
// last value set to 0xff, then compiler uses less ram, it seems some optimisations are triggered
const (
	PiccTypeUnknown			PICCType	= 0x00
	PiccTypeISO14443_4  	PICCType	= 0x01 	// PICC compliant with ISO/IEC 14443-4 
	PiccTypeISO18092		PICCType	= 0x02 	// PICC compliant with ISO/IEC 18092 (NFC)
	PiccTypeMifareMini		PICCType	= 0x03	// MIFARE Classic protocol, 320 bytes
	PiccTypeMifare1k		PICCType	= 0x04  // MIFARE Classic protocol, 1KB
	PiccTypeMifare4k		PICCType	= 0x05	// MIFARE Classic protocol, 4KB
	PiccTypeMifareUL		PICCType	= 0x06	// MIFARE Ultralight or Ultralight C
	PiccTypeMifarePlus		PICCType	= 0x07	// MIFARE Plus
	PiccTypeMifareDESFire   PICCType	= 0x08	// MIFARE DESFire
	PiccTypeTNP3XXX			PICCType	= 0x09	// Only mentioned in NXP AN 10833 MIFARE Type Identification Procedure
	PiccTypeNotComplete 	PICCType	= 0xff	// SAK indicates UID is not complete.
)

// Return codes from the functions in this class. Remember to update GetStatusCodeName() if you add more.
// last value set to 0xff, then compiler uses less ram, it seems some optimisations are triggered
const (
	StatusOK				StatusCode = 0x00	// Success
	StatusError				StatusCode = 0x01	// Error in communication
	StatusCollision			StatusCode = 0x02	// Collission detected
	StatusTimeout			StatusCode = 0x03	// Timeout in communication.
	StatusNoRoom			StatusCode = 0x04	// A buffer is not big enough.
	StatusInternalError		StatusCode = 0x05	// Internal error in the code. Should not happen ;-)
	StatusInvalid			StatusCode = 0x06	// Invalid argument.
	StatusCRCWrong			StatusCode = 0x07	// The CRC_A does not match
	StatusMifareNACK		StatusCode = 0xff	// A MIFARE PICC responded with NAK.
)

// SpiSlave PIN (CE0 = 8/24, CE1 = 7/26)
const (
	CE0 = iota
	CE1
	CE2
)

// Init sets the GPIO to SPI and resets the reader
func Init(spiSlave SpiSlave) {
	spiSlavePin = uint8(spiSlave)

	log.Printf("PCDInit          | Initializing | SPI Slave %d\n", spiSlavePin)
	if err := rpio.Open(); err != nil {
		panic(err)
	}
	rpio.SpiSpeed(500000)
	resetPin = rpio.Pin(25)
	if err := rpio.SpiBegin(rpio.Spi0); err != nil {
		panic(err)
	}
	rpio.SpiChipSelect(spiSlavePin) // Select SPI slave
	log.Println("PCDInit          | Sending reset...")
	hardReset := false
	resetPin.Mode(rpio.Input)
	r := rpio.ReadPin(resetPin)
	log.Printf("PCDInit          | reset pin %#2x\n", byte(r))
	if r == rpio.Low {				// The MFRC522 chip is in power down mode.
		resetPin.Mode(rpio.Output)
		rpio.WritePin(resetPin, rpio.Low)
		time.Sleep(time.Microsecond*2)
		rpio.WritePin(resetPin, rpio.High)
		time.Sleep(time.Millisecond*50)
		hardReset = true
	}
	if !hardReset {
		pcd.Reset()
	}

	log.Println("PCDInit          | Reset baud rates")
	pcd.WriteValueRegister(pcd.TxModeReg, 0x00)
	pcd.WriteValueRegister(pcd.RxModeReg, 0x00)
	//log.Println("PCDInit          | Reset ModWidthReg")
	//PCDWriteValueRegister(ModWidthReg, 0x26)

	pcd.WriteValueRegister(pcd.TModeReg, 0x8D)	
	pcd.WriteValueRegister(pcd.TPrescalerReg, 0x3E)	
	pcd.WriteValueRegister(pcd.TReloadRegL, 30)
	pcd.WriteValueRegister(pcd.TReloadRegH, 0)	
	
	pcd.WriteValueRegister(pcd.TxASKReg, 0x40)		// Default 0x00. Force a 100 % ASK modulation independent of the ModGsPReg register setting
	pcd.WriteValueRegister(pcd.ModeReg, 0x3D)		// Default 0x3F. Set the preset value for the CRC coprocessor for the CalcCRC command to 0x6363 (ISO 14443-3 part 6.2.4)
	//PCDAntennaOff()
	pcd.AntennaOn()								// Enable the antenna driver pins TX1 and TX2 (they were disabled by the reset)
	log.Println("PCDInit          | Init done")
}

// Dispose closes SPI
func Dispose() {
	log.Println("Dispose          | Closing SPI")
	rpio.SpiEnd(rpio.Spi0)
	rpio.Close()
}




// PCDDumpVersionToLog get and dump the MFRC522 version
func PCDDumpVersionToLog() {
	// Get the MFRC522 firmware version
	v, _ := pcd.ReadRegister(pcd.VersionReg);

	// Lookup which version
	switch(v) {
	case 0x88:
		log.Printf("PCDDumpVersion   | Firmware Version %#02x = clone\n", v)
		break
	case 0x90: 
		log.Printf("PCDDumpVersion   | Firmware Version %#02x = v0.0\n", v)
		break
	case 0x91: 
		log.Printf("PCDDumpVersion   | Firmware Version %#02x = v1.0\n", v)
		break
	case 0x92:
		log.Printf("PCDDumpVersion   | Firmware Version %#02x = v2.0\n", v)
		break
	case 0x12: 
		log.Printf("PCDDumpVersion   | Firmware Version %#02x = counterfeit chip\n", v)
		break
	default:
		log.Printf("PCDDumpVersion   | Firmware Version %#02x = (unknown)\n", v)
	}
	// When 0x00 or 0xFF is returned, communication probably failed
	if ((v == 0x00) || (v == 0xFF)) {
		log.Println("PCDDumpVersion   | WARNING: Communication failure, is the MFRC522 properly connected?")
	}
} 

/*
// PCDCalculateCRC Use the CRC coprocessor in the MFRC522 to calculate a CRC_A.
func PCDCalculateCRC(data []byte) ([]byte, StatusCode) {
	log.Printf("PCDCalculateCRC  | %s\n", dumpByteArray(data))	
	PCDWriteValueRegister(CommandReg, byte(PCDIdle))		// Stop any active command.
	PCDWriteValueRegister(DivIrqReg, 0x04)				// Clear the CRCIRq interrupt request bit
	PCDSetRegisterBitMask(FIFOLevelReg, 0x80)			// FlushBuffer = 1, FIFO initialization
	PCDWriteValuesRegister(FIFODataReg, data)		// Write data to the FIFO
	PCDWriteValueRegister(CommandReg, byte(PCDCalcCRC))	// Start the calculation

	c1 := make(chan []byte, 1)
    go func() {
		result := make([]byte, 2)
		for {
			n, _ := PCDReadRegister(DivIrqReg)
			if (n & 0x04) == 0x01 {									// CRCIRq bit set - calculation done
				PCDWriteValueRegister(CommandReg, byte(PCDIdle))	// Stop calculating CRC for new content in the FIFO.
				// Transfer the result from the registers to the result buffer
				result[0], _ = PCDReadRegister(CRCResultRegL)
				result[1], _ = PCDReadRegister(CRCResultRegH)
				c1 <- result;
			}
		}
    }()
	select {
    case res := <-c1:
        return res, StatusOK
    case <-time.After(time.Millisecond*250):		// Wait 100ms for CRC calc to finish
		return nil, StatusTimeout
    }
} 

func PCDRandomID() {
	log.Printf("PCDRandomID      | \n")
	PCDSendCommand(PCDIdle)		// Stop any active command.
	PCDWriteValueRegister(DivIrqReg, 0x04)				// Clear the CRCIRq interrupt request bit
	PCDSetRegisterBitMask(FIFOLevelReg, 0x80)			// FlushBuffer = 1, FIFO initialization

	PCDSendCommand(PCDGenerateRandomID)		// Create random id
	time.Sleep(time.Millisecond*100)		// give some time
	PCDSendCommand(PCDMem)		// Move internal buffer to FIFO
}

*/

func dumpByteArray(values []byte) string {
	var result string
	for i:=0; i < len(values); i++ {
		result = result + fmt.Sprintf("%#02x ", values[i])
	}
	return result
}