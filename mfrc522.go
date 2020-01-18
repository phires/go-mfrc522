package mfrc522

import (
	"github.com/stianeikeland/go-rpio/v4"
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

// PCDRegister MFRC522 registers. Described in chapter 9 of the datasheet.
type PCDRegister byte
// PCDCommand godoc
type PCDCommand byte
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

// When using SPI all addresses are shifted one bit left in the "SPI address byte" (section 8.1.2.3)
const (
	// Page 0: Command and status
	//						  0x00			// reserved for future use
	CommandReg 		PCDRegister		= 0x01 << 1		// starts and stops command execution
	ComIEnReg 		PCDRegister		= 0x02 << 1		// enable and disable interrupt request control bits
	DivIEnReg		PCDRegister		= 0x03 << 1 	// enable and disable interrupt request control bits
	ComIrqReg		PCDRegister		= 0x04 << 1 	// interrupt request bits
	DivIrqReg		PCDRegister		= 0x05 << 1 	// interrupt request bits
	ErrorReg		PCDRegister		= 0x06 << 1 	// error bits showing the error status of the last command executed 
	Status1Reg		PCDRegister		= 0x07 << 1 	// communication status bits
	Status2Reg		PCDRegister		= 0x08 << 1 	// receiver and transmitter status bits
	FIFODataReg		PCDRegister		= 0x09 << 1 	// input and output of 64 byte FIFO buffer
	FIFOLevelReg	PCDRegister		= 0x0A << 1 	// number of bytes stored in the FIFO buffer
	WaterLevelReg	PCDRegister		= 0x0B << 1 	// level for FIFO underflow and overflow warning
	ControlReg		PCDRegister		= 0x0C << 1 	// miscellaneous control registers
	BitFramingReg	PCDRegister		= 0x0D << 1 	// adjustments for bit-oriented frames
	CollReg			PCDRegister		= 0x0E << 1 	// bit position of the first bit-collision detected on the RF interface
	//						  0x0F			// reserved for future use
	
	// Page 1: Command
	// 						  0x10			// reserved for future use
	ModeReg			PCDRegister		= 0x11 << 1 	// defines general modes for transmitting and receiving 
	TxModeReg		PCDRegister		= 0x12 << 1 	// defines transmission data rate and framing
	RxModeReg		PCDRegister		= 0x13 << 1 	// defines reception data rate and framing
	TxControlReg	PCDRegister		= 0x14 << 1 	// controls the logical behavior of the antenna driver pins TX1 and TX2
	TxASKReg		PCDRegister		= 0x15 << 1 	// controls the setting of the transmission modulation
	TxSelReg		PCDRegister		= 0x16 << 1 	// selects the internal sources for the antenna driver
	RxSelReg		PCDRegister		= 0x17 << 1 	// selects internal receiver settings
	RxThresholdReg	PCDRegister		= 0x18 << 1 	// selects thresholds for the bit decoder
	DemodReg		PCDRegister		= 0x19 << 1 	// defines demodulator settings
	// 						  0x1A			// reserved for future use
	// 						  0x1B			// reserved for future use
	MfTxReg			PCDRegister		= 0x1C << 1 	// controls some MIFARE communication transmit parameters
	MfRxReg			PCDRegister		= 0x1D << 1 	// controls some MIFARE communication receive parameters
	// 						  0x1E			// reserved for future use
	SerialSpeedReg	PCDRegister		= 0x1F << 1 	// selects the speed of the serial UART interface
	
	// Page 2: Configuration
	// 						  0x20			// reserved for future use
	CRCResultRegH	PCDRegister		= 0x21 << 1 	// shows the MSB and LSB values of the CRC calculation
	CRCResultRegL	PCDRegister		= 0x22 << 1 
	// 						  0x23			// reserved for future use
	ModWidthReg		PCDRegister		= 0x24 << 1 	// controls the ModWidth setting?
	// 						  0x25			// reserved for future use
	RFCfgReg		PCDRegister		= 0x26 << 1 	// configures the receiver gain
	GsNReg			PCDRegister		= 0x27 << 1 	// selects the conductance of the antenna driver pins TX1 and TX2 for modulation 
	CWGsPReg		PCDRegister		= 0x28 << 1 	// defines the conductance of the p-driver output during periods of no modulation
	ModGsPReg		PCDRegister		= 0x29 << 1 	// defines the conductance of the p-driver output during periods of modulation
	TModeReg		PCDRegister		= 0x2A << 1 	// defines settings for the internal timer
	TPrescalerReg	PCDRegister		= 0x2B << 1 	// the lower 8 bits of the TPrescaler value. The 4 high bits are in TModeReg.
	TReloadRegH		PCDRegister		= 0x2C << 1 	// defines the 16-bit timer reload value
	TReloadRegL		PCDRegister		= 0x2D << 1 
	TCounterValueRegH	PCDRegister	= 0x2E << 1 	// shows the 16-bit timer value
	TCounterValueRegL	PCDRegister	= 0x2F << 1 
	
	// Page 3: Test Registers
	// 						  0x30			// reserved for future use
	TestSel1Reg		PCDRegister		= 0x31 << 1 	// general test signal configuration
	TestSel2Reg		PCDRegister		= 0x32 << 1		// general test signal configuration
	TestPinEnReg	PCDRegister		= 0x33 << 1		// enables pin output driver on pins D1 to D7
	TestPinValueReg	PCDRegister		= 0x34 << 1 	// defines the values for D1 to D7 when it is used as an I/O bus
	TestBusReg		PCDRegister		= 0x35 << 1 	// shows the status of the internal test bus
	AutoTestReg		PCDRegister		= 0x36 << 1 	// controls the digital self-test
	VersionReg		PCDRegister		= 0x37 << 1 	// shows the software version
	AnalogTestReg	PCDRegister		= 0x38 << 1 	// controls the pins AUX1 and AUX2
	TestDAC1Reg		PCDRegister		= 0x39 << 1 	// defines the test value for TestDAC1
	TestDAC2Reg		PCDRegister		= 0x3A << 1 	// defines the test value for TestDAC2
	TestADCReg		PCDRegister		= 0x3B << 1		// shows the value of ADC I and Q channels
	// 						  0x3C			// reserved for production tests
	// 						  0x3D			// reserved for production tests
	// 						  0x3E			// reserved for production tests
	// 						  0x3F			// reserved for production tests
)

// MFRC522 commands. Described in chapter 10 of the datasheet.
const (
	PCDIdle				PCDCommand	= 0x00		// no action, cancels current command execution
	PCDMem				PCDCommand	= 0x01		// stores 25 bytes into the internal buffer
	PCDGenerateRandomID	PCDCommand	= 0x02		// generates a 10-byte random ID number
	PCDCalcCRC			PCDCommand	= 0x03		// activates the CRC coprocessor or performs a self-test
	PCDTransmit			PCDCommand	= 0x04		// transmits data from the FIFO buffer
	PCDNoCmdChange		PCDCommand	= 0x07		// no command change, can be used to modify the CommandReg register bits without affecting the command, for example, the PowerDown bit
	PCDReceive			PCDCommand	= 0x08		// activates the receiver circuits
	PCDTransceive 		PCDCommand	= 0x0C		// transmits data from FIFO buffer to antenna and automatically activates the receiver after transmission
	PCDMFAuthent 		PCDCommand	= 0x0E		// performs the MIFARE standard authentication as a reader
	PCDSoftReset		PCDCommand	= 0x0F		// resets the MFRC522
)

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
	rpio.SpiSpeed(1000000)
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
		PCDReset()
	}

	log.Println("PCDInit          | Reset baud rates")
	PCDWriteValueRegister(TxModeReg, 0x00)
	PCDWriteValueRegister(RxModeReg, 0x00)
	//log.Println("PCDInit          | Reset ModWidthReg")
	//PCDWriteValueRegister(ModWidthReg, 0x26)

	PCDWriteValueRegister(TModeReg, 0x8D)	
	PCDWriteValueRegister(TPrescalerReg, 0x3E)	
	PCDWriteValueRegister(TReloadRegL, 30)
	PCDWriteValueRegister(TReloadRegH, 0)	
	
	PCDWriteValueRegister(TxASKReg, 0x40)		// Default 0x00. Force a 100 % ASK modulation independent of the ModGsPReg register setting
	PCDWriteValueRegister(ModeReg, 0x3D)		// Default 0x3F. Set the preset value for the CRC coprocessor for the CalcCRC command to 0x6363 (ISO 14443-3 part 6.2.4)
	//PCDAntennaOff()
	PCDAntennaOn()								// Enable the antenna driver pins TX1 and TX2 (they were disabled by the reset)
	log.Println("PCDInit          | Init done")
}

// Dispose closes SPI
func Dispose() {
	log.Println("Dispose          | Closing SPI")
	rpio.SpiEnd(rpio.Spi0)
	rpio.Close()
}

// PCDSendCommand sends a single command to MFRC522
func PCDSendCommand(cmd PCDCommand) error {
	log.Printf("PCDSendCommand   | Command  %#02x\n", cmd)
	rpio.SpiTransmit(byte(CommandReg))				// MSB == 0 is for writing. LSB is not used in address. Datasheet section 8.1.2.3.
	rpio.SpiTransmit(byte(cmd))
	return nil
}

// PCDWriteValueRegister writes a single byte to a register
func PCDWriteValueRegister(reg PCDRegister, value byte) error {
	log.Printf("PCDWriteRegister | Register %#02x | Value  %#02x\n", reg, value)
	rpio.SpiTransmit(byte(reg))				// MSB == 0 is for writing. LSB is not used in address. Datasheet section 8.1.2.3.
	rpio.SpiTransmit(value)
	return nil
} 

// PCDWriteValuesRegister Writes a number of bytes to the specified register in the MFRC522 chip.
// The interface is described in the datasheet section 8.1.2.
func PCDWriteValuesRegister(reg PCDRegister, values []byte) error {
	var count int = len(values)
	s := dumpByteArray(values)
	log.Printf("PCDWriteRegister | Register %#02x | Length %d | Value  %s\n", reg, count, s)

	rpio.SpiTransmit(byte(reg))				// MSB == 0 is for writing. LSB is not used in address. Datasheet section 8.1.2.3.
	rpio.SpiTransmit(values...)
	
	return nil
} 

// PCDReadRegister reads a single byte from a register
func PCDReadRegister(reg PCDRegister) (byte, error) {
	buffer := []byte{ 0x80 | (byte(reg) & 0x7E) } 	// MSB == 1 is for reading. LSB is not used in address. Datasheet section 8.1.2.3.
	rpio.SpiExchange(buffer)		
	log.Printf("PCDReadRegister  | Register %#02x | Result %#02x\n", reg, buffer[0])	
	return buffer[0], nil
} 

// PCDClearRegisterBitMask  godoc
func PCDClearRegisterBitMask(reg PCDRegister, mask byte) error {
	tmp, err := PCDReadRegister(reg);
	if err != nil {
		return err
	}
	PCDWriteValueRegister(reg, tmp & (^mask));		// clear bit mask
	return nil
} 

// PCDSetRegisterBitMask  godoc
func PCDSetRegisterBitMask(reg PCDRegister, mask byte) error {
	tmp, err := PCDReadRegister(reg);
	if err != nil {
		return err
	}
	PCDWriteValueRegister(reg, tmp | mask);		
	return nil
} 

// PCDReset issues the SoftReset command.
func PCDReset() {
	PCDWriteValueRegister(CommandReg, byte(PCDSoftReset))	
	// The datasheet does not mention how long the SoftRest command takes to complete.
	// But the MFRC522 might have been in soft power-down mode (triggered by bit 4 of CommandReg) 
	// Section 8.8.2 in the datasheet says the oscillator start-up time is the start up time of the crystal + 37,74μs. Let us be generous: 50ms.
	var count uint8

	for {
		v, err := PCDReadRegister(CommandReg)
		if !((v & (1 << 4)) == 0 && count < 3) || err != nil  {
			break
		}
		count = count + 1
		time.Sleep(time.Millisecond*50)
	}
}

// PCDAntennaOn turns on the antenna if it is disabled currently
func PCDAntennaOn() {
	value, _ := PCDReadRegister(TxControlReg)
	log.Printf("PCDAntennaOn     | Antenna status %#02x\n", value & 0x03)	
	if ((value & 0x03) != 0x03) {
		PCDWriteValueRegister(TxControlReg, value | 0x03)
	}
}

func PCDAntennaOff() {
	PCDClearRegisterBitMask(TxControlReg, 0x03)
}

// PCDDumpVersionToLog get and dump the MFRC522 version
func PCDDumpVersionToLog() {
	// Get the MFRC522 firmware version
	v, _ := PCDReadRegister(VersionReg);

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

func dumpByteArray(values []byte) string {
	var result string
	for i:=0; i < len(values); i++ {
		result = result + fmt.Sprintf("%#02x ", values[i])
	}
	return result
}