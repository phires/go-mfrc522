package pcd

// Register MFRC522 registers. Described in chapter 9 of the datasheet.
type Register byte
// Command godoc
type Command byte
// StatusCode godoc
type StatusCode byte

// When using SPI all addresses need to be shifted one bit left in the "SPI address byte" (section 8.1.2.3)
const (
	// Page 0: Command and status
	//						  0x00			// reserved for future use
	CommandReg 			Register	= 0x01 		// starts and stops command execution
	ComIEnReg 			Register	= 0x02 		// enable and disable interrupt request control bits
	DivIEnReg			Register	= 0x03  	// enable and disable interrupt request control bits
	ComIrqReg			Register	= 0x04  	// interrupt request bits
	DivIrqReg			Register	= 0x05  	// interrupt request bits
	ErrorReg			Register	= 0x06  	// error bits showing the error status of the last command executed 
	Status1Reg			Register	= 0x07  	// communication status bits
	Status2Reg			Register	= 0x08  	// receiver and transmitter status bits
	FIFODataReg			Register	= 0x09  	// input and output of 64 byte FIFO buffer
	FIFOLevelReg		Register	= 0x0A  	// number of bytes stored in the FIFO buffer
	WaterLevelReg		Register	= 0x0B  	// level for FIFO underflow and overflow warning
	ControlReg			Register	= 0x0C  	// miscellaneous control registers
	BitFramingReg		Register	= 0x0D  	// adjustments for bit-oriented frames
	CollReg				Register	= 0x0E  	// bit position of the first bit-collision detected on the RF interface
	//						  0x0F			// reserved for future use
	
	// Page 1: Command
	// 						  0x10			// reserved for future use
	ModeReg				Register	= 0x11  	// defines general modes for transmitting and receiving 
	TxModeReg			Register	= 0x12  	// defines transmission data rate and framing
	RxModeReg			Register	= 0x13  	// defines reception data rate and framing
	TxControlReg		Register	= 0x14  	// controls the logical behavior of the antenna driver pins TX1 and TX2
	TxASKReg			Register	= 0x15  	// controls the setting of the transmission modulation
	TxSelReg			Register	= 0x16  	// selects the internal sources for the antenna driver
	RxSelReg			Register	= 0x17  	// selects internal receiver settings
	RxThresholdReg		Register	= 0x18  	// selects thresholds for the bit decoder
	DemodReg			Register	= 0x19  	// defines demodulator settings
	// 						  0x1A				// reserved for future use
	// 						  0x1B				// reserved for future use
	MfTxReg				Register	= 0x1C  	// controls some MIFARE communication transmit parameters
	MfRxReg				Register	= 0x1D  	// controls some MIFARE communication receive parameters
	// 						  0x1E				// reserved for future use
	SerialSpeedReg		Register	= 0x1F  	// selects the speed of the serial UART interface
	
	// Page 2: Configuration
	// 						  0x20				// reserved for future use
	CRCResultRegH		Register	= 0x21 		// shows the MSB and LSB values of the CRC calculation
	CRCResultRegL		Register	= 0x22  
	// 						  0x23				// reserved for future use
	ModWidthReg			Register	= 0x24  	// controls the ModWidth setting?
	// 						  0x25				// reserved for future use
	RFCfgReg			Register	= 0x26  	// configures the receiver gain
	GsNReg				Register	= 0x27  	// selects the conductance of the antenna driver pins TX1 and TX2 for modulation 
	CWGsPReg			Register	= 0x28  	// defines the conductance of the p-driver output during periods of no modulation
	ModGsPReg			Register	= 0x29  	// defines the conductance of the p-driver output during periods of modulation
	TModeReg			Register	= 0x2A  	// defines settings for the internal timer
	TPrescalerReg		Register	= 0x2B  	// the lower 8 bits of the TPrescaler value. The 4 high bits are in TModeReg.
	TReloadRegH			Register	= 0x2C  	// defines the 16-bit timer reload value
	TReloadRegL			Register	= 0x2D  
	TCounterValueRegH	Register	= 0x2E  	// shows the 16-bit timer value
	TCounterValueRegL	Register	= 0x2F  
	
	// Page 3: Test Registers
	// 						  0x30				// reserved for future use
	TestSel1Reg			Register	= 0x31  	// general test signal configuration
	TestSel2Reg			Register	= 0x32 		// general test signal configuration
	TestPinEnReg		Register	= 0x33 		// enables pin output driver on pins D1 to D7
	TestPinValueReg		Register	= 0x34  	// defines the values for D1 to D7 when it is used as an I/O bus
	TestBusReg			Register	= 0x35  	// shows the status of the internal test bus
	AutoTestReg			Register	= 0x36  	// controls the digital self-test
	VersionReg			Register	= 0x37  	// shows the software version
	AnalogTestReg		Register	= 0x38  	// controls the pins AUX1 and AUX2
	TestDAC1Reg			Register	= 0x39  	// defines the test value for TestDAC1
	TestDAC2Reg			Register	= 0x3A  	// defines the test value for TestDAC2
	TestADCReg			Register	= 0x3B 		// shows the value of ADC I and Q channels
	// 						  0x3C				// reserved for production tests
	// 						  0x3D				// reserved for production tests
	// 						  0x3E				// reserved for production tests
	// 						  0x3F				// reserved for production tests
)

// MFRC522 commands. Described in chapter 10 of the datasheet.
const (
	CommandIdle				Command	= 0x00		// no action, cancels current command execution
	CommandMem				Command	= 0x01		// stores 25 bytes into the internal buffer
	CommandGenerateRandomID	Command	= 0x02		// generates a 10-byte random ID number
	CommandCalcCRC			Command	= 0x03		// activates the CRC coprocessor or performs a self-test
	CommandTransmit			Command	= 0x04		// transmits data from the FIFO buffer
	CommandNoCmdChange		Command	= 0x07		// no command change, can be used to modify the CommandReg register bits without affecting the command, for example, the PowerDown bit
	CommandReceive			Command	= 0x08		// activates the receiver circuits
	CommandTransceive 		Command	= 0x0C		// transmits data from FIFO buffer to antenna and automatically activates the receiver after transmission
	CommandMFAuthent 		Command	= 0x0E		// performs the MIFARE standard authentication as a reader
	CommandSoftReset		Command	= 0x0F		// resets the MFRC522
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


// Firmware data for self-test
// Reference values based on firmware version

// FirmwareReferenceV0_0 Version 0.0 (0x90)
// Philips Semiconductors; Preliminary Specification Revision 2.0 - 01 August 2005; 16.1 self-test
var FirmwareReferenceV0_0 =	[]byte {
	0x00, 0x87, 0x98, 0x0f, 0x49, 0xFF, 0x07, 0x19,
	0xBF, 0x22, 0x30, 0x49, 0x59, 0x63, 0xAD, 0xCA,
	0x7F, 0xE3, 0x4E, 0x03, 0x5C, 0x4E, 0x49, 0x50,
	0x47, 0x9A, 0x37, 0x61, 0xE7, 0xE2, 0xC6, 0x2E,
	0x75, 0x5A, 0xED, 0x04, 0x3D, 0x02, 0x4B, 0x78,
	0x32, 0xFF, 0x58, 0x3B, 0x7C, 0xE9, 0x00, 0x94,
	0xB4, 0x4A, 0x59, 0x5B, 0xFD, 0xC9, 0x29, 0xDF,
	0x35, 0x96, 0x98, 0x9E, 0x4F, 0x30, 0x32, 0x8D,
}
// FirmwareReferenceV1_0 Version 1.0 (0x91)
// NXP Semiconductors; Rev. 3.8 - 17 September 2014; 16.1.1 self-test
var FirmwareReferenceV1_0 = []byte {
	0x00, 0xC6, 0x37, 0xD5, 0x32, 0xB7, 0x57, 0x5C,
	0xC2, 0xD8, 0x7C, 0x4D, 0xD9, 0x70, 0xC7, 0x73,
	0x10, 0xE6, 0xD2, 0xAA, 0x5E, 0xA1, 0x3E, 0x5A,
	0x14, 0xAF, 0x30, 0x61, 0xC9, 0x70, 0xDB, 0x2E,
	0x64, 0x22, 0x72, 0xB5, 0xBD, 0x65, 0xF4, 0xEC,
	0x22, 0xBC, 0xD3, 0x72, 0x35, 0xCD, 0xAA, 0x41,
	0x1F, 0xA7, 0xF3, 0x53, 0x14, 0xDE, 0x7E, 0x02,
	0xD9, 0x0F, 0xB5, 0x5E, 0x25, 0x1D, 0x29, 0x79,
}
// FirmwareReferenceV2_0 Version 2.0 (0x92)
// NXP Semiconductors; Rev. 3.8 - 17 September 2014; 16.1.1 self-test
var FirmwareReferenceV2_0 = []byte {
	0x00, 0xEB, 0x66, 0xBA, 0x57, 0xBF, 0x23, 0x95,
	0xD0, 0xE3, 0x0D, 0x3D, 0x27, 0x89, 0x5C, 0xDE,
	0x9D, 0x3B, 0xA7, 0x00, 0x21, 0x5B, 0x89, 0x82,
	0x51, 0x3A, 0xEB, 0x02, 0x0C, 0xA5, 0x00, 0x49,
	0x7C, 0x84, 0x4D, 0xB3, 0xCC, 0xD2, 0x1B, 0x81,
	0x5D, 0x48, 0x76, 0xD5, 0x71, 0x61, 0x21, 0xA9,
	0x86, 0x96, 0x83, 0x38, 0xCF, 0x9D, 0x5B, 0x6D,
	0xDC, 0x15, 0xBA, 0x3E, 0x7D, 0x95, 0x3B, 0x2F,
}
// FM17522FirmwareReference is a clone 
// Fudan Semiconductor FM17522 (0x88)
var FM17522FirmwareReference = []byte {
	0x00, 0xD6, 0x78, 0x8C, 0xE2, 0xAA, 0x0C, 0x18,
	0x2A, 0xB8, 0x7A, 0x7F, 0xD3, 0x6A, 0xCF, 0x0B,
	0xB1, 0x37, 0x63, 0x4B, 0x69, 0xAE, 0x91, 0xC7,
	0xC3, 0x97, 0xAE, 0x77, 0xF4, 0x37, 0xD7, 0x9B,
	0x7C, 0xF5, 0x3C, 0x11, 0x8F, 0x15, 0xC3, 0xD7,
	0xC1, 0x5B, 0x00, 0x2A, 0xD0, 0x75, 0xDE, 0x9E,
	0x51, 0x64, 0xAB, 0x3E, 0xE9, 0x15, 0xB5, 0xAB,
	0x56, 0x9A, 0x98, 0x82, 0x26, 0xEA, 0x2A, 0x62,
}



func commandByteToString(reg Command) string {
	switch reg {
	case CommandIdle:
		return "Idle"
	case CommandSoftReset:
		return "SoftReset"
	case CommandMem:
		return "Mem"
	case CommandGenerateRandomID:
		return "Generate RandomID"
	case CommandCalcCRC:
		return "CalcCRC"
	case CommandTransmit:
		return "Transmit"
	case CommandNoCmdChange:
		return "NoCmdChange"
	case CommandReceive:
		return "Receive"
	case CommandTransceive:
		return "Transceive"
	case CommandMFAuthent:
		return "MFAuthent"
	}
	return ""
}

func registerByteToString(reg Register) string {
	switch reg {
	case CommandReg:
		return "CommandReg"
	case ComIEnReg:
		return "ComIEnReg"
	case DivIEnReg:
		return "DivIEnReg"
	case ComIrqReg:
		return "ComIrqReg"
	case DivIrqReg:
		return "DivIrqReg"
	case ErrorReg:
		return "ErrorReg"
	case Status1Reg:
		return "Status1Reg"
	case Status2Reg:
		return "Status2Reg"
	case FIFODataReg:
		return "FIFODataReg"
	case FIFOLevelReg:
		return "FIFOLevelReg"
	case WaterLevelReg:
		return "WaterLevelReg"
	case ControlReg:
		return "ControlReg"
	case BitFramingReg:
		return "BitFramingReg"
	case CollReg:
		return "CollReg"
	case ModeReg:
		return "ModeReg"
	case TxModeReg:
		return "TxModeReg"
	case RxModeReg:
		return "RxModeReg"
	case TxControlReg:
		return "TxControlReg"
	case TxASKReg:
		return "TxASKReg"
	case TxSelReg:
		return "TxSelReg"
	case RxSelReg:
		return "RxSelReg"
	case RxThresholdReg:
		return "RxThresholdReg"
	case DemodReg:
		return "DemodReg"
	case MfTxReg:
		return "MfTxReg"
	case MfRxReg:
		return "MfRxReg"
	case SerialSpeedReg:
		return "SerialSpeedReg"
	case CRCResultRegH:
		return "CRCResultRegH"
	case CRCResultRegL:
		return "CRCResultRegL"
	case ModWidthReg:
		return "ModWidthReg"
	case RFCfgReg:
		return "RFCfgReg"
	case GsNReg:
		return "GsNReg"
	case CWGsPReg:
		return "CWGsPReg"
	case ModGsPReg:
		return "ModGsPReg"
	case TModeReg:
		return "TModeReg"
	case TPrescalerReg:
		return "TPrescalerReg"
	case TReloadRegH:
		return "TReloadRegH"
	case TReloadRegL:
		return "TReloadRegL"
	case TCounterValueRegH:
		return "TCounterValueRegH"
	case TCounterValueRegL:
		return "TCounterValueRegL"
	case TestSel1Reg:
		return "TestSel1Reg"
	case TestSel2Reg:
		return "TestSel2Reg"
	case TestPinEnReg:
		return "TestPinEnReg"
	case TestPinValueReg:
		return "TestPinValueReg"
	case TestBusReg:
		return "TestBusReg"
	case AutoTestReg:
		return "AutoTestReg"
	case VersionReg:
		return "VersionReg"
	case AnalogTestReg:
		return "AnalogTestReg"
	case TestDAC1Reg:
		return "TestDAC1Reg"
	case TestDAC2Reg:
		return "TestDAC2Reg"
	case TestADCReg:
		return "TestADCReg"

	}
	return ""
}
