package pcd

// Register MFRC522 registers. Described in chapter 9 of the datasheet.
type Register byte

// Command godoc
type Command byte

// When using SPI all addresses are shifted one bit left in the "SPI address byte" (section 8.1.2.3)
const (
	// Page 0: Command and status
	//						  0x00			// reserved for future use
	CommandReg 		Register		= 0x01 << 1		// starts and stops command execution
	ComIEnReg 		Register		= 0x02 << 1		// enable and disable interrupt request control bits
	DivIEnReg		Register		= 0x03 << 1 	// enable and disable interrupt request control bits
	ComIrqReg		Register		= 0x04 << 1 	// interrupt request bits
	DivIrqReg		Register		= 0x05 << 1 	// interrupt request bits
	ErrorReg		Register		= 0x06 << 1 	// error bits showing the error status of the last command executed 
	Status1Reg		Register		= 0x07 << 1 	// communication status bits
	Status2Reg		Register		= 0x08 << 1 	// receiver and transmitter status bits
	FIFODataReg		Register		= 0x09 << 1 	// input and output of 64 byte FIFO buffer
	FIFOLevelReg	Register		= 0x0A << 1 	// number of bytes stored in the FIFO buffer
	WaterLevelReg	Register		= 0x0B << 1 	// level for FIFO underflow and overflow warning
	ControlReg		Register		= 0x0C << 1 	// miscellaneous control registers
	BitFramingReg	Register		= 0x0D << 1 	// adjustments for bit-oriented frames
	CollReg			Register		= 0x0E << 1 	// bit position of the first bit-collision detected on the RF interface
	//						  0x0F			// reserved for future use
	
	// Page 1: Command
	// 						  0x10			// reserved for future use
	ModeReg			Register		= 0x11 << 1 	// defines general modes for transmitting and receiving 
	TxModeReg		Register		= 0x12 << 1 	// defines transmission data rate and framing
	RxModeReg		Register		= 0x13 << 1 	// defines reception data rate and framing
	TxControlReg	Register		= 0x14 << 1 	// controls the logical behavior of the antenna driver pins TX1 and TX2
	TxASKReg		Register		= 0x15 << 1 	// controls the setting of the transmission modulation
	TxSelReg		Register		= 0x16 << 1 	// selects the internal sources for the antenna driver
	RxSelReg		Register		= 0x17 << 1 	// selects internal receiver settings
	RxThresholdReg	Register		= 0x18 << 1 	// selects thresholds for the bit decoder
	DemodReg		Register		= 0x19 << 1 	// defines demodulator settings
	// 						  0x1A			// reserved for future use
	// 						  0x1B			// reserved for future use
	MfTxReg			Register		= 0x1C << 1 	// controls some MIFARE communication transmit parameters
	MfRxReg			Register		= 0x1D << 1 	// controls some MIFARE communication receive parameters
	// 						  0x1E			// reserved for future use
	SerialSpeedReg	Register		= 0x1F << 1 	// selects the speed of the serial UART interface
	
	// Page 2: Configuration
	// 						  0x20			// reserved for future use
	CRCResultRegH	Register		= 0x21 << 1 	// shows the MSB and LSB values of the CRC calculation
	CRCResultRegL	Register		= 0x22 << 1 
	// 						  0x23			// reserved for future use
	ModWidthReg		Register		= 0x24 << 1 	// controls the ModWidth setting?
	// 						  0x25			// reserved for future use
	RFCfgReg		Register		= 0x26 << 1 	// configures the receiver gain
	GsNReg			Register		= 0x27 << 1 	// selects the conductance of the antenna driver pins TX1 and TX2 for modulation 
	CWGsPReg		Register		= 0x28 << 1 	// defines the conductance of the p-driver output during periods of no modulation
	ModGsPReg		Register		= 0x29 << 1 	// defines the conductance of the p-driver output during periods of modulation
	TModeReg		Register		= 0x2A << 1 	// defines settings for the internal timer
	TPrescalerReg	Register		= 0x2B << 1 	// the lower 8 bits of the TPrescaler value. The 4 high bits are in TModeReg.
	TReloadRegH		Register		= 0x2C << 1 	// defines the 16-bit timer reload value
	TReloadRegL		Register		= 0x2D << 1 
	TCounterValueRegH	Register	= 0x2E << 1 	// shows the 16-bit timer value
	TCounterValueRegL	Register	= 0x2F << 1 
	
	// Page 3: Test Registers
	// 						  0x30			// reserved for future use
	TestSel1Reg		Register		= 0x31 << 1 	// general test signal configuration
	TestSel2Reg		Register		= 0x32 << 1		// general test signal configuration
	TestPinEnReg	Register		= 0x33 << 1		// enables pin output driver on pins D1 to D7
	TestPinValueReg	Register		= 0x34 << 1 	// defines the values for D1 to D7 when it is used as an I/O bus
	TestBusReg		Register		= 0x35 << 1 	// shows the status of the internal test bus
	AutoTestReg		Register		= 0x36 << 1 	// controls the digital self-test
	VersionReg		Register		= 0x37 << 1 	// shows the software version
	AnalogTestReg	Register		= 0x38 << 1 	// controls the pins AUX1 and AUX2
	TestDAC1Reg		Register		= 0x39 << 1 	// defines the test value for TestDAC1
	TestDAC2Reg		Register		= 0x3A << 1 	// defines the test value for TestDAC2
	TestADCReg		Register		= 0x3B << 1		// shows the value of ADC I and Q channels
	// 						  0x3C			// reserved for production tests
	// 						  0x3D			// reserved for production tests
	// 						  0x3E			// reserved for production tests
	// 						  0x3F			// reserved for production tests
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
