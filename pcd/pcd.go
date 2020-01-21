package pcd

import (
	"fmt"
	"time"
	"bytes"
	"github.com/stianeikeland/go-rpio/v4"
	"github.com/phires/go-mfrc522/helper"

	log "github.com/sirupsen/logrus"
)

// SendCommand sends a single command to MFRC522
func SendCommand(cmd Command) error {
	log.WithFields(log.Fields{
		"cmd": fmt.Sprintf("%#02x", cmd),
		"name": commandByteToString(cmd),
	}).Trace("SendCommand")

	data := []byte{byte(CommandReg) << 1, byte(cmd)}
	if err := rpio.SpiBegin(rpio.Spi0); err != nil {
		panic(err)
	}
	rpio.SpiTransmit(data...)
	rpio.SpiEnd(rpio.Spi0)

	return nil
}

// WriteRegisterValue writes a single byte to a register
func WriteRegisterValue(reg Register, value byte) error {
	data := []byte{byte(reg) << 1, value}		// Register needs to be shifted by 1 when using SPI
	log.WithFields(log.Fields{
		"register": fmt.Sprintf("%#02x", reg),
		"name": registerByteToString(reg),
		"value": fmt.Sprintf("%#02x", value),
	}).Trace("WriteRegisterValue")

	
	if err := rpio.SpiBegin(rpio.Spi0); err != nil {
		panic(err)
	}
	rpio.SpiTransmit(data...)
	rpio.SpiEnd(rpio.Spi0)

	return nil
} 

// WriteRegisterValues Writes a number of bytes to the specified register in the MFRC522 chip.
// The interface is described in the datasheet section 8.1.2.
func WriteRegisterValues(reg Register, values []byte) error {
	var count int = len(values)
	data := make([]byte, 1)
	data[0] = byte(reg) << 1
	data = append(data, values...)

	log.WithFields(log.Fields{
		"register": fmt.Sprintf("%#02x", reg),
		"name": registerByteToString(reg),
		"count": count,
		"values": helper.DumpByteArray(data),
	}).Trace("WriteRegisterValues")
	
	if err := rpio.SpiBegin(rpio.Spi0); err != nil {
		panic(err)
	}
	rpio.SpiTransmit(data...)
	rpio.SpiEnd(rpio.Spi0)

	return nil
} 

// ReadRegisterValue reads a single byte from a register
func ReadRegisterValue(reg Register) (byte, error) {
	var r byte = byte(reg) << 1						// Register needs to be shifted by 1 when using SPI
	buffer := []byte{ 0x80 | (r & 0x7E) } 	// MSB == 1 is for reading. LSB is not used in address. Datasheet section 8.1.2.3.
	
	if err := rpio.SpiBegin(rpio.Spi0); err != nil {
		panic(err)
	}
	rpio.SpiTransmit(buffer...)
	v := rpio.SpiReceive(1)
	rpio.SpiEnd(rpio.Spi0)

	log.WithFields(log.Fields{
		"register": fmt.Sprintf("%#02x", reg),
		"name": registerByteToString(reg),
		"value": fmt.Sprintf("%#02x", v[0]),
	}).Trace("ReadRegisterValue")

	return v[0], nil
} 

// ReadRegisterValues reads a number of bytes from the specified register in the MFRC522 chip.
// The interface is described in the datasheet section 8.1.2.
func ReadRegisterValues(reg Register, count uint8, rxAlign byte) ([]byte, error) {
	values := make([]byte, count)
	var v []byte
	if (count == 0) {
		return values, nil
	}
	log.WithFields(log.Fields{
		"register": fmt.Sprintf("%#02x", reg),
		"name": registerByteToString(reg),
		"count": count,
		"rxAlign": fmt.Sprintf("%#02x", rxAlign),
	}).Trace("ReadRegisterValues")
	address := 0x80 | (byte(reg) << 1)			// MSB == 1 is for reading. LSB is not used in address. Datasheet section 8.1.2.3.
	var index uint8 = 0							// Index in values array.

	if err := rpio.SpiBegin(rpio.Spi0); err != nil {
		panic(err)
	}
	//count--;								// One read is performed outside of the loop
	//rpio.SpiTransmit(address)				// Tell MFRC522 which address we want to read
	if (rxAlign > 0x00) {		// Only update bit positions rxAlign..7 in values[0]
		// Create bit mask for bit positions rxAlign..7
		var mask byte = (0xFF << rxAlign) & 0xFF
		// Read value and tell that we want to read the same address again.
		rpio.SpiTransmit(address)
		v = rpio.SpiReceive(1)
		
		// Apply mask to both current value of values[0] and the new data in value.
		values[0] = (values[0] & ^mask) | (v[0] & mask)
		index = index + 1
	}
	
	for ; index < count; index++ {
		rpio.SpiTransmit(address)
		v = rpio.SpiReceive(1)
		values[index] = v[0]
	}
	
	rpio.SpiTransmit(0x0)

	rpio.SpiEnd(rpio.Spi0)
	return values, nil
} 

// ClearRegisterBitMask  godoc
func ClearRegisterBitMask(reg Register, mask byte) error {
	tmp, err := ReadRegisterValue(reg);
	if err != nil {
		return err
	}
	WriteRegisterValue(reg, tmp & (^mask));		// clear bit mask
	return nil
} 

// SetRegisterBitMask  godoc
func SetRegisterBitMask(reg Register, mask byte) error {
	tmp, err := ReadRegisterValue(reg);
	if err != nil {
		return err
	}
	WriteRegisterValue(reg, tmp | mask);		
	return nil
} 

// Reset issues the SoftReset command.
func Reset() {
	SendCommand(CommandSoftReset)
	// The datasheet does not mention how long the SoftRest command takes to complete.
	// But the MFRC522 might have been in soft power-down mode (triggered by bit 4 of CommandReg) 
	// Section 8.8.2 in the datasheet says the oscillator start-up time is the start up time of the crystal + 37,74μs. Let us be generous: 50ms.
	var count uint8

	for {
		time.Sleep(time.Millisecond*50)

		v, err := ReadRegisterValue(CommandReg)
		if ((v & (1<<4) == 0) && count < 3) || err != nil  {
			break
		}
		count = count + 1
		
	}
}

func ResetBaudRates() {
	WriteRegisterValue(TxModeReg, 0x00)
	WriteRegisterValue(RxModeReg, 0x00)
	WriteRegisterValue(ModWidthReg, 0x26)

	WriteRegisterValue(TModeReg, 0x8D)	
	WriteRegisterValue(TPrescalerReg, 0x3E)	
	WriteRegisterValue(TReloadRegL, 30)
	WriteRegisterValue(TReloadRegH, 0)	
	
	WriteRegisterValue(TxASKReg, 0x40)		// Default 0x00. Force a 100 % ASK modulation independent of the ModGsPReg register setting
	WriteRegisterValue(ModeReg, 0x3D)		// Default 0x3F. Set the preset value for the CRC coprocessor for the CalcCRC command to 0x6363 (ISO 14443-3 part 6.2.4)
}

// AntennaOn turns on the antenna if it is disabled currently
func AntennaOn() {
	value, _ := ReadRegisterValue(TxControlReg)
	log.WithFields(log.Fields{
		"status": fmt.Sprintf("%#02x", value),
	}).Trace("AntennaOn")

	if ((value & 0x03) != 0x03) {
		log.WithFields(log.Fields{
			"action": "turnon",
		}).Trace("AntennaOn")
		WriteRegisterValue(TxControlReg, value | 0x03)
	}
}

// AntennaOff turns off the antenna
func AntennaOff() {
	log.WithFields(log.Fields{
		"action": "turnoff",
	}).Trace("AntennaOff")
	ClearRegisterBitMask(TxControlReg, 0x03)
}

// GetVersion godoc
func GetVersion() (byte, string) {
	v, _ := ReadRegisterValue(VersionReg);
	var result string
	// Lookup which version
	switch(v) {
	case 0x88:
		result = "clone"
		break
	case 0x90: 
		result = "v0.0"
		break
	case 0x91: 
		result = "v1.0"
		break
	case 0x92:
		result = "v2.0"
		break
	case 0x12: 
		result = "counterfeit chip"
		break
	default:
		result = "unknown"
	}
	// When 0x00 or 0xFF is returned, communication probably failed
	if ((v == 0x00) || (v == 0xFF)) {
		result = "WARNING: Communication failure, is the MFRC522 properly connected?"
	}

	return v, result
}

// CalculateCRC Use the CRC coprocessor in the MFRC522 to calculate a CRC_A.
func CalculateCRC(data []byte) ([]byte, StatusCode) {
	log.WithFields(log.Fields{
		"data": helper.DumpByteArray(data),
	}).Debug("CalculateCRC")
	
	SendCommand(CommandIdle)

	WriteRegisterValue(DivIrqReg, 0x04)				// Clear the CRCIRq interrupt request bit
	SetRegisterBitMask(FIFOLevelReg, 0x80)			// FlushBuffer = 1, FIFO initialization
	WriteRegisterValues(FIFODataReg, data)			// Write data to the FIFO

	SendCommand(CommandCalcCRC)

	c1 := make(chan []byte, 1)
    go func() {
		result := make([]byte, 2)
		for {
			n, _ := ReadRegisterValue(DivIrqReg)
			if n == 0x04 {							// CRCIRq bit set - calculation done
				SendCommand(CommandIdle)					// Stop calculating CRC for new content in the FIFO.
				// Transfer the result from the registers to the result buffer
				result[0], _ = ReadRegisterValue(CRCResultRegL)
				result[1], _ = ReadRegisterValue(CRCResultRegH)
				c1 <- result;
			}
			time.Sleep(time.Millisecond*25)
		}
    }()
	select {
	case res := <-c1:
		log.WithFields(log.Fields{
			"result": helper.DumpByteArray(res),
		}).Debug("CalculateCRC")

        return res, StatusOK
    case <-time.After(time.Millisecond*250):		// Wait 100ms for CRC calc to finish
		return nil, StatusTimeout
    }
} 
/*
func RandomID()  ([]byte, StatusCode) {
	SendCommand(CommandIdle)				// Stop any active command.
	WriteRegisterValue(DivIrqReg, 0x04)		// Clear the CRCIRq interrupt request bit
	SetRegisterBitMask(FIFOLevelReg, 0x80)	// FlushBuffer = 1, FIFO initialization

	SendCommand(CommandGenerateRandomID)	// Create random id
	time.Sleep(time.Millisecond*100)		// give some time
	SendCommand(CommandMem)					// Move internal buffer to FIFO


}
*/

// SelfTest follows directly the steps outlined in 16.1.1
func SelfTest() bool {
	// 1. Perform a soft reset.
	Reset()
	
	// 2. Clear the internal buffer by writing 25 bytes of 00h
	zeroes := make([]byte, 25)
	WriteRegisterValue(FIFOLevelReg, 0x80)			// flush the FIFO buffer
	WriteRegisterValues(FIFODataReg, zeroes)		// write 25 bytes of 00h to FIFO
	SendCommand(CommandMem)						// transfer to internal buffer
	
	// 3. Enable self-test
	WriteRegisterValue(AutoTestReg, 0x09)
	
	// 4. Write 00h to FIFO buffer
	WriteRegisterValue(FIFODataReg, 0x00)
	
	// 5. Start self-test by issuing the CalcCRC command
	SendCommand(CommandCalcCRC)
	
	// 6. Wait for self-test to complete
	var n byte
	for i := 0; i < 0xFF; i++ {
		time.Sleep(time.Millisecond*50)		// Give the CRC computation some time
		// The datasheet does not specify exact completion condition except
		// that FIFO buffer should contain 64 bytes.
		// While selftest is initiated by CalcCRC command
		// it behaves differently from normal CRC computation,
		// so one can't reliably use DivIrqReg to check for completion.
		// It is reported that some devices does not trigger CRCIRq flag
		// during selftest.
		n, _ = ReadRegisterValue(FIFOLevelReg)
		if (n >= 64) {
			break
		}
	}
	SendCommand(CommandIdle);		// Stop calculating CRC for new content in the FIFO.
	
	// 7. Read out resulting 64 bytes from the FIFO buffer.
	result, _ := ReadRegisterValues(FIFODataReg, 64, 0)

	// Auto self-test done
	// Reset AutoTestReg register to be 0 again. Required for normal operation.
	WriteRegisterValue(AutoTestReg, 0x00)
	
	
	// Determine firmware version (see section 9.3.4.8 in spec)
	version, _ := ReadRegisterValue(VersionReg)
	
	// Pick the appropriate reference values
	var valid int
	switch (version) {
		case 0x88:	// Fudan Semiconductor FM17522 clone
			valid = bytes.Compare(FM17522FirmwareReference, result)
			break;
		case 0x90:	// Version 0.0
			valid = bytes.Compare(FirmwareReferenceV0_0, result)
			break;
		case 0x91:	// Version 1.0
			valid = bytes.Compare(FirmwareReferenceV1_0, result)
			break;
		case 0x92:	// Version 2.0
			valid = bytes.Compare(FirmwareReferenceV2_0, result)
			break;
		default:	// Unknown version
			return false; // abort test
	}
	
	// Test passed; all is good.
	return valid == 0;
}

// SoftPowerDown godoc
func SoftPowerDown() {
	//Note : Only soft power down mode is available throught software
	val, _  := ReadRegisterValue(CommandReg) 	// Read state of the command register 
	val |= (1<<4)								// set PowerDown bit ( bit 4 ) to 1 
	WriteRegisterValue(CommandReg, val)			//write new value to the command register
}

// SoftPowerUp godoc
func SoftPowerUp(){
	val, _ := ReadRegisterValue(CommandReg) 	// Read state of the command register 
	val -= (1<<4)							// set PowerDown bit ( bit 4 ) to 0 
	WriteRegisterValue(CommandReg, val)			// write new value to the command register
	// wait until PowerDown bit is cleared (this indicates end of wake up procedure) 
	c1 := make(chan bool, 1)
	
	go func() {
		for {
			n, _ := ReadRegisterValue(CommandReg)
			if n != 0x04 {							// CRCIRq bit set - calculation done
				SendCommand(CommandIdle)					// Stop calculating CRC for new content in the FIFO.
				c1 <- true;
			}
			time.Sleep(time.Millisecond*25)
		}
    }()
	select {
	case <-c1:
	    return 
    case <-time.After(time.Millisecond*500):		// Wait 500ms for CRC calc to finish
		return
    }
}


// CommunicateWithPICC transfers data to the MFRC522 FIFO, executes a command, waits for completion and transfers data back from the FIFO.
// CRC validation can only be done if backData and backLen are specified.
func CommunicateWithPICC(command Command, waitIRq byte, sendData []byte, validBits *byte, rxAlign byte, backLen byte, checkCRC bool) ([]byte, StatusCode, error) {
	// Prepare values for BitFramingReg
	var result []byte

	var txLastBits byte 
	if *validBits > 0 {
		txLastBits = *validBits
	}

	bitFraming := (rxAlign << 4) + txLastBits		// RxAlign = BitFramingReg[6..4]. TxLastBits = BitFramingReg[2..0]

	SendCommand(CommandIdle)						// Stop any active command.
	WriteRegisterValue(ComIrqReg, 0x7F)				// Clear all seven interrupt request bits
	WriteRegisterValue(FIFOLevelReg, 0x80)			// FlushBuffer = 1, FIFO initialization
	WriteRegisterValues(FIFODataReg, sendData)		// Write sendData to the FIFO
	WriteRegisterValue(BitFramingReg, bitFraming)	// Bit adjustments
	WriteRegisterValue(CommandReg, byte(command))	// Execute the command

	if command == CommandTransceive {
		SetRegisterBitMask(BitFramingReg, 0x80)		// StartSend=1, transmission of data starts
	}

	// Wait for the command to complete.
	// In PCD_Init() we set the TAuto flag in TModeReg. This means the timer automatically starts when the PCD stops transmitting.
	// Each iteration of the do-while-loop takes 17.86μs.
	
	c1 := make(chan byte, 1)
	go func() {
		for {
			n, _ := ReadRegisterValue(ComIrqReg)	// ComIrqReg[7..0] bits are: Set1 TxIRq RxIRq IdleIRq HiAlertIRq LoAlertIRq ErrIRq TimerIRq
			if (n & waitIRq) == 0 || (n & 0x01) == 0 {		// One of the interrupts that signal success has been set or TimerIRq
				c1 <- n
			}
			time.Sleep(time.Millisecond*25)
		}
	}()
	
	select {
	case res := <-c1:
		log.WithFields(log.Fields{
			"result": res,
		}).Debug("CommunicateWithPICC")
		if res == 0x01 {						// TimerIRq
			return result, StatusTimeout, nil
		}

		break
	case <-time.After(time.Millisecond*50):		// Wait 50ms to finish
		return result, StatusTimeout, nil
    }

	// Stop now if any errors except collisions were detected.
	errorRegValue, _ := ReadRegisterValue(ErrorReg); // ErrorReg[7..0] bits are: WrErr TempErr reserved BufferOvfl CollErr CRCErr ParityErr ProtocolErr
	if errorRegValue == 0x13 {	 // BufferOvfl ParityErr ProtocolErr
		return result, StatusError, nil
	}

	// If the caller wants data back, get it from the MFRC522.
	if backLen > 0 {
		n, _ := ReadRegisterValue(FIFOLevelReg);	// Number of bytes in the FIFO

		if n > backLen {
			return result, StatusNoRoom, nil
		}

		result, _ = ReadRegisterValues(FIFODataReg, n, rxAlign)	// Get received data from FIFO
		vb, _ := ReadRegisterValue(ControlReg)			// RxLastBits[2:0] indicates the number of valid bits in the last received byte. If this value is 000b, the whole byte is valid.
		*validBits = vb & 0x07
	}

	// Tell about collisions
	if errorRegValue == 0x08 {		// CollErr
		return result, StatusCollision, nil
	}

	// Perform CRC_A validation if requested.
	if len(result) > 0 && backLen > 0 && checkCRC {
		// In this case a MIFARE Classic NAK is not OK.
		if (backLen == 1 && *validBits == 0x4) {
			return result, StatusMifareNACK, nil
		}
		// We need at least the CRC_A value and all 8 bits of the last byte must be received.
		if (backLen < 2 || *validBits != 0x0) {
			return result, StatusCRCWrong, nil
		}
		// Verify CRC_A - do our own calculation and store the control in controlBuffer.
		crc, status := CalculateCRC(result[:len(result)-2])
		if status != StatusOK {
			return result, status, nil
		}
		if result[len(result) - 2] != crc[0] || result[len(result) - 1] != crc[1] {
			return result, StatusCRCWrong, nil
		}
	}
	return result, StatusOK, nil
}