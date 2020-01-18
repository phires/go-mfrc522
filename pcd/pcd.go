package pcd

import (
	"log"
	"time"
	"github.com/stianeikeland/go-rpio/v4"
)

// SendCommand sends a single command to MFRC522
func SendCommand(cmd Command) error {
	log.Printf("PCDSendCommand   | Command %s [%#02x]\n", commandByteToString(cmd), cmd)
	rpio.SpiTransmit(byte(CommandReg))				// MSB == 0 is for writing. LSB is not used in address. Datasheet section 8.1.2.3.
	rpio.SpiTransmit(byte(cmd))
	return nil
}

// WriteValueRegister writes a single byte to a register
func WriteValueRegister(reg Register, value byte) error {
	log.Printf("PCDWriteRegister | Register %s [%#02x] to value %#02x\n", registerByteToString(reg), reg, value)
	rpio.SpiTransmit(byte(reg))				// MSB == 0 is for writing. LSB is not used in address. Datasheet section 8.1.2.3.
	rpio.SpiTransmit(value)
	return nil
} 

// WriteValuesRegister Writes a number of bytes to the specified register in the MFRC522 chip.
// The interface is described in the datasheet section 8.1.2.
func WriteValuesRegister(reg Register, values []byte) error {
	var count int = len(values)
	log.Printf("PCDWriteRegister | Register %#02x | Length %d \n", reg, count)

	rpio.SpiTransmit(byte(reg))				// MSB == 0 is for writing. LSB is not used in address. Datasheet section 8.1.2.3.
	rpio.SpiTransmit(values...)
	
	return nil
} 

// ReadRegister reads a single byte from a register
func ReadRegister(reg Register) (byte, error) {
	buffer := []byte{ 0x80 | (byte(reg) & 0x7E) } 	// MSB == 1 is for reading. LSB is not used in address. Datasheet section 8.1.2.3.
	rpio.SpiExchange(buffer)		
	log.Printf("PCDReadRegister  | Register %s [%#02x] contains value %#02x\n", registerByteToString(reg), reg, buffer[0])	
	return buffer[0], nil
} 

// ClearRegisterBitMask  godoc
func ClearRegisterBitMask(reg Register, mask byte) error {
	tmp, err := ReadRegister(reg);
	if err != nil {
		return err
	}
	WriteValueRegister(reg, tmp & (^mask));		// clear bit mask
	return nil
} 

// SetRegisterBitMask  godoc
func SetRegisterBitMask(reg Register, mask byte) error {
	tmp, err := ReadRegister(reg);
	if err != nil {
		return err
	}
	WriteValueRegister(reg, tmp | mask);		
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
		v, err := ReadRegister(CommandReg)
		if !(v == 0x20 && count < 3) || err != nil  {
			break
		}
		count = count + 1
		time.Sleep(time.Millisecond*50)
	}
}

// AntennaOn turns on the antenna if it is disabled currently
func AntennaOn() {
	value, _ := ReadRegister(TxControlReg)
	log.Printf("AntennaOn        | Antenna status %#02x\n", value & 0x03)	
	if ((value & 0x03) != 0x03) {
		WriteValueRegister(TxControlReg, value | 0x03)
	}
}

// AntennaOff turns off the antenna
func AntennaOff() {
	ClearRegisterBitMask(TxControlReg, 0x03)
}


