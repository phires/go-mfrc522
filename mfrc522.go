package mfrc522

import (
	"github.com/stianeikeland/go-rpio/v4"
	"github.com/phires/go-mfrc522/pcd"
	"fmt"
	"time"

	log "github.com/sirupsen/logrus"
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
	keyByte	[6]byte
}



// SpiSlave PIN (CE0 = 8/24, CE1 = 7/26)
const (
	CE0 = iota
	CE1
	CE2
)

// Init sets the GPIO to SPI and resets the reader
func Init(spiSlave SpiSlave, logLevel log.Level) {
	log.SetLevel(logLevel)

	spiSlavePin = uint8(spiSlave)

	log.WithFields(log.Fields{
		"status": "Initializing",
		"spiSlavePin": spiSlavePin,
	}).Debug("Init")

	if err := rpio.Open(); err != nil {
		panic(err)
	}
	rpio.SpiSpeed(500000)
	resetPin = rpio.Pin(25)
	if err := rpio.SpiBegin(rpio.Spi0); err != nil {
		panic(err)
	}
	rpio.SpiChipSelect(spiSlavePin) // Select SPI slave

	hardReset := false
	resetPin.Mode(rpio.Input)
	r := rpio.ReadPin(resetPin)
	
	log.WithFields(log.Fields{
		"status": "Sending reset",
		"resetPinStatus": fmt.Sprintf("%#2x", byte(r)),
	}).Debug("Init")

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

	log.WithFields(log.Fields{
		"status": "Reset baud rates",
		"resetPinStatus": fmt.Sprintf("%#2x", byte(r)),
	}).Debug("Init")

	pcd.ResetBaudRates()
	pcd.AntennaOn()								// Enable the antenna driver pins TX1 and TX2 (they were disabled by the reset)
	log.WithFields(log.Fields{
		"status": "Initialization done",
	}).Info("Init")
}

// Dispose closes SPI
func Dispose() {
	log.WithFields(log.Fields{
		"status": "Closing SPI",
	}).Info("Dispose")

	rpio.SpiEnd(rpio.Spi0)
	rpio.Close()
}

// DumpVersionToLog get and dump the MFRC522 version
func DumpVersionToLog() {
	// Get the MFRC522 firmware version
	v, s := pcd.GetVersion()
	log.WithFields(log.Fields{
		"firmware_version": fmt.Sprintf("%#02x", v),
		"firmware": fmt.Sprintf("%s", s),
	}).Info("Init")
} 

//CalculateCRC godoc
func CalculateCRC(v []byte) ([]byte, error) {
	r, s := pcd.CalculateCRC(v)
	if s != pcd.StatusOK {
		return r, fmt.Errorf("error calculating CRC %v", s)
	}

	return r, nil
}

// SelfTest godoc
func SelfTest() {
	valid := pcd.SelfTest()
	log.WithFields(log.Fields{
		"result": valid,
	}).Info("SelfTest")
}


