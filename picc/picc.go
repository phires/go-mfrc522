package picc

import (
	"fmt"
	"time"
	"github.com/phires/go-mfrc522/pcd"

	log "github.com/sirupsen/logrus"
)

// ToCard sends a command to the card
func ToCard(command pcd.Command, sendData []byte) (pcd.StatusCode, []byte, byte) {
	log.WithFields(log.Fields{
		"command": command,
		"sendData": sendData,
	}).Debug("ToCard")

	var backData []byte
	var backLen byte

	var n byte
	var status pcd.StatusCode

	var irqEn byte = 0x00
	var waitIRq byte = 0x00

	if command == pcd.CommandMFAuthent {
		irqEn = 0x12
		waitIRq = 0x10
	}
	if command == pcd.CommandTransceive {
		irqEn = 0x77
		waitIRq = 0x30
	}

	pcd.WriteRegisterValue(pcd.ComIEnReg, irqEn | 0x80)
	pcd.ClearRegisterBitMask(pcd.ComIrqReg, 0x80)
	pcd.SetRegisterBitMask(pcd.FIFOLevelReg, 0x80)
	pcd.SendCommand(pcd.CommandIdle)

	pcd.WriteRegisterValues(pcd.FIFODataReg, sendData)
	pcd.SendCommand(command)

	if command == pcd.CommandTransceive {
		pcd.SetRegisterBitMask(pcd.BitFramingReg, 0x80)
	}

	var successful bool
	for i := 0; i <= 5; i++ {
		n, _ = pcd.ReadRegisterValue(pcd.ComIrqReg)
		if (^(n & 0x01) == 0x00) && (^(n & waitIRq) == 0x00) {
			successful = true
			break
		}
		time.Sleep(50*time.Millisecond)
	}

	pcd.ClearRegisterBitMask(pcd.BitFramingReg, 0x80)

	if !successful {
		return pcd.StatusError, backData, backLen
	}
	v, err := pcd.ReadRegisterValue(pcd.ErrorReg) 
	log.WithFields(log.Fields{
		"error": err,
	}).Error("ToCard")

	if v & 0x1B != 0x00 {
		return pcd.StatusError, backData, backLen
	}

	status = pcd.StatusOK

	if (n & irqEn & 0x01) == 0x00 {
		status = pcd.StatusMINotAgErr
	}

	if command == pcd.CommandTransceive {
		n, _ = pcd.ReadRegisterValue(pcd.FIFOLevelReg)
		lastBits, err := pcd.ReadRegisterValue(pcd.ControlReg) 
		lastBits &= 0x07

		log.WithFields(log.Fields{
			"error": err,
		}).Error("ToCard")

		if lastBits != 0 {
			backLen = (n - 1) * 8 + lastBits
		} else {
			backLen = n * 8
		}

		log.WithFields(log.Fields{
			"backLen": backLen,
		}).Debug("ToCard")

		if n == 0 {
			n = 1
		}

		if n > MaxLength {
			n = MaxLength
		}
		var i byte
		for i = 0; i < n; i++ {
			v, err := pcd.ReadRegisterValue(pcd.FIFODataReg)
			log.WithFields(log.Fields{
				"error": err,
			}).Error("ToCard")
			backData = append(backData, v)
		}
	}
	return status, backData, backLen
}

// Request godoc
func Request(reqMode byte) (byte, error) {
   var tagType []byte

	pcd.WriteRegisterValue(pcd.BitFramingReg, 0x07)
    
	tagType = append(tagType, reqMode)
	
	status, _, backBits := ToCard(pcd.CommandTransceive, tagType)

	if (status != pcd.StatusOK) || (backBits != 0x10) {
		return 0x00, fmt.Errorf("error sending %v to card (backBits=%v)", tagType, backBits)
	}
    return backBits, nil
}

// Anticoll godoc
func Anticoll() ([]byte, error) {
	var serNumCheck byte = 0
	serNum := make([]byte, 2)

	err := pcd.WriteRegisterValue(pcd.BitFramingReg, 0x00)
	if err != nil {
		return nil, err
	}

	serNum[0] = byte(CMDSelCL1) // anticoll command
	serNum[1] = 0x20

	status, backData, _ := ToCard(pcd.CommandTransceive, serNum)

	if status == pcd.StatusOK {
		if len(backData) == 5 {
			for i:=0; i<4; i++ {
				serNumCheck = serNumCheck ^ backData[i]
			}
			if serNumCheck != backData[4] {
				return nil, fmt.Errorf("serNumCheck != backData[4]  -- %v != %v", serNumCheck, backData)
			}
		} else {
			return nil, fmt.Errorf("len(backData) != 5  -- %v != 5", len(backData))
		}
	}
	return backData, nil
}

// UIDtoNum converts a Byte Array UID to a number
func UIDtoNum(uid []byte) int {
    var n int
    for i := 0; i < 5; i++ {
		n = n * 256 + int(uid[i])
	}
	return n
}