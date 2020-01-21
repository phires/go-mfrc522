package main

import (
	"github.com/phires/go-mfrc522"
	"fmt"

	log "github.com/sirupsen/logrus"
)

func main() {
	mfrc522.Init(mfrc522.CE0, log.TraceLevel)
	defer mfrc522.Dispose()

	mfrc522.DumpVersionToLog()

	//mfrc522.RandomID()
	
	// Test: calc crc
	
	v := []byte{ 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F }
	r, err := mfrc522.CalculateCRC(v)
	if err != nil {
		fmt.Printf("CRC failed for %v : %v\n", v, r)
	} else {
		fmt.Printf("CRC for %v : %v\n", v, r)
	}
	
	//mfrc522.SelfTest()
	
}
