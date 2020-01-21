package main

import (
	"github.com/phires/go-mfrc522"

	log "github.com/sirupsen/logrus"
)

func main() {
	mfrc522.Init(mfrc522.CE0, log.DebugLevel)
	defer mfrc522.Dispose()

	mfrc522.DumpVersionToLog()

	mfrc522.SelfTest()

}