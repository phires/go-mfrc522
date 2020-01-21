package helper

import (
	"log"
	"fmt"
)

func Log(method string, message string) {
	log.Printf("%20s | %s\n", method, message)
}

func DumpByteArray(values []byte) string {
	var result string
	for i:=0; i < len(values); i++ {
		result = result + fmt.Sprintf("%#02x ", values[i])
	}
	return result
}