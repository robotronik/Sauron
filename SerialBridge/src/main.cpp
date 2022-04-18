#include <Arduino.h>
#include "data/SerialPacket.hpp"
#include "SerialReceiver.hpp"

void setup() {
	Serial.begin(SerialTransmission::BaudRate);
	
	StartSerialReceiver();
	vTaskDelete(NULL);
}

void loop() {
	// put your main code here, to run repeatedly:
}