#ifndef SPECRFD900_H
#define SPECRFD900_H

#include "Drop.h"

namespace SpecRFD900 {
	unsigned long UpdateTimer = 0;
	const unsigned long UpdatePeriod = 10;

	int baudrate = 9600;
	HardwareSerial *RFD900;
	
	byte in;

	void setup(HardwareSerial *serial) {
		RFD900 = serial;
		RFD900->begin(baudrate);
	}

	void send(String str) {
		RFD900->println(str);
	}

	// does stuff with the incoming string
	// should be called from update()
	void parse() {
		if (in == 0) {
			Serial.println("No ground telem");
		}
		//Serial.println("Getting RFD data");
		if (in & 0b10000000) {
			Drop::collectTarget = true;
		} else {
			Drop::collectTarget = false;
		}
		
		if (in & 0b00100000) {
			Drop::dropArmed = true;
		} else {
			Drop::dropArmed = false;
		}
		
		if (in & 0b00010000) {
			Drop::autoDrop = true;
		} else {
			Drop::autoDrop = false;
		}
		
		if (in & 0b00001000) {
			Drop::dropGlider1 = true;
		} else {
			Drop::dropGlider1 = false;
		}
		
		if (in & 0b00000100) {
			Drop::dropGlider2 = true;
		} else {
			Drop::dropGlider2 = false;
		}
		
		if (in & 0b00000010) {
			Drop::dropHabs = true;
		} else {
			Drop::dropHabs = false;
		}
		
		if (in & 0b00000001) {
			Drop::dropWater = true;
		} else {
			Drop::dropWater = false;
		}
		//Serial.println(in);
		Drop::update();
	}
	
	// to be called at a regular interval
	void update() {
		if (RFD900->available()) {
			while (RFD900->available()) {
				in = RFD900->read();
			}
			parse();
		}
	}

	void sendTelemetry(String data) {
		const char* charData = data.c_str();
		RFD900->write(charData, data.length());
	}

}; 
// namespace SpecRFD900

#endif