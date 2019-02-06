#ifndef SPECRFD900_H
#define SPECRFD900_H

#include "Drop.h"

namespace SpecRFD900 {
	unsigned long UpdateTimer = 0;
	const unsigned long UpdatePeriod = 10;

	long tLastRec = 0;
	bool hasSignal = false;

	int baudrate = 9600;
	HardwareSerial *RFD900;
	
	byte in[2];

	void setup(HardwareSerial *serial) {
		RFD900 = serial;
		RFD900->begin(baudrate);
	}

	// void send(String str) {
		// RFD900->println(str);
	// }
	
	void sendTelemetry(String data) {
		const char* charData = data.c_str();
		RFD900->write(charData, data.length());
	}
	
	// to be called at a regular interval
	void update() {
		if (RFD900->available()) {
			tLastRec = millis();
			// flush out if we have too many things
			while (RFD900->available() > 2) RFD900->read();
			
			in[0] = RFD900->read();
			in[1] = RFD900->read();
			
			// swap things if needed
			if (in[0] & 0b10000000) {
				byte temp = in[0];
				in[0] = in[1];
				in[1] = temp;
			}
			
			if (in[0] == 0) {
				Serial.println("No ground telem");
			}
			//Serial.println("Getting RFD data");
			if (in[0] & 0b10000000) {
				Drop::collectTarget = true;
			} else {
				Drop::collectTarget = false;
			}

			if (in[0] & 0b01000000) {
				hasSignal = true;
			} else {
				hasSignal = false;
			}
			
			if (in[0] & 0b00100000) {
				Drop::dropArmed = true;
			} else {
				Drop::dropArmed = false;
			}
			
			if (in[0] & 0b00010000) {
				Drop::autoDrop = true;
			} else {
				Drop::autoDrop = false;
			}
			
			if (in[0] & 0b00001000) {
				Drop::dropGlider1 = true;
			} else {
				Drop::dropGlider1 = false;
			}
			
			if (in[0] & 0b00000100) {
				Drop::dropGlider2 = true;
			} else {
				Drop::dropGlider2 = false;
			}
			
			if (in[0] & 0b00000010) {
				Drop::dropLHabs = true;
				Drop::dropRHabs = true;
			} else {
				Drop::dropLHabs = false;
				Drop::dropRHabs = false;
			}
			
			if (in[0] & 0b00000001) {
				Drop::dropWater = true;
			} else {
				Drop::dropWater = false;
			}
			//Serial.println(in);
			Drop::update();
		}
		if(tLastRec > 1000){
			in[0] = 0;
		}
	}

}; 
// namespace SpecRFD900

#endif