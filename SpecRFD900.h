#ifndef SPECRFD900_H
#define SPECRFD900_H

#include "Drop.h"
#include "Settings.h"
#include <JohnnyKalman.h>
#include <SpecGPS.h>
#include <SpecBMP180.h>

namespace SpecRFD900 {
	unsigned long UpdateTimer = 0;
	const unsigned long UpdatePeriod = 10;

	long tLastRec = 0;
	bool hasSignal = false;

	int baudrate = 9600;
	HardwareSerial *RFD900;
	
	byte in[2];
	char targetLatBA[12];
	char targetLngBA[12];

	void setup(HardwareSerial *serial) {
		RFD900 = serial;
		RFD900->begin(baudrate);
	}
	
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
			
			// receive target from ground station
			// if (in[1] & 0b00010000) {
			// 	bool goodTransmission = true;
			// 	for (int i = 0; i < 22; i++) {
			// 		if (RFD900->available() > 0) {
			// 			if (i < 11) {
			// 				targetLatBA[i] = RFD900->read();
			// 			} else {
			// 				targetLngBA[i-11] = RFD900->read();
			// 			}
			// 		} else {
			// 			// abort
			// 			goodTransmission = false;
			// 			Serial.println("didn't get enough bytes for target");
			// 		}
			// 	}
			// 	targetLatBA[11] = '\0';
			// 	targetLngBA[11] = '\0';
				
			// 	float targetLat = atof(targetLatBA);
			// 	float targetLng = atof(targetLngBA);
				
			// 	// making sure it's somewhere near the US
			// 	if (targetLat < 22.7
			// 	 || targetLng < -115.2
			// 	 || targetLat > 46.8
			// 	 || targetLng > -64.5) {
			// 		goodTransmission = false;
			// 		Serial.println("target outside US");
			// 	}
				
			// 	if (goodTransmission) {
			// 		Settings::targetLongitude = targetLat;
			// 		Settings::targetLatitude = targetLng;
			// 		Settings::saveSettings();
			// 		JohnnyKalman::initial_kf_setup();
			// 		Serial.println("Saving incoming target" + String(targetLat) + " " + String(targetLng));
			// 	}
			// }
			
			// swap things if needed
			if (in[0] & 0b10000000) {
				byte temp = in[0];
				in[0] = in[1];
				in[1] = temp;
			}
			
			if (in[0] == 0) {
				//Serial.println("No ground telem");
			}
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
			} else {
				Drop::dropLHabs = false;
			}
			
			if (in[1] & 0b00000010) {
				Drop::dropRHabs = true;
			} else {
				Drop::dropRHabs = false;
			}
			
			if (in[0] & 0b00000001) {
				Drop::dropWater = true;
			} else {
				Drop::dropWater = false;
			}
			
			if (in[1] & 0b00100000) {
				#ifdef HASBMP
					bmp.resetOffset();
				#else
					SpecGPS::resetOffset();
				#endif
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