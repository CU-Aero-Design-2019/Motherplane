#ifndef DROP_H
#define DROP_H

#include "settings.h"
#include <SpecGPS.h>

namespace Drop {
	
	bool collectTarget = false;
	bool dropArmed = false;
	bool autoDrop = false;
	bool dropGlider1 = false;
	bool dropGlider2 = false;
	bool dropHabs = false;
	bool dropWater = false;
	
	const int waterUndropped = 45;
	const int waterDropped = 135;
	
	const int habsUndropped = 45;
	const int habsDropped = 135;
	
	const int glider1Undropped = 45;
	const int glider1Dropped = 135;
	
	const int glider2Undropped = 45;
	const int glider2Dropped = 135;
	
	double latSum = 0.0;
	double lngSum = 0.0;
	double altSum = 0.0;
	uint16_t nLocationSamples = 0;
	long lastSampleTime = 0;
	
	Servo waterServo;
	Servo habServo;
	Servo glider1Servo;
	Servo glider2Servo;
	
	void setup() {
		waterServo.attach(PA0);
		habServo.attach(PA1);
		glider1Servo.attach(PA8);
		glider2Servo.attach(PB1);
		
		waterServo.write(waterUndropped);
		habServo.write(habsUndropped);
		glider1Servo.write(glider1Undropped);
		glider2Servo.write(glider2Undropped);
	}
	
	void update() {
	
		if (dropArmed) {
			if (autoDrop) {
				
			} else {
				if (dropWater) {
					waterServo.write(waterDropped);
				} else {
					waterServo.write(waterUndropped);
				}
				if (dropHabs) {
					habServo.write(habsDropped);
				} else {
					habServo.write(habsUndropped);
				}
				if (dropGlider1) {
					glider1Servo.write(glider1Dropped);
				} else {
					glider1Servo.write(glider1Undropped);
				}
				if (dropGlider2) {
					glider2Servo.write(glider2Dropped);
				} else {
					glider2Servo.write(glider2Undropped);
				} 
			}
		} else {
			if (collectTarget) {
			//if (1) {
				// only collect every 100ms
				if (millis() - lastSampleTime > 100) {
					latSum += SpecGPS::gps.location.lat();
					lngSum += SpecGPS::gps.location.lng();
					altSum += SpecGPS::gps.altitude.meters();
					nLocationSamples++;
					// change the vars that are in memory
					Settings::targetLatitude = latSum / nLocationSamples;
					Settings::targetLongitude = lngSum / nLocationSamples;
					Settings::targetAltitude = altSum / nLocationSamples;
					
					Serial.print("Saving Lat: "); Serial.println(Settings::targetLatitude);
					Serial.print("Saving Lng: "); Serial.println(Settings::targetLongitude);
					
					// save those vars to eeprom
					Settings::saveSettings();
					
					lastSampleTime = millis();
				}
			} else {
				latSum = 0.0;
				lngSum = 0.0;
				altSum = 0.0;
				nLocationSamples = 0;
			}
		}
		
		// Serial.print("Tar " + String(collectTarget) + "\n");
		// Serial.print("Arm " + String(dropArmed) + ", ");
		// Serial.print("Auto " + String(autoDrop) + ", ");
		// Serial.print("G1 " + String(dropGlider1) + ", ");
		// Serial.print("G2 " + String(dropGlider2) + ", ");
		// Serial.print("H " + String(dropHabs) + ", ");
		// Serial.print("W " + String(dropWater) + ", ");
		
	}

}

#endif