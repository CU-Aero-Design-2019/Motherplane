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
	bool dropLHabs = false;
	bool dropRHabs = false;
	bool dropWater = false;
	
	bool droppedGlider1 = false;
	bool droppedGlider2 = false;
	bool droppedLHabs = false;
	bool droppedRHabs = false;
	bool droppedWater = false;
	byte sendBack = 0;
	
	//srvo 0 
	int waterUndropped = 70;
	int waterDropped = 115;
	
	//srvo 1 
	int habsLUndropped = 50;
	int habsLDropped = 150;

	//srvo 2 
	int habsRUndropped = 50;
	int habsRDropped = 150;
	
	//srvo 3 
	int glider1Undropped = 0;
	int glider1Dropped = 130;
	
	//srvo 4 
	int glider2Undropped = 130;
	int glider2Dropped = 0;
	
	double latSum = 0.0;
	double lngSum = 0.0;
	double altSum = 0.0;
	uint16_t nLocationSamples = 0;
	long lastSampleTime = 0;

	bool manualServo = false;
	
	Servo waterServo;
	Servo habLServo;
	Servo habRServo;
	Servo glider1Servo;
	Servo glider2Servo;
	
	void setup() {
		waterServo.attach(PA0);
		habLServo.attach(PA1);
		habRServo.attach(PB9);
		glider1Servo.attach(PA8);
		glider2Servo.attach(PB1);
		
		waterServo.write(waterUndropped);
		habLServo.write(habsLUndropped);
		habRServo.write(habsRUndropped);
		glider1Servo.write(glider1Undropped);
		glider2Servo.write(glider2Undropped);
	}

	void manuallySet(int servo, int value) {

		Serial.println("Setting servo " + String(servo) + " to " + String(value));

		switch(servo){
			case 0:
				waterServo.write(value);
				break;
			case 1:
				habLServo.write(value);
				break;
			case 2:
				habRServo.write(value);
				break;
			case 3:
				glider1Servo.write(value);
				break;
			case 4:
				glider2Servo.write(value);
				break;
		}

	}
	
	void update() {
	
		if (dropArmed && !manualServo) {
			if (autoDrop) {
				
			} else {
				if (dropWater) {
					waterServo.write(waterDropped);
					droppedWater = true;
				} else {
					waterServo.write(waterUndropped);
					droppedWater = false;
				}
				if (dropLHabs) {
					habLServo.write(habsLDropped);
					droppedLHabs = true;
				} else {
					habLServo.write(habsLUndropped);
					droppedLHabs = false;
				}
				if (dropRHabs) {
					habRServo.write(habsRDropped);
					droppedRHabs = true;
				} else {
					habLServo.write(habsRUndropped);
					droppedRHabs = false;
				}
				if (dropGlider1) {
					glider1Servo.write(glider1Dropped);
					droppedGlider1 = true;
				} else {
					glider1Servo.write(glider1Undropped);
					droppedGlider1 = false;
				}
				if (dropGlider2) {
					glider2Servo.write(glider2Dropped);
					droppedGlider2 = true;
				} else {
					glider2Servo.write(glider2Undropped);
					droppedGlider2 = false;
				} 
			}
		} else {
			if (collectTarget) {
			//if (1) {
				// only collect every 100ms
				if (millis() - lastSampleTime > 100) {
					if (SpecGPS::gps.location.lat() != 0.0) {
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
				}
			} else {
				latSum = 0.0;
				lngSum = 0.0;
				altSum = 0.0;
				nLocationSamples = 0;
			}
		}
		
		// update telemetry values
		sendBack = 0;
		if (droppedGlider1) {
			sendBack |= 0b00001000;
		}
		if (droppedGlider2) {
			sendBack |= 0b00000100;
		}
		if (droppedLHabs) {
			sendBack |= 0b00000010;
		}
		if (droppedWater) {
			sendBack |= 0b00000001;
		}
		
		// Serial.print("Tar " + String(collectTarget) + ", ");
		// Serial.print("Arm " + String(dropArmed) + ", ");
		// Serial.print("Auto " + String(autoDrop) + ", ");
		// Serial.print("G1 " + String(dropGlider1) + ", ");
		// Serial.print("G2 " + String(dropGlider2) + ", ");
		// Serial.print("H " + String(dropHabs) + ", ");
		// Serial.print("W " + String(dropWater) + ", ");
		
	}

}

#endif