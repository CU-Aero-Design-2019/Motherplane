#ifndef DROP_H
#define DROP_H

#include "settings.h"
#include <SpecGPS.h>
#include "Prediction.h"

extern SpecBMP180 bmp;

namespace Drop {

	const int targetRadius = 50;
	
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

	bool autoDropWater = false;
	bool autoDropHabs = false;
	bool autoDropGliders = false;

	bool switchControl = false;
	
	byte sendBack = 0;
	
	#ifdef DEIMOS
	//srvo 0
	int waterUndropped = 90;
	int waterDropped = 50;
	
	//srvo 1 
	int habsLUndropped = 50;
	int habsLDropped = 130;

	//srvo 2 
	int habsRUndropped = 50;
	int habsRDropped = 130;
	
	//srvo 3 
	int glider1Undropped = 150;
	int glider1Dropped = 20;
	
	//srvo 4 
	int glider2Undropped = 20;
	int glider2Dropped = 150;
	
	#else
	//srvo 0
	int waterUndropped = 90;
	int waterDropped = 50;
	
	//srvo 1 
	int habsLUndropped = 50;
	int habsLDropped = 130;

	//srvo 2 
	int habsRUndropped = 50;
	int habsRDropped = 130;
	
	//srvo 3 
	int glider1Undropped = 130;
	int glider1Dropped = 0;
	
	//srvo 4 
	int glider2Undropped = 0;
	int glider2Dropped = 130;
	
	#endif
	
	double latSum = 0.0;
	double lngSum = 0.0;
	double altSum = 0.0;
	uint16_t nLocationSamples = 0;
	long lastSampleTime = 0;

	bool manualServo = false;

	long lastTime = 0;

	float lastHVE;
	float lastHVN;
	
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

	void updateAuto(bool habitat = false) {

		float course = SpecGPS::ubg.getMotionHeading_deg();
		// float course = 90;
		float speed = SpecGPS::ubg.getGroundSpeed_ms();
		// float speed = 30;
		float alt = bmp.getKAlt();
		// float alt = 35;

		double curPredE;
		double curPredN;
		if (habitat) {
			curPredE = Prediction::habPrediction.e;
			curPredN = Prediction::habPrediction.n;
		} else {
			curPredE = Prediction::watPrediction.e;
			curPredN = Prediction::watPrediction.n;
		}

		// quit early if we're outside target
		float distFromTarget = sqrt(curPredN * curPredN + curPredE * curPredE);
		if (distFromTarget > targetRadius){
			return;
		}

		// find sin and cos of heading for later
        float sn = sin(course * 0.0174533);
        float cs = cos(course * 0.0174533);

        // heading vector
        float hvE = sn;
        float hvN = cs;

        // find estimated next drop location
		float delT = (millis() - lastTime)/1000;
		float nextE = curPredE + speed * delT * lastHVE;
		float nextN = curPredN + speed * delT * lastHVN;
        
        // rotate 90 degrees to get perp line
        float tvE = -lastHVN;
        float tvN = lastHVE;
                
        double slope = tvE / tvN;
        
        if (nextE > nextN * slope && alt > 100*0.3048) {
            if (habitat) {
            	autoDropHabs = true;
            } else {
            	autoDropWater = true;
            }
        }

        lastHVN = hvN;
        lastHVE = hvE;

        lastTime = millis();
	}
	
	void update() {
//		if (switchControl) {
//			Serial.println("Switch Control");
//			if (digitalRead(PA10) == LOW) {
//				// open all
//				waterServo.write(waterDropped);
//				habLServo.write(habsLDropped);
//				habRServo.write(habsRDropped);
//				glider1Servo.write(glider1Dropped);
//				glider2Servo.write(glider2Dropped);
//			} else {
//				// close all
//				waterServo.write(waterUndropped);
//				habLServo.write(habsLUndropped);
//				habRServo.write(habsRUndropped);
//				glider1Servo.write(glider1Undropped);
//				glider2Servo.write(glider2Undropped);
//			}
//		} else 
		if (dropArmed && !manualServo) {
			if (autoDrop) {
				dropLHabs = autoDropHabs;
				dropRHabs = autoDropHabs;
				dropWater = autoDropWater;
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
					//Serial.println("hab l drop");
					droppedLHabs = true;
				} else {
					habLServo.write(habsLUndropped);
					droppedLHabs = false;
				}
				if (dropRHabs) {
					habRServo.write(habsRDropped);
					//Serial.println("hab r drop");
					droppedRHabs = true;
				} else {
					habRServo.write(habsRUndropped);
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
				// only collect every 100ms
				if (millis() - lastSampleTime > 100) {
					if (SpecGPS::ubg.getLatitude_deg() > 0.1) {
						latSum += SpecGPS::ubg.getLatitude_deg();
						lngSum += SpecGPS::ubg.getLongitude_deg();
						altSum += SpecGPS::ubg.getMSLHeight_m();
						nLocationSamples++;
						// change the vars that are in memory
						Settings::targetLatitude = latSum / nLocationSamples;
						Settings::targetLongitude = lngSum / nLocationSamples;
						Settings::targetAltitude = altSum / nLocationSamples;
						
						Serial.print("Saving Lat: "); Serial.println(Settings::targetLatitude);
						Serial.print("Saving Lng: "); Serial.println(Settings::targetLongitude);
						
						// save those vars to eeprom
						Settings::saveSettings();

						Prediction::setup();
						
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
		if (droppedRHabs) {
			sendBack |= 0b00010000;
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

	void resetDroppedStatus() {
		droppedGlider1 = false;
		droppedGlider2 = false;
		droppedLHabs = false;
		droppedRHabs = false;
		droppedWater = false;
	}

}

#endif
