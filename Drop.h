#ifndef DROP_H
#define DROP_H

#include "settings.h"

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
	
		if (collectTarget) {
			
		}
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
		}
		
		Serial.print("W " + String(dropWater) + ", ");
		Serial.print("H " + String(dropHabs) + ", ");
		Serial.print("G1 " + String(dropGlider1) + ", ");
		Serial.print("G2 " + String(dropGlider2) + ", ");
		Serial.print("Arm " + String(dropArmed) + ", ");
		Serial.print("Auto " + String(autoDrop) + ", ");
		Serial.print("Tar " + String(collectTarget) + "\n");
	}

}

#endif