// Main file for SAE aero design 2019 motherplane

//#define SDTELEMETRY
//#define RCIN
//#define LOOPTRACKER

#include "settings.h"
#include <Servo.h>
#include "constants.h"
#include <SpecGPS.h>
#include <SpecMPU6050.h>
#include "SpecRFD900.h"
#include <SpecBMP180.h>
#ifdef MEMORYCHECK
	#include <MemoryFree.h>;
#endif
#include "Drop.h"

#ifdef RCIN
	#include <SBUS.h>
#else
	#include "USB.h"
#endif

#ifdef SDTELEMETRY
	#include <SpecSD.h>
#endif

SpecBMP180 bmp;

#include <JohnnyKalman.h>
#include "Prediction.h"

#ifdef RCIN
	SBUS rcIn(Serial1);
	uint16_t channel[16];
	bool failsafe = false;
	bool lostFrame = false;
#endif

// String to write telemetry to which will be sent to ground station
String telemetry;

#ifdef SDTELEMETRY
	// String to write telemetry to which will be sent to SD card
	String sdt;
#endif

#ifdef LOOPTRACKER
	long startTime;
#endif

void setup() {
	
	delay(2000);
	
    // GPS setup
    SpecGPS::setup();
	
	Drop::setup();
	
	// USB and RC share Serial1 so only one should be used.
	#ifdef RCIN
		rcIn.begin();
	#else
		// USB serial setup
		USB::setup();
	#endif

    // IMU setup
    SpecMPU6050::setup();
			
	// bmp.begin() delays for about (35 * int)ms
	if (!bmp.begin(50)) {
        Serial.println("Could not find a valid BMP085 sensor");
    }
    
    SpecRFD900::setup(&Serial3);
	
	// load settings from EEPROM
    Settings::loadSettings();
	
	Prediction::setup();

	#ifdef SDTELEMETRY
		SpecSD::setup("test");
	#endif
}

void loop() {
	
	#ifdef LOOPTRACKER
		startTime = millis();
	#endif
	
    // check for incoming serial data
	#ifndef RCIN
		USB::update();
	#endif
    SpecRFD900::update();
	
	Drop::update();

    // needs to be constantly updated
    SpecGPS::update();
	
	bmp.update();
	
	if ((!JohnnyKalman::hasDoneSetup) && SpecGPS::gps.location.age() < 2000) {
		// get saved target coords for reference point
		SpecGPS::LLA targetLLA;
		targetLLA.lat = Settings::targetLatitude;
		targetLLA.lng = Settings::targetLongitude;
		targetLLA.alt = Settings::targetAltitude;
		
		JohnnyKalman::initial_kf_setup(targetLLA);
	}
	if (JohnnyKalman::hasDoneSetup && JohnnyKalman::nextTime < millis()) {
		JohnnyKalman::kalman_update();
		
		JohnnyKalman::nextTime = millis() + 100;
	}
    
    if (millis() - SpecMPU6050::UpdateTimer > 1000 / SpecMPU6050::UpdatePeriod) {
        SpecMPU6050::update();
        SpecMPU6050::UpdateTimer = millis();
    }

	if (millis() - Prediction::UpdateTimer > 1000 / Prediction::UpdatePeriod) {
        Prediction::update();
        Prediction::UpdateTimer = millis();
    }

    if (millis() - SpecRFD900::UpdateTimer > 1000 / SpecRFD900::UpdatePeriod) {
        SpecRFD900::UpdateTimer = millis();
		
		telemetry = "";

        // add current time
        telemetry += SpecGPS::gps.time.value();
        telemetry += " ";

        // speed
        telemetry += SpecGPS::gps.speed.value();
        telemetry += " ";
		
		if (Drop::collectTarget) {
			Serial.println("Collecting Target");
			telemetry += String(Settings::targetLatitude, 8);
	        telemetry += " ";
	        telemetry += String(Settings::targetLongitude, 8);;
	        telemetry += " ";
		} else {
			// telemetry += String(SpecGPS::gps.location.lat(), 8);
	        // telemetry += " ";
	        // telemetry += String(SpecGPS::gps.location.lng(), 8);;
	        // telemetry += " ";
			telemetry += String(JohnnyKalman::filter_output.x_pos, 8);
	        telemetry += " ";
	        telemetry += String(JohnnyKalman::filter_output.y_pos, 8);
	        telemetry += " ";
	    }

        // add altitude
        //telemetry += bmp.getKAlt();
		//telemetry += bmp.readAvgOffsetAltitude();
		telemetry += JohnnyKalman::filter_output.z_pos;
        telemetry += " ";

        telemetry += millis()/100;
        telemetry += " ";
		
		telemetry += String(Prediction::prediction.e, 1);
        telemetry += " ";
		
		telemetry += String(Prediction::prediction.n, 1);
        telemetry += " ";
		
		telemetry += String(Drop::sendBack, HEX);
        telemetry += " ";
				
		#ifdef RCIN
			// RC throttle
			rcIn.read(&channel[0], &failsafe, &lostFrame);
			telemetry += String(channel[1]);
			telemetry += " ";
		#endif
		
        telemetry += "!";
        SpecRFD900::sendTelemetry(telemetry);
		Serial.print(telemetry + " ");
		#ifdef MEMORYCHECK
			Serial.print(freeMemory(), DEC);
			Serial.print(" ");
		#endif
		Serial.print(String(Prediction::bearing) + " ");
		Serial.print(String(SpecGPS::gps.satellites.value()) + " ");
		Serial.println(SpecRFD900::in, HEX);
    }

	#ifdef SDTELEMETRY
    if (millis() - SpecSD::UpdateTimer > 1000 / SpecSD::UpdatePeriod) {
        SpecSD::UpdateTimer = millis();
		sdt = "";
		sdt += String(SpecGPS::gps.location.lat(), 9);
		sdt += " ";
        sdt += String(SpecGPS::gps.location.lng(), 9); // deg
        sdt += " ";
		sdt += String(SpecGPS::gps.altitude.meters(), 1); // m
        sdt += " ";
		sdt += String(SpecGPS::gps.speed.mps(), 2); // m/s
        sdt += " ";
		//sdt += String(bmp.readOffsetAltitude(), 2); // m
        sdt += " ";
		sdt += String(SpecMPU6050::angleAccX, 9); // deg
        sdt += " ";
		sdt += String(SpecMPU6050::angleAccY, 9);
        sdt += " ";
		sdt += String(SpecMPU6050::angleGyroX, 9); // deg/s
        sdt += " ";
		sdt += String(SpecMPU6050::angleGyroY, 9);
        sdt += " ";
		sdt += String(SpecMPU6050::angleGyroZ, 9);
		sdt += " ";
		sdt += String(SpecGPS::gps.time.minute());
		sdt += " ";
		sdt += String(SpecGPS::gps.time.second());
		sdt += " ";
		sdt += String(SpecGPS::gps.time.centisecond());
        sdt += String("\n");
		SpecSD::writeTelemetry(sdt);
    }
	#endif
	
	#ifdef LOOPTRACKER
	// to test how long it takes to run the loop
	long loopTime = millis() - startTime;
	//if(loopTime > 4) {
		Serial.println("Total Time: " + String(loopTime));
	//}
	#endif

}
