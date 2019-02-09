// Main file for SAE aero design 2019 motherplane

/* checklist for night before test flight:
Sends telemetry to ground station
Gets GPS lock
Does reasonable alt
Can release/load every payload
Sends dropped signal
Logs data to SD
Updates fast enough

*/

#define SDTELEMETRY
//#define RCIN
//#define LOOPTRACKER
#define HASBMP

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
    //SpecMPU6050::setup();
			
	#ifdef HASBMP
	// bmp.begin() delays for about (35 * int)ms
	if (!bmp.begin(50)) {
        Serial.println("Could not find a valid BMP085 sensor");
    }
	#endif
    
    SpecRFD900::setup(&Serial3);
	
	// load settings from EEPROM
    Settings::loadSettings();
	
	Prediction::setup();

	#ifdef SDTELEMETRY
		SpecSD::setup("test");
	#endif

	//Serial.println("end setup");
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
	
	#ifdef HASBMP
	bmp.update();
	#endif

	// if (millis() - Prediction::UpdateTimer > 1000 / Prediction::UpdatePeriod) {
        // Prediction::update();
        // Prediction::UpdateTimer = millis();
    // }

    if (millis() > SpecRFD900::UpdateTimer) {
        SpecRFD900::UpdateTimer = millis() + 100;
		
		if ((!JohnnyKalman::hasDoneSetup) && SpecGPS::gps.satellites.value() >= 4) {
			//Serial.println("Starting Kalman Setup");
			JohnnyKalman::initial_kf_setup();
			//Serial.println("Done Kalman Setup");
		} else if (JohnnyKalman::hasDoneSetup) {
			JohnnyKalman::kalman_update();
			//Serial.println("kalman updated");
		}
		
		telemetry = "";

        // add current time
        telemetry += SpecGPS::gps.time.value();
        telemetry += " ";

        // speed
        telemetry += SpecGPS::gps.speed.mph();
        telemetry += " ";
		
		if (Drop::collectTarget) {
			//Serial.println("Collecting Target");
			telemetry += String(Settings::targetLatitude, 1);
	        telemetry += " ";
	        telemetry += String(Settings::targetLongitude, 1);
	        telemetry += " ";
		} else {
			// telemetry += String(SpecGPS::gps.location.lat(), 8);
	        // telemetry += " ";
	        // telemetry += String(SpecGPS::gps.location.lng(), 8);;
	        // telemetry += " ";
			
			SpecGPS::LLA currentLLA;
			currentLLA.lat = SpecGPS::gps.location.lat();
			currentLLA.lng = SpecGPS::gps.location.lng();
			SpecGPS::LLA targetLLA;
			targetLLA.lat = Settings::targetLatitude;
			targetLLA.lng = Settings::targetLongitude;
			targetLLA.alt = 0;
			SpecGPS::ENU currentENU;
			SpecGPS::lla_to_enu(currentLLA, targetLLA, currentENU);

			// telemetry += String(currentENU.e, 1);
			// telemetry += " ";
			// telemetry += String(currentENU.n, 1);
			// telemetry += " ";

			telemetry += String(JohnnyKalman::filter_output.x_pos, 2);
	        telemetry += " ";
	        telemetry += String(JohnnyKalman::filter_output.y_pos, 2);
	        telemetry += " ";
	    }

        // add altitude
        //telemetry += bmp.getKAlt();
		//telemetry += bmp.readAvgOffsetAltitude();
		telemetry += String(bmp.getKAlt(), 2);
        telemetry += " ";

        telemetry += millis()/100;
        telemetry += " ";
		
		Prediction::update();
		
		telemetry += String(Prediction::watPrediction.e, 2);
        telemetry += " ";
		
		telemetry += String(Prediction::watPrediction.n, 2);
        telemetry += " ";
		
		telemetry += String(Drop::sendBack, HEX);
        telemetry += " ";
		
		telemetry += Prediction::bearing;
		telemetry += " ";
				
		#ifdef RCIN
			// RC throttle
			rcIn.read(&channel[0], &failsafe, &lostFrame);
			telemetry += String(channel[1]);
			telemetry += " ";
		#endif
		
        telemetry += "!";
        SpecRFD900::sendTelemetry(telemetry);
		//Serial.print(telemetry);
		//Serial.print(telemetry + " ");
		#ifdef MEMORYCHECK
			Serial.print(freeMemory(), DEC);
			Serial.print(" ");
		#endif
		// Serial.print(String(SpecGPS::gps.satellites.value()) + " ");
		// Serial.println(SpecRFD900::in[0], HEX);
		// Serial.print("time: "); Serial.println(SpecGPS::gps.time.value());
		// Serial.print("nSats: "); Serial.println(SpecGPS::gps.satellites.value());
		// Serial.print("posAge: "); Serial.println(SpecGPS::gps.location.age());
		// Serial.print("posValid: "); Serial.println(SpecGPS::gps.location.isValid());
		// Serial.print("lat: "); Serial.println(SpecGPS::gps.location.lat());
		// Serial.print("lng: "); Serial.println(SpecGPS::gps.location.lng());
		// Serial.print("alt: "); Serial.println(SpecGPS::gps.altitude.meters());
    }

	#ifdef SDTELEMETRY
    if (millis() > SpecSD::UpdatePeriod) {
        SpecSD::UpdateTimer = millis() + 100;
		sdt = "";
		sdt += String(SpecGPS::gps.location.lat(), 9);
		sdt += " ";
        sdt += String(SpecGPS::gps.location.lng(), 9); // deg
        sdt += " ";
		sdt += String(SpecGPS::gps.altitude.meters(), 1); // m
        sdt += " ";
		sdt += String(SpecGPS::gps.speed.mps(), 2); // m/s
        sdt += " ";
		sdt += String(bmp.getKAlt(), 2); // m
        sdt += " ";
		sdt += String(millis() / 1000, 2);
        sdt += String("\n");
		SpecSD::writeTelemetry(sdt);
    }
	#endif
	
	#ifdef LOOPTRACKER
	// to test how long it takes to run the loop
	long loopTime = millis() - startTime;
	if(loopTime > 1) {
		Serial.println("Total Time: " + String(loopTime));
	}
	#endif

}
