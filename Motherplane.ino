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

//#define SDTELEMETRY
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

#include "USB.h"

#ifdef SDTELEMETRY
	#include <SpecSD.h>
#endif

SpecBMP180 bmp;

//#include <JohnnyKalman.h>
#include "Prediction.h"

// String to write telemetry to which will be sent to ground station
String telemetry;

#ifdef SDTELEMETRY
	// String to write telemetry to which will be sent to SD card
	String sdt;
#endif

#ifdef LOOPTRACKER
	long startTime;
#endif

bool hasBMPReset = false;
int bmpStartTime;
//unsigned long kalmanStartTime = 2147483647;

void setup() {
	
	delay(2000);
	
    // GPS setup
    SpecGPS::setup();
	
	Drop::setup();
	
	// USB serial setup
	USB::setup();

    // IMU setup
    //SpecMPU6050::setup();
			
	#ifdef HASBMP
	// bmp.begin() delays for about (35 * int)ms
	if (!bmp.begin(50)) {
        Serial.println("Could not find a valid BMP085 sensor");
    }
	bmpStartTime = millis();
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
	
	if (!hasBMPReset) {
		if (millis() >= bmpStartTime + 1000) {
			bmp.resetOffset();
			hasBMPReset = true;
		} else {
			bmp.update();
		}
		return;
	}
	
    // check for incoming serial data
    SpecRFD900::update();
	
	USB::update();
	
	// update drop values
	Drop::update();

    // needs to be constantly fed with gps data
    SpecGPS::update();
	
	// updates the kalman filter on the baro
	#ifdef HASBMP
		bmp.update();
	#endif

    if (millis() > SpecRFD900::UpdateTimer) {
        SpecRFD900::UpdateTimer = millis() + 100;
		
		// if ((!JohnnyKalman::hasDoneSetup) && SpecGPS::gps.satellites.value() >= 4) {
			// //Serial.println("Starting Kalman Setup");
			// JohnnyKalman::initial_kf_setup();
			// bmp.resetOffset(100);
			// kalmanStartTime = millis();
			// //Serial.println("Done Kalman Setup");
		// } else if (JohnnyKalman::hasDoneSetup) {
			// JohnnyKalman::kalman_update();
			// //Serial.println("kalman updated");
		// }

		Serial.print(SpecGPS::gps.satellites.value()); Serial.print(" ");
		
		telemetry = "";
		
        // add current time
        telemetry += SpecGPS::gps.time.value();
        telemetry += " ";

        // speed
        telemetry += SpecGPS::gps.speed.mph();
        telemetry += " ";
		
		if (Drop::collectTarget) {
			//Serial.println("Collecting Target");

			// Settings::nSamples++;

			// Settings::totalTargetLatitude += SpecGPS::gps.location.lat();
			// Settings::totalTargetLongitude += SpecGPS::gps.location.lng();

			// Settings::targetLatitude = Settings::totalTargetLatitude / Settings::nSamples;
			// Settings::targetLongitude = Settings::totalTargetLongitude / Settings::nSamples;

			// Settings::saveSettings();

			telemetry += String(Settings::targetLatitude, 6);
	        telemetry += " ";
	        telemetry += String(Settings::targetLongitude, 6);
	        telemetry += " ";
		} else {
			// telemetry += String(SpecGPS::gps.location.lat(), 8);
	        // telemetry += " ";
	        // telemetry += String(SpecGPS::gps.location.lng(), 8);;
	        // telemetry += " ";
			
			double lat = SpecGPS::gps.location.lat();
			double lng = SpecGPS::gps.location.lng();
			double alt = bmp.getKAlt();
			// double alt = SpecGPS::getOffsetAlt();
			
			SpecGPS::prevENU = SpecGPS::currentENU;
			SpecGPS::lla_to_enu(lat, lng, alt, Settings::targetLatitude, Settings::targetLongitude);
			SpecGPS::currentENU.e = lat;
			SpecGPS::currentENU.n = lng;
			SpecGPS::currentENU.u = alt;

			// must update prediction before this
			Drop::updateAuto();

			if (lat > 10000 || lat < -10000){
				telemetry += String(lat, 4);
				telemetry += " ";
				telemetry += String(lng, 4);
				telemetry += " ";
			} else {
				telemetry += String(lat, 4);
				telemetry += " ";
				telemetry += String(lng, 4);
				telemetry += " ";
			}

			// telemetry += String(JohnnyKalman::filter_output.x_pos, 2);
	        // telemetry += " ";
	        // telemetry += String(JohnnyKalman::filter_output.y_pos, 2);
	        // telemetry += " ";
	    }

        // add altitude
		// if (bmp.getKAlt() > 1000) {
			// telemetry += String(SpecGPS::prevENU.u, 2);
		// } else {
			telemetry += String(bmp.getKAlt(), 2);
		// }
		//telemetry += SpecGPS::getOffsetAlt();
        
        //telemetry += bmp.readOffsetAltitude();
		//telemetry += bmp.readAvgOffsetAltitude();
		//telemetry += String(bmp.getKAlt(), 2);
		//if (millis() + 10000 > kalmanStartTime) {
			//telemetry += String(JohnnyKalman::filter_output.z_pos, 1);
			//telemetry += String(bmp.readOffsetAltitude(), 1);
        //} else {
			//telemetry += "0.00";
		//}
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
		
		telemetry += SpecGPS::gps.course.value();
		telemetry += " ";
		
        telemetry += "!";
        SpecRFD900::sendTelemetry(telemetry);
		Serial.println(telemetry);
		#ifdef MEMORYCHECK
			Serial.print(freeMemory(), DEC);
			Serial.print(" ");
		#endif

    }

	#ifdef SDTELEMETRY
    if (millis() > SpecSD::UpdatePeriod) {
        SpecSD::UpdateTimer = millis() + 100;
		sdt = "";
		sdt += String(SpecGPS::gps.location.lat(), 6);
		sdt += " ";
        sdt += String(SpecGPS::gps.location.lng(), 6); // deg
        sdt += " ";
		sdt += String(SpecGPS::getOffsetAlt(), 1); // m
        sdt += " ";
		sdt += String(SpecGPS::gps.speed.mps(), 2); // m/s
        sdt += " ";
		sdt += String(bmp.getKAlt(), 2); // m
        sdt += " ";
		sdt += String(Prediction::watPrediction.e, 2);
		sdt += " ";
		sdt += String(Prediction::watPrediction.n, 2);
		sdt += " ";
		sdt += String(Prediction::habPrediction.e, 2);
		sdt += " ";
		sdt += String(Prediction::habPrediction.n, 2);
		sdt += " ";
		sdt += String(SpecGPS.currentENU.e);
		sdt += " ";
		sdt += String(SpecGPS.currentENU.n);
		sdt += " ";
		sdt += String(SpecGPS.currentENU.u);
		sdt += " ";
		sdt += String(millis() / 100);
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
