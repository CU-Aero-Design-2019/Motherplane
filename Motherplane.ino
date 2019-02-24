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
//#include <SpecMPU6050.h>
#include "SpecRFD900.h"
#include <SpecBMP180.h>
#include <SpecHMC5883.h>
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
	
	SpecHMC5883::setup();
			
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
	
	SpecHMC5883::update();
	
	// update drop values
	Drop::update();

	// must update prediction before this
	//Drop::updateAuto();

    // needs to be constantly fed with gps data
    SpecGPS::update();
	
	// updates the kalman filter on the baro
	#ifdef HASBMP
		bmp.update();
	#endif

    if (millis() > SpecRFD900::UpdateTimer) {
        SpecRFD900::UpdateTimer = millis() + 100;

		//Serial.print(SpecGPS::gps.satellites.value()); Serial.print(" ");
		
		telemetry = "";
		
        // add current time
        telemetry += String(SpecGPS::gps.time.value()) + " ";

        // speed
        telemetry += String(SpecGPS::gps.speed.mph()) + " ";
		
		if (Drop::collectTarget) {

			telemetry += String(Settings::targetLatitude, 6) + " ";
	        telemetry += String(Settings::targetLongitude, 6) + " ";
		} else {
			
			double lat = SpecGPS::gps.location.lat();
			double lng = SpecGPS::gps.location.lng();
			double alt = bmp.getKAlt();
			// double alt = SpecGPS::getOffsetAlt();
			
			// Serial.println(String(SpecGPS::prevENU.e, 1) + String(SpecGPS::prevENU.n, 1) + String(SpecGPS::prevENU.u, 1));
			// Serial.println(String(SpecGPS::currentENU.e, 1) + String(SpecGPS::currentENU.n, 1) + String(SpecGPS::currentENU.u, 1));
			
			SpecGPS::lla_to_enu(lat, lng, alt, Settings::targetLatitude, Settings::targetLongitude);
			SpecGPS::currentENU.e = lat;
			SpecGPS::currentENU.n = lng;
			SpecGPS::currentENU.u = alt;

			Prediction::update();

			if (lat > 10000 || lat < -10000){
				telemetry += String(lat, 4) + " ";
				telemetry += String(lng, 4) + " ";
			} else {
				telemetry += String(lat, 4) + " ";
				telemetry += String(lng, 4) + " ";
			}

	    }

		telemetry += String(bmp.getKAlt(), 2) + " ";

        telemetry += String(millis()/100) + " ";
		
		telemetry += String(Prediction::watPrediction.e, 2) + " ";
		
		telemetry += String(Prediction::watPrediction.n, 2) + " ";
		
		telemetry += String(Drop::sendBack, HEX) + " ";
		
		//telemetry += SpecGPS::gps.course.value();
		telemetry += String(Prediction::bearing) + " ";

		telemetry += String(Prediction::habPrediction.e, 2) + " ";
		telemetry += String(Prediction::habPrediction.n, 2) + " ";
		
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
        sdt += String(SpecHMC5883::heading, 2); // deg
        sdt += " ";
		sdt += String(Prediction::watPrediction.e, 2);
		sdt += " ";
		sdt += String(Prediction::watPrediction.n, 2);
		sdt += " ";
		sdt += String(Prediction::habPrediction.e, 2);
		sdt += " ";
		sdt += String(Prediction::habPrediction.n, 2);
		sdt += " ";
		sdt += String(SpecGPS::currentENU.e);
		sdt += " ";
		sdt += String(SpecGPS::currentENU.n);
		sdt += " ";
		sdt += String(SpecGPS::currentENU.u);
		sdt += " ";
		sdt += String(SpecGPS::gps.time.value());
		// sdt += " ";
		// sdt += String(millis());
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
