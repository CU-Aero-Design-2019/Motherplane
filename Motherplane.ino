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

#include "Prediction.h"

// String to write telemetry to which will be sent to ground station.
// Done this way so that we can construct the string once then print to USB and RFD.
String telemetry;

#ifdef SDTELEMETRY
	String sdt;
#endif

#ifdef LOOPTRACKER
	long startTime;
#endif

bool hasBMPReset = false;
int bmpStartTime;

void setup() {
	
	// this gives you time to open the serial monitor
	delay(1000);
	
    SpecGPS::setup();
	
	Drop::setup();
	
	USB::setup();
				
	#ifdef HASBMP
		// bmp.begin() delays for about (35 * int)ms
		if (!bmp.begin(50)) {
	        Serial.println("Could not find a valid BMP085 sensor");
	    }
		bmpStartTime = millis();
	#endif
    
    SpecRFD900::setup(&Serial3);
	
    Settings::loadSettings();
	
	Prediction::setup();
	
	pinMode(PA10, INPUT_PULLUP);
	
	if (digitalRead(PA10) == LOW) {
		Drop::switchControl = true;
	}

	#ifdef SDTELEMETRY
		SpecSD::setup("test");
	#endif
	
	Serial.println("done setup");
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
	
    SpecRFD900::update();
	
	USB::update();
		
	Drop::update();

	// must update prediction before this
	//Drop::updateAuto();

    // needs to be constantly fed with gps data
    SpecGPS::update();
	
	#ifdef HASBMP
		// updates the kalman filter on the baro
		bmp.update();
	#endif

    if (millis() > SpecRFD900::UpdateTimer) {
        SpecRFD900::UpdateTimer = millis() + 100;

        Serial.print(SpecGPS::ubg.getNumSatellites());
		
		telemetry = "";
		
        telemetry += String(SpecGPS::ubg.getDay()) + String(SpecGPS::ubg.getHour()) + String(SpecGPS::ubg.getMin()) + 
        			 String(SpecGPS::ubg.getSec()) + String(SpecGPS::ubg.getNanoSec()).substring(0, 1) + " ";

        telemetry += String(SpecGPS::ubg.getGroundSpeed_ms()) + " ";
		
		if (Drop::collectTarget) {
			telemetry += String(Settings::targetLatitude, 6) + " ";
	        telemetry += String(Settings::targetLongitude, 6) + " ";
		} else {
			double lat = SpecGPS::ubg.getLatitude_deg();
			double lng = SpecGPS::ubg.getLongitude_deg();
			double alt = bmp.getKAlt();

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
		sdt += String(SpecGPS::ubg.getLatitude_deg(), 6);
		sdt += " ";
        sdt += String(SpecGPS::ubg.getLongitude_deg(), 6); // deg
        sdt += " ";
		sdt += String(SpecGPS::getOffsetAlt(), 1); // m
        sdt += " ";
		sdt += String(SpecGPS::ubg.getGroundSpeed_ms(), 2); // m/s
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
		sdt += String(SpecGPS::ubg.getTow_ms());
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
