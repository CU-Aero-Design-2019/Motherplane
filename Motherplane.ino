// Main file for SAE aero design 2019 motherplane

#define DEBUG

#include "settings.h"
#include <Servo.h>
#include "constants.h"
#include "USB.h"
#include <SpecGPS.h>
#include <SpecSD.h>
#include <SpecMPU6050.h>
#include <SpecRFD900.h>
#include <SpecBMP180.h>
#include <Drop.h>
//#include <HMC5883L_Simple.h>

SpecBMP180 bmp;
//HMC5883L_Simple compass;

String telemetry;
String sdt;

void setup() {
	
    // load settings from EEPROM
    Settings::loadSettings();
	
    // GPS setup
    SpecGPS::setup();

    // USB serial setup
    USB::setup();

    // IMU setup
    SpecMPU6050::setup();

    delay(1000);
    
    SpecRFD900::setup(&Serial3);

    SpecSD::setup("test");

    if (!bmp.begin()) {
        Serial.println("Could not find a valid BMP085 sensor");
    }
}

void loop() {
  
    // check for incoming serial data
    USB::update();
    SpecRFD900::update();
	
	Drop::update();
	
	if (SpecGPS::gps.location.isUpdated()) {
		digitalWrite(LED_BUILTIN, HIGH);
	} else {
		digitalWrite(LED_BUILTIN, LOW);
	}

    // needs to be constantly updated
    SpecGPS::update();
    
    if (millis() - SpecMPU6050::UpdateTimer > 1000/SpecMPU6050::UpdatePeriod) {
        SpecMPU6050::update();
        SpecMPU6050::UpdateTimer = millis();
    }

    if (millis() - SpecRFD900::UpdateTimer > 1000 / SpecRFD900::UpdatePeriod) {
        telemetry = "";

        // add current time
        telemetry += SpecGPS::gps.time.value();
        telemetry += " ";

        // speed
        telemetry += SpecGPS::gps.speed.value();
        telemetry += " ";

        // location
        telemetry += SpecGPS::gps.location.lat();
        telemetry += " ";
        telemetry += SpecGPS::gps.location.lng();
        telemetry += " ";

        // add altitude
        telemetry += bmp.readOffsetAltitude();
        telemetry += " ";

        telemetry += millis()/100;
        telemetry += " ";
		
		telemetry += SpecGPS::gps.altitude.meters();
		telemetry += " ";
		
		if (SpecGPS::hasLock) {
			telemetry += "1";
		} else {
			telemetry += "0";
		}
		telemetry += " ";

        telemetry += "!";
        SpecRFD900::sendTelemetry(telemetry);
        Serial.println(telemetry);
        SpecRFD900::UpdateTimer = millis();
    }

    if (millis() - SpecSD::UpdateTimer > 1000 / SpecSD::UpdatePeriod) {
        sdt = "";
		sdt += String(SpecGPS::gps.location.lat(), 9);
		sdt += " ";
        sdt += String(SpecGPS::gps.location.lng(), 9); // deg
        sdt += " ";
		sdt += String(SpecGPS::gps.altitude.meters(), 1); // m
        sdt += " ";
		sdt += String(SpecGPS::gps.speed.mps(), 2); // m/s
        sdt += " ";
		sdt += String(bmp.readOffsetAltitude(), 2); // m
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
        SpecSD::UpdateTimer = millis();
    }

}
