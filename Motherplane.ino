// Main file for SAE aero design 2019 motherplane
// General rules:
//     Never use delay()s.

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

SpecBMP180 bmp;

// Make a string to send to SD and RFD900
String telemetry = "";

void setup(){
    // GPS setup
    SpecGPS::setup();

    // USB serial setup
    USB::setup();

    // IMU setup
    SpecMPU6050::setup();

    delay(2000);

    // load settings from EEPROM
    Settings::loadSettings();

    SpecRFD900::setup(&Serial3);

    SpecSD::setup();

    if (!bmp.begin()) {
        Serial.println("Could not find a valid BMP085 sensor");
    }
}

void loop(){
    // check for incoming serial data
    USB::update();
    SpecRFD900::update();
    SpecGPS::update();
    
    if(millis() - SpecMPU6050::UpdateTimer > 1000/SpecMPU6050::UpdatePeriod){
        SpecMPU6050::update();
        //Serial.println(IMU::rawGyroX);
        SpecMPU6050::UpdateTimer = millis();
    }

    if(millis() - SpecRFD900::UpdateTimer > 1000/SpecRFD900::UpdatePeriod){
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
        
        // add angleX
        telemetry += SpecMPU6050::angleX;
        telemetry += " ";

        telemetry += millis()/1000;
        telemetry += " ";

        telemetry += "!";
        SpecRFD900::sendTelemetry(telemetry);
        Serial.println(telemetry);
        //Serial.println("sending telemetry");
        SpecRFD900::UpdateTimer = millis();
    }

    if(millis() - SpecSD::UpdateTimer > 1000/SpecSD::UpdatePeriod){
        SpecSD::writeTelemetry(telemetry);
        SpecSD::UpdateTimer = millis();
    }

}
