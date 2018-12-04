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
#include <SpecHMC5883.h>

SpecBMP180 bmp;

// Make a string to send to SD and RFD900
String telemetry = "";

void setup(){
    // load settings from EEPROM
    Settings::loadSettings();

    // GPS setup
    SpecGPS::setup();

    // USB serial setup
    USB::setup();

    SpecHMC5883::setup();

    // IMU setup
    SpecMPU6050::setup();

    delay(1000);
    
    SpecRFD900::setup(&Serial3);

    SpecSD::setup("test002.txt");

    if (!bmp.begin()) {
        Serial.println("Could not find a valid BMP085 sensor");
    }
}

void loop(){
    // check for incoming serial data
    USB::update();
    SpecRFD900::update();

    // needs to be constantly updated
    SpecGPS::update();
    
    if(millis() - SpecMPU6050::UpdateTimer > 1000/SpecMPU6050::UpdatePeriod){
        SpecMPU6050::update();
        //Serial.println(IMU::rawGyroX);
        SpecMPU6050::UpdateTimer = millis();
    }

    if(millis() - SpecHMC5883::UpdateTimer > 1000/SpecHMC5883::UpdatePeriod){
        SpecHMC5883::update();
        SpecHMC5883::UpdateTimer = millis();
        //Serial.println(SpecHMC5883::x);
    }

    if(millis() - SpecRFD900::UpdateTimer > 1000/SpecRFD900::UpdatePeriod){
        telemetry = "";

        // // add current time
        // telemetry += SpecGPS::gps.time.value();
        // telemetry += " ";

        // // speed
        // telemetry += SpecGPS::gps.speed.value();
        // telemetry += " ";

        // // location
        // telemetry += SpecGPS::gps.location.lat();
        // telemetry += " ";
        // telemetry += SpecGPS::gps.location.lng();
        // telemetry += " ";

        // // add altitude
        // telemetry += bmp.readOffsetAltitude();
        // telemetry += " ";
        
        // // add angleX
        // telemetry += SpecMPU6050::angleX;
        // telemetry += " ";

        // telemetry += millis()/1000;
        // telemetry += " ";

        telemetry += SpecGPS::gps.location.lat(); // deg
        telemetry += " ";
        telemetry += SpecGPS::gps.location.lng(); // deg
        telemetry += " ";
        telemetry += SpecGPS::gps.speed.mps(); // m/s
        telemetry += " ";
        telemetry += SpecGPS::gps.altitude.value(); // cm
        telemetry += " ";
        telemetry += bmp.readOffsetAltitude(); // m
        telemetry += " ";
        telemetry += SpecMPU6050::angleAccX; // deg
        telemetry += " ";
        telemetry += SpecMPU6050::angleAccY;
        telemetry += " ";
        telemetry += SpecMPU6050::angleAccZ;
        telemetry += " ";
        telemetry += SpecMPU6050::angleGyroX; // deg/s
        telemetry += " ";
        telemetry += SpecMPU6050::angleGyroY;
        telemetry += " ";
        telemetry += SpecMPU6050::angleGyroZ;
        telemetry += " ";
        telemetry += SpecHMC5883::x; // ?
        telemetry += " ";
        telemetry += SpecHMC5883::y;
        telemetry += " ";
        telemetry += SpecHMC5883::z;
        telemetry += " ";
        telemetry += SpecHMC5883::heading; // deg
        telemetry += " ";
        telemetry += millis()/100; // ds
        telemetry += " ";

        telemetry += "!";
        SpecRFD900::sendTelemetry(telemetry);
        SpecSD::writeTelemetry(telemetry);
        //Serial.println("sending telemetry");
        Serial.println(telemetry);
        SpecRFD900::UpdateTimer = millis();

    }

    if(millis() - SpecSD::UpdateTimer > 1000/SpecSD::UpdatePeriod){
        Serial.println(telemetry);
        SpecSD::UpdateTimer = millis();
    }

}
