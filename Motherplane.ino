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
#include <SpecHMC5883.h>

SpecBMP180 bmp;

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

    SpecSD::setup("test003.txt");

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

    if(millis() - SpecRFD900::UpdateTimer > 1000 / SpecRFD900::UpdatePeriod){
        //telemetry = "";

        //telemetry += SpecGPS::gps.location.lat(); // deg
        Serial.println(SpecGPS::gps.location.lat());
        //telemetry += " ";
        Serial.println(SpecGPS::gps.location.lng()); // deg
        //telemetry += " ";
        Serial.println(SpecGPS::gps.speed.mps()); // m/s
        //telemetry += " ";
        Serial.println(SpecGPS::gps.altitude.value()); // cm
        //telemetry += " ";
        Serial.println(bmp.readOffsetAltitude()); // m
        //telemetry += " ";
        Serial.println(SpecMPU6050::angleAccX); // deg
        //telemetry += " ";
        Serial.println(SpecMPU6050::angleAccY);
        //telemetry += " ";
        Serial.println(SpecMPU6050::angleAccZ);
        //telemetry += " ";
        Serial.println(SpecMPU6050::angleGyroX); // deg/s
        //telemetry += " ";
        Serial.println(SpecMPU6050::angleGyroY);
        //telemetry += " ";
        Serial.println(SpecMPU6050::angleGyroZ);
        //telemetry += " ";
        Serial.println(SpecHMC5883::x); // ?
        //telemetry += " ";
        Serial.println(SpecHMC5883::y);
        //telemetry += " ";
        Serial.println(SpecHMC5883::z);
        //telemetry += " ";
        //Serial.println(SpecHMC5883::heading); // deg
        //telemetry += " ";
        Serial.println(millis()/100); // ds
        //telemetry += " ";

        //telemetry += "!";
        //SpecRFD900::sendTelemetry(telemetry);
        //SpecSD::writeTelemetry(telemetry);
        //Serial.println("sending telemetry");
        //Serial.println(telemetry);
        SpecRFD900::UpdateTimer = millis();

    }

    if(millis() - SpecSD::UpdateTimer > 1000 / SpecSD::UpdatePeriod){
        SpecSD::writeTelemetry(SpecGPS::gps.location.lat(), 15);
        SpecSD::writeTelemetry(SpecGPS::gps.location.lng(), 15); // deg
        SpecSD::writeTelemetry(SpecGPS::gps.altitude.value() / 100, 1); // m
        SpecSD::writeTelemetry(SpecGPS::gps.speed.mps(), 2); // m/s
        SpecSD::writeTelemetry(bmp.readOffsetAltitude(), 2); // m
        SpecSD::writeTelemetry(SpecMPU6050::angleAccX, 15); // deg
        SpecSD::writeTelemetry(SpecMPU6050::angleAccY, 15);
        SpecSD::writeTelemetry(SpecMPU6050::angleAccZ, 15);
        SpecSD::writeTelemetry(SpecMPU6050::angleGyroX, 15); // deg/s
        SpecSD::writeTelemetry(SpecMPU6050::angleGyroY, 15);
        SpecSD::writeTelemetry(SpecMPU6050::angleGyroZ, 15);
        SpecSD::writeTelemetry(SpecHMC5883::x, 10); // compass x
        SpecSD::writeTelemetry(SpecHMC5883::y, 10);
        SpecSD::writeTelemetry(SpecHMC5883::z, 10);
        //SpecSD::writeTelemetry(SpecHMC5883::heading); // deg
        SpecSD::writeTelemetry((float)millis()/1000, 1); // s
        SpecSD::writeTelemetry("\n");
        SpecSD::UpdateTimer = millis();
    }

}
