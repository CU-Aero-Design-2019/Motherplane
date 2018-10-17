// Main file for SAE aero design 2019 motherplane
// General rules:
//     Never use delay()s.

#define DEBUG

#include "settings.h"
#include <Servo.h>
#include "constants.h"
#include "USB.h"
#include <SpecGPS.h>
#include <SpecMPU6050.h>
#include <SpecRFD900.h>
#include <SpecBMP180.h>

SpecBMP180 bmp;

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

    if (!bmp.begin()) {
        Serial.println("Could not find a valid BMP085 sensor, check wiring!");
    }
}

void loop(){
    USB::update();
    
    if(millis() - SpecMPU6050::UpdateTimer > 1000/SpecMPU6050::UpdatePeriod){
        SpecMPU6050::update();
        //Serial.println(IMU::rawGyroX);
        SpecMPU6050::UpdateTimer = millis();
    }

    if(millis() - bmp.UpdateTimer > 1000/bmp.UpdatePeriod){
        //Serial.println(bmp.readAltitude());
        bmp.UpdateTimer = millis();
    }

    //SpecRFD900::update();

    //SpecRFD900::send(String(millis()));

}
