// Main file for SAE aero design 2019 motherplane
// General rules:
//     Never use delay()s.
//     Tabs or 4 spaces (go into arduino settings and check "use external editor" then use a real text editor)

#define DEBUG

#include "settings.h"
#include <Servo.h>
#include "constants.h"
#include "USB.h"
#include <SpecGPS.h>
#include <SpecMPU6050.h>
#include <SpecRFD900.h>

void setup(){
    // GPS setup
    SpecGPS::setup();

    // USB serial setup
    USB::setup();

    // IMU setup
    SpecMPU6050::setup();

    // load settings from EEPROM
    Settings::loadSettings();

    SpecRFD900::setup(&Serial3);
}

void loop(){
    USB::update();
    
    if(millis() - SpecMPU6050::UpdateTimer > 1000/SpecMPU6050::UpdatePeriod){
        SpecMPU6050::update();
        //Serial.println(IMU::rawGyroX);
        SpecMPU6050::UpdateTimer = millis();
    }

    SpecRFD900::update();

    SpecRFD900::send(String(millis()));
    delay(100);

}
