#ifndef USB_H
#define USB_H

#include "constants.h"
#include "settings.h"
#include "Drop.h"
#include <SpecGPS.h>
#include <SpecBMP180.h>

// struct to hold serial communication stuff
namespace USB {

    String incoming = "";
    bool waitingForSerial = false;
    long firstSerialAvailableTime;
	
	const int USBSerialBaudrate = 115200;

    void setup() {
        Serial.begin(USBSerialBaudrate);
		//Serial.println("USB Serial Started");
    }

    // does stuff with the incoming string
    // should be called from update()
    void parse() {
        incoming.toUpperCase();
        if (incoming.substring(0, 4).equals("STAR")) {
            String x = incoming.substring(4,5);
            Serial.println(x);
            String y = incoming.substring(5,6);
            Serial.println(y);
            String z = incoming.substring(6,7);
            Serial.println(z);
			Settings::targetLatitude = 39.747834;
			Settings::targetLongitude = -83.812673;
			Settings::saveSettings();
        }else if (incoming.substring(0, 4).equals("GTAR")) {
            Serial.println("Target Lat: " + String(Settings::targetLatitude, 8));
            Serial.println("Target Lng: " + String(Settings::targetLongitude, 8));
        }else if (incoming.substring(0, 4).equals("SRVO")) {
            incoming = incoming.substring(5);
            Serial.println(incoming);

            if(incoming.substring(0, 4).equals("AUTO")) {
                Drop::manualServo = false;
                return;
            }
            
            int index = incoming.substring(0, incoming.indexOf(' ')).toInt();
            int val = incoming.substring(2, incoming.indexOf(' ', 2)).toInt();

            Drop::manualServo = true;

            Drop::manuallySet(index, val);

        }else if (incoming.substring(0, 4).equals("RBLA")) {
			#ifdef HASBMP
				bmp.resetOffset();
			#else
				SpecGPS::resetOffset();
			#endif
		}
    }

    // to be called at a regular interval
    // updates incoming string
    void update() {
        //parse serial1 input
        if (Serial.available() && !waitingForSerial) {
            waitingForSerial = true;
            firstSerialAvailableTime = millis();
        }
        // if serial has something available and we've waited serialDelay
        if (waitingForSerial && (firstSerialAvailableTime + serialDelay >= millis())) {
            // set this pesky thing back to false since we're not waiting anymore
            waitingForSerial = false;
            incoming = "";
            while (Serial.available()) {
                incoming += (char)Serial.read();
            }
            #ifdef DEBUG
            Serial.print("Incoming USB Serial Data: ");
            Serial.print(incoming);
            #endif
            // do something with serial input
            USB::parse();
        }
    }

};


#endif
