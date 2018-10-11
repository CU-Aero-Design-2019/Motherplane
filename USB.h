#ifndef USB_H
#define USB_H

#include "constants.h"
#include "settings.h"

Servo testServo;

// struct to hold serial communication stuff
namespace USB{

    String incoming = "";
    bool waitingForSerial = false;
    long firstSerialAvailableTime;

    void setup(){
        Serial.begin(USBSerialBaudrate);
    }

    // does stuff with the incoming string
    // should be called from update()
    void parse(){
        if(incoming.substring(0, 4).equals("STAR")){
            Settings::setTarget("implementme!", "implementme!", "implementme!");
        }else if(incoming.substring(0, 4).equals("SRVO")){
            Serial.println(incoming.substring(4));
            Serial.println(incoming.substring(4).toInt());
            //testServo.write(incoming.substring(4).toInt());
        }
    }

    // to be called at a regular interval
    // updates incoming string
    void update(){
        //parse serial1 input
        if(Serial.available() && !waitingForSerial){
            waitingForSerial = true;
            firstSerialAvailableTime = millis();
        }
        // if serial has something available and we've waited serialDelay
        if(waitingForSerial && (firstSerialAvailableTime + serialDelay >= millis())){
            // set this pesky thing back to false since we're not waiting anymore
            waitingForSerial = false;
            incoming = "";
            while(Serial.available()){
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