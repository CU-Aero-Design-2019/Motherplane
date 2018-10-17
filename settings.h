#ifndef SETTINGS_H
#define SETTINGS_H

#include <EEPROM.h>

namespace Settings
{
const int StartAddress = 3200;
const int varSize = 20;

String targetLongitude;
String targetLatitude;
String targetAltitude;

// define a struct for storing the settings in EEPROM and instantiate one
struct SettingsStruct
{
    long saveTime;
    char targetLongitude[varSize];
    char targetLatitude[varSize];
    char targetAltitude[varSize];
};

SettingsStruct settings{
    millis(),
    "defaultLongitude",
    "defaultLatitude",
    "defaultAltitude"
};

void saveSettings()
{
    settings.saveTime = millis();
    targetLongitude.toCharArray(settings.targetLongitude, 20);
    targetLatitude.toCharArray(settings.targetLatitude, 20);
    targetAltitude.toCharArray(settings.targetAltitude, 20);
    for (int addressOffset = 0; addressOffset < sizeof(settings); addressOffset++)
    {
        #ifdef DEBUG
        //Serial.println("Writing EEPROM");
        #endif
        EEPROM.write(StartAddress + addressOffset, *((char *)&settings + addressOffset));
    }
}

void loadSettings()
{
    SettingsStruct loaded;
    for (int addressOffset = 0; addressOffset < sizeof(loaded); addressOffset++)
    {
        *((char *)&loaded + addressOffset) = EEPROM.read(StartAddress + addressOffset);
    }
#ifdef DEBUG
    Serial.println(loaded.saveTime);
    Serial.println(loaded.targetLongitude);
    Serial.println(loaded.targetLatitude);
    Serial.println(loaded.targetAltitude);
#endif
}

void setTarget(String newLongitude, String newLatitude, String newAltitude)
{
    targetLongitude = newLongitude;
    targetLatitude = newLatitude;
    targetAltitude = newAltitude;
    saveSettings();
}
}; // namespace Settings

#endif
