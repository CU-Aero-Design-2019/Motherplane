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

void saveSettings()
{
    SettingsStruct sendingSettings;
    sendingSettings.saveTime = millis();
    targetLongitude.toCharArray(sendingSettings.targetLongitude, 20);
    targetLatitude.toCharArray(sendingSettings.targetLatitude, 20);
    targetAltitude.toCharArray(sendingSettings.targetAltitude, 20);
    for (int addressOffset = 0; addressOffset < sizeof(sendingSettings); addressOffset++)
    {
        #ifdef DEBUG
        //Serial.println("Writing EEPROM");
        #endif
        EEPROM.write(StartAddress + addressOffset, *((char *)&sendingSettings + addressOffset));
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
