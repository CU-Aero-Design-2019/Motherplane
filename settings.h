#ifndef SETTINGS_H
#define SETTINGS_H

#include <EEPROM.h>

namespace Settings
{
const int StartAddress = 32;
const int varSize = 20;

String targetLongitude;
String targetLatitude;
String targetAltitude;

// define a struct for storing the settings in EEPROM and instantiate one
struct SettingsStruct
{
    char targetLongitude[varSize];
    char targetLatitude[varSize];
    char targetAltitude[varSize];
};

SettingsStruct settings{
    "defaultLongitude",
    "defaultLatitude",
    "defaultAltitude"
};

void saveSettings()
{
    targetLongitude.toCharArray(settings.targetLongitude, 20);
    targetLatitude.toCharArray(settings.targetLatitude, 20);
    targetAltitude.toCharArray(settings.targetAltitude, 20);
    for (int addressOffset = 0; addressOffset < sizeof(settings); addressOffset++)
    {
        EEPROM.write(StartAddress + addressOffset, *((char *)&settings + addressOffset));
    }
}

void loadSettings()
{
    for (int addressOffset = 0; addressOffset < sizeof(settings); addressOffset++)
    {
        *((char *)&settings + addressOffset) = EEPROM.read(StartAddress + addressOffset);
    }
#ifdef DEBUG
    Serial.println(targetLongitude);
    Serial.println(targetLatitude);
    Serial.println(targetAltitude);
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