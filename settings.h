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
	unsigned int fileIndex;
};

void saveSettings()
{
	// create object to be pushed to EEPROM
    SettingsStruct sendingSettings;
	// record time
    sendingSettings.saveTime = millis();
    // record lat long alt
	targetLongitude.toCharArray(sendingSettings.targetLongitude, 20);
    targetLatitude.toCharArray(sendingSettings.targetLatitude, 20);
    targetAltitude.toCharArray(sendingSettings.targetAltitude, 20);
    
	// write
	for (int addressOffset = 0; addressOffset < sizeof(sendingSettings); addressOffset++) {
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
    Serial.println(loaded.saveTime);
    Serial.println(loaded.targetLongitude);
    Serial.println(loaded.targetLatitude);
    Serial.println(loaded.targetAltitude);
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
