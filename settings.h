#ifndef SETTINGS_H
#define SETTINGS_H

#include <EEPROM.h>

namespace Settings {
	const int StartAddress = 3000;
	const int varSize = 20;

	int nSamples = 0;
	double totalTargetLongitude;
	double totalTargetLatitude;

	// vars in memory
	double targetLongitude;
	double targetLatitude;
	double targetAltitude;

	// define a struct for storing the settings in EEPROM and instantiate one
	struct SettingsStruct {
		long saveTime;
		double targetLatitude;
		double targetLongitude;
		double targetAltitude;
	};

	void saveSettings() {
		// create object to be pushed to EEPROM
		SettingsStruct toWrite;
		// coords
		toWrite.targetLatitude = targetLatitude;
		toWrite.targetLongitude = targetLongitude;
		toWrite.targetAltitude = targetAltitude;
		// record time
		toWrite.saveTime = millis();
		
		// write
		for (int addressOffset = 0; addressOffset < sizeof(toWrite); addressOffset++) {
			EEPROM.write(StartAddress + addressOffset, *((char *)&toWrite + addressOffset));
		}
		
		//Serial.println("Saved Settings");
	}

	void loadSettings() {
		SettingsStruct loaded;
		for (int addressOffset = 0; addressOffset < sizeof(loaded); addressOffset++)
		{
			*((char *)&loaded + addressOffset) = EEPROM.read(StartAddress + addressOffset);
		}
		
		if (loaded.targetLatitude < 1 || isnan(loaded.targetLatitude)) {
			loaded.targetLatitude = 39.747834;
			loaded.targetLongitude = -83.812673;
			loaded.targetAltitude = 0.0;
		}
		
		Serial.println("Loaded Settings:");
		Serial.print("Save Time: "); Serial.println(loaded.saveTime);
		Serial.print("Target Latitude: "); Serial.println(String(loaded.targetLatitude, 9));
		Serial.print("Target Longitude: "); Serial.println(String(loaded.targetLongitude, 9));
		Serial.print("Target Altitude: "); Serial.println(String(loaded.targetAltitude, 9));
		
		targetLatitude = loaded.targetLatitude;
		targetLongitude = loaded.targetLongitude;
		targetAltitude = loaded.targetAltitude;
	}

}; // namespace Settings

#endif
