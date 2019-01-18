 #ifndef PREDICTION
 #define PREDICTION
 
 #include <SpecGPS.h>
 #include "settings.h"
 #include <SpecBMP180.h>
 
 namespace Prediction {
	 
	unsigned long UpdateTimer = 0;
    const unsigned long UpdatePeriod = 100;
	
	// get current coords
	SpecGPS::ENU curENU;
	SpecGPS::LLA curLLA;
	
	// previous location
	SpecGPS::ENU prevENU;
	
	// and target coords
	SpecGPS::ECEF tarECEF;
	SpecGPS::LLA tarLLA;
	
	SpecGPS::ENU prediction;
	
	const float tDel = 0.01;
	const float dragVert = 0.01;
	const float dragHorz = 0.01;
	
	// prototype
	SpecGPS::ENU makePrediction(float uAirGround = 0, float uAirPlane = 0, float vAirGround = 0, float vAirPlane = 0, float packageMass = 2);
 
	void setup() {
		
		// get target coords - WORKS
		tarLLA.lat = Settings::targetLatitude;
		tarLLA.lng = Settings::targetLongitude;
		tarLLA.alt = Settings::targetAltitude;
	
		// convert target to ECEF, which is needed for ENU - WORKS
		SpecGPS::lla_to_ecef(tarLLA, tarECEF);
		
	}
	
	void update() {
		
		// curLLA.lat = 39.748119;
		// curLLA.lng = -83.813505;
		// curLLA.alt = 40;
		
		update current LLA
		curLLA.lat = SpecGPS::gps.location.lat();
		curLLA.lng = SpecGPS::gps.location.lng();
		curLLA.alt = bmp.readAvgOffsetAltitude();
		
		prevENU = curENU;
		
		SpecGPS::lla_to_enu(curLLA, tarLLA, tarECEF, curENU);
		
		makePrediction();
	}
	
	SpecGPS::ENU makePrediction(float uAirGround, float uAirPlane, float vAirGround, float vAirPlane, float packageMass) {
		
		// find velocity from two points - WORKS for 0...
		float uIni = (curENU.e - prevENU.e) / 0.1;
		float vIni = (curENU.n - prevENU.n) / 0.1;
		float wIni = (curENU.u - prevENU.u) / 0.1;
		
		float x = curENU.e;
        float y = curENU.n;
        float z = curENU.u;
		
        float u = uIni;
        float v = vIni;
        float w = wIni;
		
		float groundAirSpeedOffset = 2;
		
		float uAir = (((z-groundAirSpeedOffset)*(uAirPlane - uAirGround))/(z-groundAirSpeedOffset)) + uAirGround;
		float vAir = (((z-groundAirSpeedOffset)*(vAirPlane - vAirGround))/(z-groundAirSpeedOffset)) + vAirGround;
		
		float ax = -(dragHorz/packageMass)*(u-uAir)*abs(u-uAir);
        float ay = -(dragHorz/packageMass)*(v-vAir)*abs(v-vAir);
        float az = -9.807 -(dragVert/packageMass)*(w)*abs(w);
		
		int count = 0;
        while (z > 0){
            count++;
            x = u*tDel + x;
            y = v*tDel + y;
            z = w*tDel + z;
            
            u = u + (ax*tDel);
            v = v + (ay*tDel);
            w = w + (az*tDel);
            
            ax = -(dragHorz/packageMass)*(u-uAir)*abs(u-uAir);
            ay = -(dragHorz/packageMass)*(v-vAir)*abs(v-vAir);
            az = -9.807 -(dragVert/packageMass)*(w)*abs(w);
        }
		
		prediction.e = x;
		prediction.n = y;
		prediction.u = z;
		
		// Serial.println("E: " + String(prediction.e, 9));
		// Serial.println("N: " + String(prediction.n, 9));
		// Serial.println("U: " + String(prediction.u, 2));
		
	}
 
 };
 
 
 #endif
 
 
 
 
 
 
 
 
 