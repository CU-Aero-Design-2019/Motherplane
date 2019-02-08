 #ifndef PREDICTION
 #define PREDICTION
 
 #include <SpecGPS.h>
 #include "settings.h"
 #include <SpecBMP180.h>
 
 namespace Prediction {
	 
	unsigned long UpdateTimer = 0;
    const unsigned long UpdatePeriod = 100;
	
	float fake = -100;
	float fakeSpeed = 10;
	
	// get current coords
	SpecGPS::ENU curENU;
	SpecGPS::LLA curLLA;
	
	// previous location
	SpecGPS::ENU prevENU;
	SpecGPS::LLA prevLLA;
	
	// and target coords
	SpecGPS::ECEF tarECEF;
	SpecGPS::LLA tarLLA;
	
	SpecGPS::ENU habPrediction;
	SpecGPS::ENU watPrediction;

	uint16_t bearing;
	
	const float tDel = 0.005;
	
	// prototype
	SpecGPS::ENU makePrediction(float packageMass, bool habitat);
 
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
		
		prevLLA = curLLA;
		
		//update current LLA
		curLLA.lat = SpecGPS::gps.location.lat();
		curLLA.lng = SpecGPS::gps.location.lng();
		//curLLA.alt = bmp.readAvgOffsetAltitude();
		//curLLA.alt = bmp.getKAlt();
		
		prevENU = curENU;
		
		SpecGPS::lla_to_enu(curLLA, tarLLA, tarECEF, curENU);
		
		//habPrediction = makePrediction(0.12856509, true);
		watPrediction = makePrediction(0.50121957, false);
	}
	
	SpecGPS::ENU makePrediction(float packageMass, bool habitat) {
		
		//float speed = SpecGPS::gps.speed.mps();
		//bearing = SpecGPS::bearing(prevLLA.lat, prevLLA.lng, curLLA.lat, curLLA.lng);
		//bearing = SpecGPS::gps.course.deg();
		//bearing = 90;
		
		float x = curENU.e;
        float y = curENU.n;
        float z = curENU.u;
		
		// fake += 1.6;
		// if(fake > 10) fake = -300;
		// float x = fake;
        // float y = 0;
        // float z = 30;
		
		// find initial velocity from speed and bearing
        // float u = speed * cos(bearing * 180 / 3.14159265);
        // float v = speed * sin(bearing * 180 / 3.14159265);
        // float w = 0;
		float u = JohnnyKalman::filter_output.x_vel;
		float v = JohnnyKalman::filter_output.y_vel;
		float w = JohnnyKalman::filter_output.z_vel;
		// fakeSpeed += 0.5;
		// if(fakeSpeed > 90) fakeSpeed = 10;
		// float u = fakeSpeed;
		// float v = 0;
		// float w = 0;
		
		float groundAirSpeedOffset = 2;
		
		// float uAir = (((z-groundAirSpeedOffset)*(uAirPlane - uAirGround))/(z-groundAirSpeedOffset)) + uAirGround;
		// float vAir = (((z-groundAirSpeedOffset)*(vAirPlane - vAirGround))/(z-groundAirSpeedOffset)) + vAirGround;
		float uAir = 0;
		float vAir = 0;
		
		float dragVert; // new
		float dragHorz; // new
		float liftConst; // new
		// true - habitat, false - water bottles
		// new
		if (habitat == true) {
			liftConst = 0;
			dragVert = 0.764;;
			dragHorz = 0.139;
		} else {
			liftConst = 0.128;//?
			dragVert = 1.05;//?
			dragHorz = 1.5;//?
		}
		
		float ax = -(dragHorz/packageMass)*(u-uAir)*abs(u-uAir);
        float ay = -(dragHorz/packageMass)*(v-vAir)*abs(v-vAir);
        float az = -9.807 -(dragVert/packageMass)*(w)*abs(w);
		
		int count = 0;
        while (z > 0){
            count++;
            x = u*tDel + x;
			//Serial.println(x);
            y = v*tDel + y;
            z = w*tDel + z;
            
            u = u + (ax*tDel);
            v = v + (ay*tDel);
            w = w + (az*tDel);
            
            ax = -(dragHorz/packageMass)*(u-uAir)*abs(u-uAir);
            ay = -(dragHorz/packageMass)*(v-vAir)*abs(v-vAir);
            az = -9.807 -(dragVert/packageMass)*(w)*abs(w) - liftConst * (abs(w) * abs(w));
        }
		
		SpecGPS::ENU prediction;
		
		prediction.e = x;
		prediction.n = y;
		prediction.u = z;
		
		// Serial.println("fake e: " + String(fake) + " prediction e: " + String(prediction.e) + " speed: " + String(fakeSpeed) + " difference:" + String(fake - prediction.e));
		
		return prediction;
		
		// Serial.println("E: " + String(prediction.e, 9));
		// Serial.println("N: " + String(prediction.n, 9));
		// Serial.println("U: " + String(prediction.u, 2));
		
	}
 
 };
 
 
 #endif
 
 
 
 
 
 
 
 
 