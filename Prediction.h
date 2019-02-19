 #ifndef PREDICTION
 #define PREDICTION
 
 #include <SpecGPS.h>
 #include "settings.h"
 #include <SpecBMP180.h>
 #include <SpecHMC5883.h>
 
extern SpecBMP180 bmp;


namespace Prediction {
	 
	unsigned long UpdateTimer = 0;
    const unsigned long UpdatePeriod = 100;
	
	// float fake = -100;
	// float fakeSpeed = 10;
	
	// get current coords
	SpecGPS::ENU curENU;
	SpecGPS::LLA curLLA;
	
	// and target coords
	SpecGPS::ECEF tarECEF;
	SpecGPS::LLA tarLLA;
	
	SpecGPS::ENU habPrediction;
	SpecGPS::ENU watPrediction;

	uint16_t bearing;
	
	const float tDel = 0.01;
	
	// prototype
	SpecGPS::ENU makePrediction(bool habitat);
 
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
				
		//update current LLA
		curLLA.lat = SpecGPS::gps.location.lat();
		curLLA.lng = SpecGPS::gps.location.lng();
		//curLLA.alt = bmp.readAvgOffsetAltitude();
		curLLA.alt = bmp.getKAlt();
		// curLLA.alt = SpecGPS::getOffsetAlt();
				
		SpecGPS::lla_to_enu(curLLA, tarLLA, tarECEF, curENU);
		
		habPrediction = makePrediction(true);
		watPrediction = makePrediction(false);
	}
	
	SpecGPS::ENU makePrediction(bool habitat) {
		
		float packageMass;

		float speed = SpecGPS::gps.speed.mps();
		bearing = SpecHMC5883::heading;
		// float speed = 30;
		// bearing = 90;
		
		float x = curENU.e;
        float y = curENU.n;
        float z = curENU.u;
        // float x = -50;
        // float y = 0;
        // float z = 35;
		
		// find initial velocity from speed and bearing
        float u = speed * cos(bearing * 180 / 3.14159265);
        float v = speed * sin(bearing * 180 / 3.14159265);
        float w = 0;
		// float u = JohnnyKalman::filter_output.x_vel;
		// float v = JohnnyKalman::filter_output.y_vel;
		// float w = JohnnyKalman::filter_output.z_vel;
		
		//float groundAirSpeedOffset = 2;
		
		float uAir = 0;
		float vAir = 0;
		
		float dragVert; // new
		float dragHorz; // new
		float rho = 2.699;
		float area_xy;
		float area_z;
		float dragVert_para;
		float area_z_para = 0.2919;

		float habDelay = 1.52;
		float waterDelay = 1.02;

		// true - habitat, false - water bottles
		// new
		if (habitat == true) {
			dragVert_para = 0;
			dragVert = 0.764;
			dragHorz = 0.139;
			area_xy = 0.00541;
			area_z = 0.018177;
			packageMass = 0.12856509;
			
		} else {
			dragVert_para = 0.128;
			dragVert = 1.05;//?
			dragHorz = 1.5;//?
			area_xy = 0.0129;
			area_z = 0.0032;
			packageMass = 0.50121957;
		}
		
		float ax = -(dragHorz/packageMass)*0.5*rho*area_xy*(u-uAir)*abs(u-uAir);
        float ay = -(dragHorz/packageMass)*0.5*rho*area_xy*(v-vAir)*abs(v-vAir);
        float az = -9.807 + 0.5*rho*w*w*(dragVert_para * area_z_para + dragVert * area_z);
		
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
            
            ax = -(dragHorz/packageMass)*0.5*rho*area_xy*(u-uAir)*abs(u-uAir);
			ay = -(dragHorz/packageMass)*0.5*rho*area_xy*(v-vAir)*abs(v-vAir);
			az = -9.807 + 0.5*rho*w*w*(dragVert_para * area_z_para + dragVert * area_z);
        }
		
		SpecGPS::ENU prediction;
		
		// prediction.e = x;
		// prediction.n = y;
		// prediction.u = z;

		if (habitat) {
			prediction.e = x + habDelay * u;
			prediction.n = y + habDelay * v;
			prediction.u = z;
		} else {
			prediction.e = x + waterDelay * u;
			prediction.n = y + waterDelay * v;
			prediction.u = z;
		}
		
		// Serial.println("fake e: " + String(fake) + " prediction e: " + String(prediction.e) + " speed: " + String(fakeSpeed) + " difference:" + String(fake - prediction.e));
		
		return prediction;
		
		// Serial.println("E: " + String(prediction.e, 9));
		// Serial.println("N: " + String(prediction.n, 9));
		// Serial.println("U: " + String(prediction.u, 2));
		
	}
 
 };
 
 
 #endif
 
 
 
 
 
 
 
 
 