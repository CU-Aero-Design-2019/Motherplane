#ifndef PREDICTION
#define PREDICTION

#define SIMPLE_PREDICTION

#include <SpecGPS.h>
#include "settings.h"
#include <SpecBMP180.h>
#include <SpecHMC5883.h>
 
extern SpecBMP180 bmp;

namespace Prediction {
	 
	unsigned long UpdateTimer = 0;
    const unsigned long UpdatePeriod = 100;
	
	SpecGPS::ENU curENU;
	SpecGPS::LLA curLLA;
	
	SpecGPS::ECEF tarECEF;
	SpecGPS::LLA tarLLA;
	
	SpecGPS::ENU habPrediction;
	SpecGPS::ENU watPrediction;

	double prevLat;
	double prevLng;
	double prevAlt;
	double prevE;
	double prevN;

	double bearing;
	
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

		prevLat = curLLA.lat;
		prevLng = curLLA.lng;
		prevAlt = curLLA.alt;

		prevE = curENU.e;
		prevN = curENU.n;

		curLLA.lat = SpecGPS::ubg.getLatitude_deg();
		curLLA.lng = SpecGPS::ubg.getLongitude_deg();
		curLLA.alt = bmp.getKAlt();

		SpecGPS::lla_to_enu(curLLA, tarLLA, tarECEF, curENU);

		bearing = SpecGPS::ubg.getMotionHeading_deg();
		
		habPrediction = makePrediction(true);
		watPrediction = makePrediction(false);

	}
	
	SpecGPS::ENU makePrediction(bool habitat) {
	#ifdef SIMPLE_PREDICTION
		float speed = SpecGPS::ubg.getGroundSpeed_ms();

		float a = -9.8/2;
		float b = 0;
		float c = bmp.getKAlt();

		float fallTime = (-b - sqrt(b*b-4*a*c))/(2*a);

		if (habitat) {
			fallTime += 1.5;
		} else {
			fallTime += 0.9;
		}

		float velE = speed * sin(bearing / 180 * 3.14159);
		float velN = speed * cos(bearing / 180 * 3.14159);

		float diffE = velE * fallTime;
		float diffN = velN * fallTime;

		SpecGPS::ENU prediction;
		prediction.e = curENU.e + diffE;
		prediction.n = curENU.n + diffN;

		return prediction;

	#else

		float speed = SpecGPS::gps.speed.mps();
		
		float x = curENU.e;
        float y = curENU.n;
        float z = curENU.u;
		
		// find initial velocity from speed and bearing
        float u = speed * cos(bearing / 180 * 3.14159265);
        float v = speed * sin(bearing / 180 * 3.14159265);
        float w = 0;
		
		float rho = 2.699;
		float area_z_para = 0.2919;
		float habDelay = 1.52;
		float waterDelay = 0.87;
		
		float dragVert;
		float dragHorz;
		float packageMass;
		float area_xy;
		float area_z;
		float dragVert_para;

		// true - habitat, false - water bottles
		if (habitat == true) {
			dragVert_para = 0;
			dragVert = 0.764;
			dragHorz = 0.139;
			area_xy = 0.00541;
			area_z = 0.018177;
			packageMass = 0.12856509;
		} else {
			dragVert_para = 0.128;
			dragVert = 1.05;
			dragHorz = 1.5;
			area_xy = 0.0129;
			area_z = 0.0032;
			packageMass = 0.50121957;
		}
		
		float ax = -(dragHorz/packageMass)*0.5*rho*area_xy*u*abs(u);
        float ay = -(dragHorz/packageMass)*0.5*rho*area_xy*v*abs(v);
        float az = -9.807 + 0.5*rho*w*w*(dragVert_para * area_z_para + dragVert * area_z);
		
        while (z > 0){
            x = u*tDel + x;
            y = v*tDel + y;
            z = w*tDel + z;
            
            u = u + (ax*tDel);
            v = v + (ay*tDel);
            w = w + (az*tDel);
            
            ax = -(dragHorz/packageMass)*0.5*rho*area_xy*u*abs(u);
			ay = -(dragHorz/packageMass)*0.5*rho*area_xy*v*abs(v);
			az = -9.807 + 0.5*rho*w*w*(dragVert_para * area_z_para + dragVert * area_z);
        }
		
		SpecGPS::ENU prediction;

		if (habitat) {
			prediction.e = x + habDelay * u;
			prediction.n = y + habDelay * v;
			prediction.u = z;
		} else {
			prediction.e = x + waterDelay * u;
			prediction.n = y + waterDelay * v;
			prediction.u = z;
		}
					
		return prediction;

	#endif
	}
 
 };
 
 
 #endif
 
 
 
 
 
 
 
 
 