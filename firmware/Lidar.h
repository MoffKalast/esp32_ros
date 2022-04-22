#include <Wire.h>
#include "VL53L0X.h"

VL53L0X sensor;

int sensor_started = 0;
int skip_counter = 0;

extern int range;

void startRanging(){
	if (sensor.init())
	{
    	sensor.setSignalRateLimit(0.1);
		sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
		sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);
        sensor.startContinuous();
        sensor_started = 1;
	}
}

void initLidar(){
	Wire.begin(2,3,400000);
	sensor.setTimeout(500);
	startRanging();
}

void runUpdate(){
 	if(sensor_started)
 	{
 		int16_t rawrange = sensor.readRangeContinuousMillimeters();
		if(rawrange > 2000)
			rawrange = 2000;
		else if(rawrange < 30)
			rawrange = 30;

		range = rawrange;
 	}
}

void updateLidar(){
	if(skip_counter >= 20){ //reducing frequency of updating from 200/s to 10/s
		runUpdate();
		skip_counter = 0;
	}
	else{
		skip_counter++;
	}
}
