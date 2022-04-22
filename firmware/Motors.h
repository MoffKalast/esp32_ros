const int FREQUENCY = 20000;

const int MOTOR_1 = 12;
const int MOTOR_2 = 13;

const int STEER_1 = 14;
const int STEER_2 = 15;

const int CH_0 = 5;
const int CH_1 = 6;

const int CH_2 = 7;
const int CH_3 = 8;

int deadman_enabled = false;
unsigned long last_updated = 0;

extern int range;

const int BACKOFF_DIST = 50;
const int CRASH_DIST = 75;

uint8_t current_dir = 0;
uint8_t current_spd = 0;

int super_ultra_late_braking = 0;

void initMotors(){
	pinMode(MOTOR_1, OUTPUT);
	pinMode(MOTOR_2, OUTPUT);
	
	pinMode(STEER_1, OUTPUT);
	pinMode(STEER_2, OUTPUT);
	
	ledcSetup(CH_0, FREQUENCY, 8); //channel, freq, bits
	ledcSetup(CH_1, FREQUENCY, 8); //channel, freq, bits

	ledcSetup(CH_2, FREQUENCY, 8); //channel, freq, bits
	ledcSetup(CH_3, FREQUENCY, 8); //channel, freq, bits

	ledcAttachPin(STEER_2, CH_3); //pin, channel
	ledcAttachPin(STEER_1, CH_2); //pin, channel

	ledcAttachPin(MOTOR_2, CH_1); //pin, channel
	ledcAttachPin(MOTOR_1, CH_0); //pin, channel

	ledcWrite(CH_2, 0);
	ledcWrite(CH_3, 0);
	ledcWrite(CH_0, 0);
	ledcWrite(CH_1, 0);
}

void setSteer(uint8_t dir, uint8_t amount){
	if(dir == 2){
		ledcWrite(CH_2, 0);
		ledcWrite(CH_3, amount);
	}else if (dir == 1){
		ledcWrite(CH_2, amount);
		ledcWrite(CH_3, 0);
	}else{
		ledcWrite(CH_2, 0);
		ledcWrite(CH_3, 0);
	}
	last_updated = millis();
	deadman_enabled = true;
}

void writeForward(uint8_t spd){
	ledcWrite(CH_0, spd);
	ledcWrite(CH_1, 0);
}

void writeReverse(uint8_t spd){
	ledcWrite(CH_0, 0);
	ledcWrite(CH_1, spd);
}

void writeStop(){
	ledcWrite(CH_0, 0);
	ledcWrite(CH_1, 0);
}

void setMotor(uint8_t dir, uint8_t spd){
	last_updated = millis();
	deadman_enabled = true;
	
	if(dir != 2 && range < BACKOFF_DIST){
		return;
	}

	if(spd > 0){
		current_spd = spd;
	}

	if(dir == 2){
		writeReverse(spd);
		current_dir = 2;
	}else if (dir == 1){
		writeForward(spd);
		current_dir = 1;
	}else{
		writeStop();
		current_dir = 0;
	}
}

void updateMotors(){

	if(deadman_enabled && millis() - last_updated > 500){
		setMotor(0,0);
		setSteer(0,0);
		deadman_enabled = false;
	}
	else if(current_dir == 1 && range < CRASH_DIST){
		super_ultra_late_braking = 1;
	}

	if(super_ultra_late_braking == 1){
		if(range > BACKOFF_DIST){
			super_ultra_late_braking = 0;
			writeStop();
		}
		else{
			writeReverse(current_spd);
		}
	}
}
