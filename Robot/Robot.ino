#include <Wire.h>

#include <Servo.h>

#include "Robot.h"

// ultrasonic sensors
#define NUMBER_OF_ULTRASONIC_SENSORS 2

// sensor 0 is mobile, sensor 1 is straight ahead
#define DIST_SENSOR_0_ECHO_PIN 10
#define DIST_SENSOR_1_ECHO_PIN 13
#define DIST_SENSOR_0_TRIGGER_PIN 11
#define DIST_SENSOR_1_TRIGGER_PIN 12

// servo to orient the rotatable distance sensor
#define SERVO_PIN 9

// mode swtich
#define MODE_SWITCH_PIN 8

// reflectivity sensors
#define NUMBER_OF_REFLECTIVITY_SENSORS 4

#define REFLECTIVITY_SENSOR_1_PIN A3 // leftmost sensor
#define REFLECTIVITY_SENSOR_2_PIN A2
#define REFLECTIVITY_SENSOR_3_PIN A1
#define REFLECTIVITY_SENSOR_4_PIN A0 // rightmost sensor

// variables required to initialize ExternalData
uint8_t ultrasonicSensorPins[4] = {
	DIST_SENSOR_0_TRIGGER_PIN, DIST_SENSOR_0_ECHO_PIN,
	DIST_SENSOR_1_TRIGGER_PIN, DIST_SENSOR_1_ECHO_PIN
};

uint8_t reflectivitySensors[4] = {
	REFLECTIVITY_SENSOR_1_PIN,
	REFLECTIVITY_SENSOR_2_PIN,
	REFLECTIVITY_SENSOR_3_PIN,
	REFLECTIVITY_SENSOR_4_PIN
};

ExternalData externalData(NUMBER_OF_ULTRASONIC_SENSORS, ultrasonicSensorPins, NUMBER_OF_REFLECTIVITY_SENSORS, reflectivitySensors, MODE_SWITCH_PIN);
Control control((uint8_t)SERVO_PIN);
AI ai(&externalData, &control);

State state;
Vector destination;

void setup() {
#ifdef DEBUG
	Serial.begin(9600);
#endif
	Wire.begin();
	
	externalData.initializePins();
	control.attachRangeFinder();
	control.initializePins();
	
	control.sendByteToSlave('k');
	delay(1500);
 
	state.x = 0;
	state.y = 0;
	state.v = 61;
	state.w = 0;
	state.dt = 1;
	state.heading = M_PI/2;
	state.l_PWM = 0;
	state.r_PWM = 0;
	destination.x = 0;
	destination.y = 300;
  	
	control.stop();
}

void loop() {
	externalData.clearCache();
	
#ifdef DEBUG
	Mode mode = externalData.mode();

	if (mode == FreeDrive) {
		Serial.println("Free drive");
	} else if (mode == FollowLine) {
		Serial.println("Follow line");
	} else {
		Serial.println("Invaid mode");
	}
#endif
		
  ai.decide(&state);
}
