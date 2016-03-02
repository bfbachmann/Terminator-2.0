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

#define REFLECTIVITY_SENSOR_1_PIN A0
#define REFLECTIVITY_SENSOR_2_PIN A1
#define REFLECTIVITY_SENSOR_3_PIN A2
#define REFLECTIVITY_SENSOR_4_PIN A3

#define WIRE_DEVICE 8

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
	Serial.begin(9600);
	Wire.begin();
	
	externalData.initializePins();
	control.attachRangeFinder();
	control.initializePins();
 
	state.x = 0;
	state.y = 0;
	state.v = 0;
	state.w = 0;
	state.dt = 1;
	state.heading = M_PI/2;
	state.l_PWM = 0;
	state.r_PWM = 0;
	destination.x = 10;
	destination.y = 100;
	
	control.stop();
}

void loop() {
	// externalData.clearCache();
	//
	// Mode mode = externalData.mode();
	//
	// if (mode == FreeDrive) {
	// 	Serial.println("Free drive");
	// } else if (mode == FollowLine) {
	// 	Serial.println("Follow line");
	// } else {
	// 	Serial.println("Invaid mode");
	// }
	//
	// Serial.print("Distances:\t");
	// Serial.print(externalData.distance(0));
	// Serial.print('\t');
	// Serial.println(externalData.distance(1));
	//
	// Serial.print("Temperature:\t");
	// Serial.println(externalData.temperature());
	//
	// delay(500);
		
	ai.decide(&state);
}
