#include <Servo.h>

#include "Robot.h"

#define TEMPERATURE_SENSOR_PIN 2

// ultrasonic sensors
#define NUMBER_OF_ULTRASONIC_SENSORS 3

#define DIST_SENSOR_1_ECHO_PIN 12
#define DIST_SENSOR_2_ECHO_PIN 7
#define DIST_SENSOR_3_ECHO_PIN 6

#define DIST_SENSOR_1_TRIGGER_PIN 11
#define DIST_SENSOR_2_TRIGGER_PIN 8
#define DIST_SENSOR_3_TRIGGER_PIN 9

// servo to orient the rotatable distance sensor
#define SERVO_PIN 5

// reflectivity sensors
#define NUMBER_OF_REFLECTIVITY_SENSORS 4

#define REFLECTIVITY_SENSOR_1_PIN 0
#define REFLECTIVITY_SENSOR_2_PIN 0
#define REFLECTIVITY_SENSOR_3_PIN 0
#define REFLECTIVITY_SENSOR_4_PIN 0

// variables required to initialize ExternalData
uint8_t ultrasonicSensorPins[3][2] = {
	{DIST_SENSOR_1_TRIGGER_PIN, DIST_SENSOR_1_ECHO_PIN},
	{DIST_SENSOR_1_TRIGGER_PIN, DIST_SENSOR_1_ECHO_PIN},
	{DIST_SENSOR_1_TRIGGER_PIN, DIST_SENSOR_1_ECHO_PIN}
};

uint8_t reflectivitySensors[4] = {
	REFLECTIVITY_SENSOR_1_PIN,
	REFLECTIVITY_SENSOR_2_PIN,
	REFLECTIVITY_SENSOR_3_PIN,
	REFLECTIVITY_SENSOR_4_PIN
};

ExternalData externalData(TEMPERATURE_SENSOR_PIN, NUMBER_OF_ULTRASONIC_SENSORS, (uint8_t**)ultrasonicSensorPins, NUMBER_OF_REFLECTIVITY_SENSORS, (uint8_t*)reflectivitySensors);
Control control((uint8_t)0, (uint8_t)0, (uint8_t)0, (uint8_t)0, (uint8_t)SERVO_PIN);

void setup() {
	externalData.initializePins();
	Serial.begin(9600);
}

void loop() {
	
}
