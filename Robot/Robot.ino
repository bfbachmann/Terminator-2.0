#include <Servo.h>
#include "Robot.h"

#define TEMPERATURE_SENSOR_PIN 2

#define NUMBER_OF_ULTRASONIC_SENSORS 3

//these are global so they can be set up in setup() and used in loop()

int ultrasonicSensorPins[3][2] = {{1,2}, {3,4}, {5,6}};
ExternalData externalData(TEMPERATURE_SENSOR_PIN, NUMBER_OF_ULTRASONIC_SENSORS, (int**)ultrasonicSensorPins);
float data[3] = {0, 0, 0};            //crude representation of the cache

void setup() {
	externalData.initializePins();
	Serial.begin(9600);
}

void loop() {

  
}
