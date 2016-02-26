//
//  externalData.cpp
//  ExternalData
//
//  Created by Bruno Bachmann on 2016-02-16.
//  Copyright © 2016 Bruno Bachmann. All rights reserved.
//

#include <stdio.h>
#include <stdlib.h>
#include "robot.h"

/*Input pins to be read from*/
#define TEMPERATURE_PIN 3
#define DIST_SENSOR_ECHO_PIN1 4
#define DIST_SENSOR_ECHO_PIN2 5
#define DIST_SENSOR_ECHO_PIN3 6
#define REFLECTIVITY_PIN1 A0
#define REFLECTIVITY_PIN2 A1
#define REFLECTIVITY_PIN3 A2
#define REFLECTIVITY_PIN4 A3

/*Output pins for sensors*/
#define DISTANCE_SENSOR_TRIGGER_PIN1 7
#define DISTANCE_SENSOR_TRIGGER_PIN2 8
#define DISTANCE_SENSOR_TRIGGER_PIN3 9

#pragma mark Public functions
Robot::ExternalData::ExternalData(int receivedTemperaturePin, int receivedNumberOfUltrasonicSensors, int** ultrasonicSensors) {
    // save pins
    temperaturePin = receivedTemperaturePin;
    numberOfUltrasonicSensors = receivedNumberOfUltrasonicSensors;
    ultrasonicSensorPins = ultrasonicSensors;

    // ----> need new parameters?? ----> numberOfReflectivitySensors = receivedNumberOfReflectivitySensors;
	
    // initialize caching variables
    distancesCached = malloc(numberOfUltrasonicSensors * sizeof(bool));
    clearCache();
    lastDistances = malloc(numberOfUltrasonicSensors * sizeof(float));
}

Robot::ExternalData::~ExternalData() {
    // free allocated memory
    free(distancesCached);
    free(lastDistances);
}

Robot::ExternalData::clearCache() {
    temperatureCached = false;
    reflectivityCached = false;
    
    int i;
    for (i = 0; i < numberOfUltrasonicSensors; i++) {
        distancesCached[i] = false;
    }
}

Robot::ExternalData::temperature(bool fresh = false) {
    if (!fresh) {
        if (temperatureCached) {
            return lastTemperature;
        }
    }
    
    lastTemperature = readTemperature();
    temperatureCached = true;
    
    return lastTemperature;
}

Robot::ExternalData::distance(int sensor, bool fresh = false) {
    if (!fresh) {
        if (distancesCached[sensor]) {
            return lastDistances[sensor];
        }
    }
    
    lastDistances[sensor] = readDistance(ultrasonicSensorPins[sensor][0], ultrasonicSensorPins[sensor][1], temperature());
    distancesCached[sensor] = true;
    
    return lastDistances[sensor];
}

Robot::ExternalData::distances(int sensor, bool fresh = false) {
    float *returnValues = malloc(numberOfUltrasonicSensors * sizeof(float));
    
    int i;
    for (i = 0; i < numberOfUltrasonicSensors; i++) {
        if (!fresh && distancesCached[i]) {
            returnValues[i] = lastDistances[i];
        } else {
            lastDistances[i] = readDistance(ultrasonicSensorPins[i][0], ultrasonicSensorPins[i][1], temperature());
            distancesCached[i] = true;
            returnValues[i] = lastDistances[i];
        }
    }
    
    return returnValues;
}

/* JORDAN: I'm putting this here because I didn't want to mess around with the .ino file
 * you created
 *
 * Notes:
 *
 * (1) There are four reflective optical sensors, so we'll need to return an array of floats
 * (2) defined analog pins at the top
 * (3) I have assumed a new parameter in the constructor called numberOfReflectivitSensors
 *
 */
Robot::ExternalData::reflectivity(bool fresh = false) {
	float *returnValues = malloc(numberOfReflectivitySensors * sizeof(float));
	returnValues[0] = (float)analogRead(REFLECTIVITY_PIN1);
	returnValues[1] = (float)analogRead(REFLECTIVITY_PIN2);
	returnValues[2] = (float)analogRead(REFLECTIVITY_PIN3);
	returnValues[3] = (float)analogRead(REFLECTIVITY_PIN4);
    return returnValues;
}

float Robot::ExternalData::readReflectivity(uint8_t sensor) {
	return (float)analogRead(reflectivitySensorPins[i]) 	
}

#pragma mark Private data acquisition functions
float Robot::ExternalData::readTemperature() {
    /*read the voltage on the temperature pin*/
    float voltage = (float)analogRead(temperaturePin);
    /*scale it, taking into account the arduino's return range and the sensor's specs*/
    return (voltage * 500.0) / 1023.0;
}

void Robot::ExternalData::pulseOut(uint8_t pin, int microseconds) {
    // set the pin to high
    digitalWrite(pin, HIGH);
    // wait for the prescribed time
    delayMicroseconds(microseconds);
    // set the pin to low
    digitalWrite(pin, LOW);
}

float Robot::ExternalData::readDistance(uint8_t triggerPin, uint8_t echoPin, float temperature) {
    // send the trigger pulse
    pulseOut(triggerPin, 10);
    // read the response pulse
    unsigned long pulseWidth = pulseIn(echoPin, HIGH);
    // compute the speed of sound
    float speedOfSound = 20000.0 / (331.5 + (0.6 * temperature));
    // compute the distance
    return ((float)pulseWidth) / (speedOfSound);
}
