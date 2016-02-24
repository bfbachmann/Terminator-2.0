//
//  externalData.cpp
//  ExternalData
//
//  Created by Bruno Bachmann on 2016-02-16.
//  Copyright © 2016 Bruno Bachmann. All rights reserved.
//

#include <stdio.h>
#include <stdlib.h>

/*Input pins to be read from*/
#define TEMPERATURE_PIN 3
#define DIST_SENSOR_ECHO_PIN1 4
#define DIST_SENSOR_ECHO_PIN2 5
#define DIST_SENSOR_ECHO_PIN3 6

/*Output pins for sensors*/
#define DISTANCE_SENSOR_TRIGGER_PIN1 7
#define DISTANCE_SENSOR_TRIGGER_PIN2 8
#define DISTANCE_SENSOR_TRIGGER_PIN3 9

class ExternalData {
    
public:
    
    
private:
    
    float readTemperature() {
        /*read the voltage on the temperature pin*/
        float voltage = (float)analogRead(TEMPERATURE_PIN);
        /*scale it, taking into account the arduino's return range and the sensor's specs*/
        return (voltage * 500.0) / 1023.0;
    }
    
    void pulseOut(uint8_t pin, int microseconds) {
        // set the pin to high
        digitalWrite(pin, HIGH);
        // wait for the prescribed time
        delayMicroseconds(microseconds);
        // set the pin to low
        digitalWrite(pin, LOW);
    }
    
    float readDistance(uint8_t sensorPin, float temperature) {
        // send the trigger pulse
        pulseOut(sensorPin, 10);
        // read the response pulse
        unsigned long pulseWidth = pulseIn(DISTANCE_SENSOR_ECHO_PIN, HIGH);
        // compute the speed of sound
        float speedOfSound = 10000.0 / (331.5 + (0.6 * temperature));
        // compute the distance
        return ((float)pulseWidth) / (2 * speedOfSound);
    }
    
};
