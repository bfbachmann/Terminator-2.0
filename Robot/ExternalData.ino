// minimum and maximum distances to be read from the ultrasonic sensors, in cm
#define DIST_MAX 200
#define DIST_MIN 0
 
#pragma mark Initializers

ExternalData::ExternalData(int receivedTemperaturePin, int receivedNumberOfUltrasonicSensors, uint8_t** ultrasonicSensors, int receivedNumberOfReflectivitySensors, uint8_t* reflectivitySensors) {
	// save pins
	temperaturePin = receivedTemperaturePin;
	
	numberOfUltrasonicSensors = receivedNumberOfUltrasonicSensors;
	ultrasonicSensorPins = ultrasonicSensors;
	
	numberOfReflectivitySensors = receivedNumberOfReflectivitySensors;
	reflectivitySensorPins = reflectivitySensors;
    
	// initialize caching variables
	distancesCached = (bool*)malloc(numberOfUltrasonicSensors * sizeof(bool));
	clearCache();
	lastDistances = (float*)malloc(numberOfUltrasonicSensors * sizeof(float));
}

ExternalData::~ExternalData() {
	// free allocated memory
	free(distancesCached);
	free(lastDistances);
}

#pragma mark Utility functions

void ExternalData::initializePins() {
	// initialize pins
	pinMode(temperaturePin, INPUT);

	int i;
	for (i = 0; i < numberOfUltrasonicSensors; i++) {
		pinMode(ultrasonicSensorPins[i][0], OUTPUT);
		pinMode(ultrasonicSensorPins[i][1], INPUT);
	}
}

void ExternalData::clearCache() {
    temperatureCached = false;
    reflectivityCached = false;
    
    int i;
    for (i = 0; i < numberOfUltrasonicSensors; i++) {
        distancesCached[i] = false;
    }
}

#pragma mark Public data acquisition functions

float ExternalData::temperature(bool fresh) {
    if (!fresh) {
        if (temperatureCached) {
            return lastTemperature;
        }
    }
    
    lastTemperature = readTemperature();
    temperatureCached = true;
    
    return lastTemperature;
}

float ExternalData::distance(int sensor, bool fresh) {
    if (!fresh) {
        if (distancesCached[sensor]) {
            return lastDistances[sensor];
        }
    }
    
    lastDistances[sensor] = readDistance(ultrasonicSensorPins[sensor][0], ultrasonicSensorPins[sensor][1], temperature());
    distancesCached[sensor] = true;
    
    return lastDistances[sensor];
}

float *ExternalData::distances(bool fresh) {
	float *returnValues = (float*)malloc(numberOfUltrasonicSensors * sizeof(float));
    
    int i;
    for (i = 0; i < numberOfUltrasonicSensors; i++) {
        returnValues[i] = distance(i, fresh);
    }
  
    return returnValues;
}


// float ExternalData::get_distance_at_angle(int angle) {
// 	control.orientRangeFinder(angle);
// 	return readDistance(ultrasonicSensors[1][0], ultrasonicSensors[1][1], read_tempterature());
// }

#pragma mark Private functions
        
float ExternalData::readTemperature() {
	/*read the voltage on the temperature pin*/
	float voltage = (float)analogRead(temperaturePin);
	/*scale it, taking into account the arduino's return range and the sensor's specs*/
	return (voltage * 500.0) / 1023.0;
}
    
void ExternalData::pulseOut(uint8_t pin, int microseconds) {
	// set the pin to high
	digitalWrite(pin, HIGH);
	// wait for the prescribed time
	delayMicroseconds(microseconds);
	// set the pin to low
	digitalWrite(pin, LOW);
}
    
float ExternalData::readDistance(uint8_t triggerPin, uint8_t echoPin, float temperature) {
	// send the trigger pulse
	pulseOut(triggerPin, 10);
	// read the response pulse
	unsigned long pulseWidth = pulseIn(echoPin, HIGH);
	// compute the speed of sound
	float speedOfSound = 20000.0 / (331.5 + (0.6 * temperature));
	// compute the distance
	float distance = ((float)pulseWidth) / (speedOfSound);
	//return max value if no objects are detected in range
	if (distance > DIST_MAX) {
		return DIST_MAX;
	}
	return distance;
}
