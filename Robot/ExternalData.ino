// minimum and maximum distances to be read from the ultrasonic sensors, in cm
#define DIST_MAX 200
#define DIST_MIN 0
 
#pragma mark Initializers

ExternalData::ExternalData(int temperaturePin, int numberOfUltrasonicSensors, uint8_t ultrasonicSensors[], int numberOfReflectivitySensors, uint8_t reflectivitySensors[], int modePin) {
	// save pins
	_temperaturePin = temperaturePin;
  _modePin = modePin;
	
	_numberOfUltrasonicSensors = numberOfUltrasonicSensors;
	_ultrasonicSensorPins = ultrasonicSensors;
	
	_numberOfReflectivitySensors = numberOfReflectivitySensors;
	_reflectivitySensorPins = reflectivitySensors;
    
	// initialize caching variables
	_distancesCached = (bool*)malloc(_numberOfUltrasonicSensors * sizeof(bool));
	_reflectivitiesCached = (bool*)malloc(_numberOfReflectivitySensors * sizeof(bool));
	clearCache();
	_lastDistances = (float*)malloc(_numberOfUltrasonicSensors * sizeof(float));
	_lastReflectivities = (float*)malloc(_numberOfReflectivitySensors * sizeof(float));
}

ExternalData::~ExternalData() {
	// free allocated memory
	free(_distancesCached);
	free(_reflectivitiesCached);
	free(_lastDistances);
	free(_lastReflectivities);
}

#pragma mark Utility functions

void ExternalData::initializePins() {
	// initialize pins
	pinMode(_temperaturePin, INPUT);
  pinMode(_modePin, INPUT);

	int i;
	for (i = 0; i < _numberOfUltrasonicSensors; i++) {
		pinMode(_ultrasonicSensorPins[2 * i], OUTPUT);
		pinMode(_ultrasonicSensorPins[(2 * i) + 1], INPUT);
	}
}

void ExternalData::clearCache() {
    _temperatureCached = false;
    
    int i;
    for (i = 0; i < _numberOfUltrasonicSensors; i++) {
    	_distancesCached[i] = false;
    }
		
		for (i = 0; i < _numberOfReflectivitySensors; i++) {
			_reflectivitiesCached[i] = false;
		}
}

#pragma mark Public data acquisition functions

float ExternalData::temperature(bool fresh) {
    if (!fresh) {
        if (_temperatureCached) {
            return _lastTemperature;
        }
    }

    _lastTemperature = _readTemperature();
    _temperatureCached = true;
    
    return _lastTemperature;
}

float ExternalData::distance(int sensor, bool fresh) {
    if (!fresh) {
        if (_distancesCached[sensor]) {
            return _lastDistances[sensor];
        }
    }
    
    _lastDistances[sensor] = _readDistance(sensor, temperature());
    _distancesCached[sensor] = true;
    
    return _lastDistances[sensor];
}

float *ExternalData::distances(bool fresh) {
	float *returnValues = (float*)malloc(_numberOfUltrasonicSensors * sizeof(float));
    
    int i;
    for (i = 0; i < _numberOfUltrasonicSensors; i++) {
        returnValues[i] = distance(i, fresh);
    }
  
    return returnValues;
}

float *ExternalData::reflectivity(bool fresh) {
  float *returnValues = (float*)malloc(_numberOfReflectivitySensors * sizeof(float));
  // get reading from each refectivity sensor
  for (int i = 0; i < _numberOfReflectivitySensors; i++) {
    returnValues[i] = analogRead(_reflectivitySensorPins[i]);
  }
  return returnValues;
}

uint8_t ExternalData::mode() {
  return digitalRead(_modePin);
}

#pragma mark Private functions
        
float ExternalData::_readTemperature() {
	/*read the voltage on the temperature pin*/
	float voltage = (float)analogRead(_temperaturePin);
	/*scale it, taking into account the arduino's return range and the sensor's specs*/
	return (voltage * 500.0) / 1023.0;
}
    
void ExternalData::_pulseOut(uint8_t pin, int microseconds) {

	// set the pin to high
	digitalWrite(pin, HIGH);
	// wait for the prescribed time
	delayMicroseconds(microseconds);
	// set the pin to low
	digitalWrite(pin, LOW);
}
    
float ExternalData::_readDistance(int sensor, float temperature) {
	// send the trigger pulse
	_pulseOut(_ultrasonicSensorPins[sensor * 2], 10);
	// read the response pulse
  
	unsigned long pulseWidth = pulseIn(_ultrasonicSensorPins[(sensor * 2) + 1], HIGH);
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


