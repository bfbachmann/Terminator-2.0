// minimum and maximum distances to be read from the ultrasonic sensors, in cm
#define DIST_MAX 200
#define DIST_MIN 0

// how long a temperature value is considered fresh in milliseconds 
#define TEMPERATURE_CACHE_AGE 2000

#pragma mark Initializers

ExternalData::ExternalData(int numberOfUltrasonicSensors, uint8_t ultrasonicSensors[], int numberOfReflectivitySensors, uint8_t reflectivitySensors[], int modePin) {
	// save pins
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
	
	// initialize the temperature to twenty so if we can't ever read from the slave we have a value
	_lastTemperature = 20.0;
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
  pinMode(_modePin, INPUT_PULLUP);

	int i;
	for (i = 0; i < _numberOfUltrasonicSensors; i++) {
		pinMode(_ultrasonicSensorPins[2 * i], OUTPUT);
		pinMode(_ultrasonicSensorPins[(2 * i) + 1], INPUT);
	}
	
	for (i = 0; i < _numberOfReflectivitySensors; i++) {
		pinMode(_reflectivitySensorPins[i], INPUT);
	}
}

void ExternalData::clearCache() {
    _lastTemperatureTimestamp = 0;
    
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
			// if our last reading is less than two seconds old, return it
        if ((_lastTemperatureTimestamp + TEMPERATURE_CACHE_AGE) < millis()) {
            return _lastTemperature;
        }
    }

		float newTemperature = _readTemperature();
		
		if (newTemperature != INFINITY) {
			Serial.print("Valid temperature recieved: ");
			Serial.println(newTemperature);
	    _lastTemperature = newTemperature;
	    _lastTemperatureTimestamp = millis();
		}
    
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

float ExternalData::reflectivity(int sensor, bool fresh) {
  if (!fresh) {
      if (_reflectivitiesCached[sensor]) {
          return _lastReflectivities[sensor];
      }
  }
  
  _lastReflectivities[sensor] = analogRead(_reflectivitySensorPins[sensor]);
  _reflectivitiesCached[sensor] = true;
  
  return _lastReflectivities[sensor];
}

float *ExternalData::reflectivities(bool fresh) {
  float *returnValues = (float*)malloc(_numberOfReflectivitySensors * sizeof(float));
  // get reading from each refectivity sensor
  for (int i = 0; i < _numberOfReflectivitySensors; i++) {
    returnValues[i] = reflectivity(i, fresh);
  }
  return returnValues;
}

Mode ExternalData::mode() {
	if (digitalRead(_modePin) == HIGH) {
		return FreeDrive;
	} else {
		return FollowLine;
	}
}

#pragma mark Private functions
        
float ExternalData::_readTemperature() {
	// request the temperature from the slave
	Wire.beginTransmission(WIRE_DEVICE);
	Wire.write('t');
	Wire.endTransmission();
	
	// request one byte from slave
	Wire.requestFrom(WIRE_DEVICE, 1);
	
	char receivedByte;
	
	if (Wire.available()) {
		receivedByte = Wire.read();
	} else {
		return INFINITY; // if something goes wrong, return not a number
	}
	
	int voltage = (int)receivedByte;
	
	if (voltage < 0 || voltage > 1023) {
		return INFINITY;
	}

	/*scale it, taking into account the arduino's return range and the sensor's specs*/
	return ((float)voltage * 500.0) / 1023.0;
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


