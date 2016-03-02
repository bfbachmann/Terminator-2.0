// minimum and maximum distances to be read from the ultrasonic sensors, in cm
#define DIST_MAX 200
#define DIST_MIN 0

#define DIST_SENSOR_1_ECHO_PIN 4
#define DIST_SENSOR_2_ECHO_PIN 7
#define DIST_SENSOR_3_ECHO_PIN 10

#define DIST_SENSOR_1_TRIGGER_PIN 3
#define DIST_SENSOR_2_TRIGGER_PIN 8
#define DIST_SENSOR_3_TRIGGER_PIN 9
 
#pragma mark Initializers

ExternalData::ExternalData(int receivedTemperaturePin, int receivedNumberOfUltrasonicSensors, uint8_t** ultrasonicSensors, int receivedNumberOfReflectivitySensors, uint8_t* reflectivitySensors, int modePin) {
	// save pins
	temperaturePin = receivedTemperaturePin;
  mode_pin = modePin;
	
	numberOfUltrasonicSensors = receivedNumberOfUltrasonicSensors;
	ultrasonicSensorPins = ultrasonicSensors;
	
	numberOfReflectivitySensors = receivedNumberOfReflectivitySensors;
	reflectivitySensorPins = reflectivitySensors;
    
	// initialize caching variables
	distancesCached = (bool*)malloc(numberOfUltrasonicSensors * sizeof(bool));
	reflectivitiesCached = (bool*)malloc(numberOfReflectivitySensors * sizeof(bool));
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
  pinMode(mode_pin, INPUT);

	int i;
	for (i = 0; i < numberOfUltrasonicSensors; i++) {
		pinMode(ultrasonicSensorPins[i][0], OUTPUT);
		pinMode(ultrasonicSensorPins[i][1], INPUT);
	}
}

void ExternalData::clearCache() {
    temperatureCached = false;
    
    int i;
    for (i = 0; i < numberOfUltrasonicSensors; i++) {
        distancesCached[i] = false;
    }
		
		for (i = 0; i < numberOfReflectivitySensors; i++) {
			reflectivitiesCached[i] = false;
		}
}

#pragma mark Public data acquisition functions

float ExternalData::temperature(bool fresh) {
    if (!fresh) {
        if (temperatureCached) {
            return lastTemperature;
        }
    }
    
    lastTemperature = read_temperature();
    temperatureCached = true;
    
    return lastTemperature;
}

float ExternalData::distance(int sensor, bool fresh) {
    if (!fresh) {
        if (distancesCached[sensor]) {
            return lastDistances[sensor];
        }
    }
    
    lastDistances[sensor] = readDistance(sensor, read_temperature());
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

float *ExternalData::reflectivity(bool fresh) {
  float *returnValues = (float*)malloc(numberOfReflectivitySensors * sizeof(float));
  // get reading from each refectivity sensor
  for (int i = 0; i < numberOfReflectivitySensors; i++) {
    returnValues[i] = analogRead(reflectivitySensors[i]);
  }
  return returnValues;
}

uint8_t ExternalData::mode() {
  return digitalRead(mode_pin);
}

#pragma mark Private functions
        
float ExternalData::read_temperature() {
	/*read the voltage on the temperature pin*/
	float voltage = (float)analogRead(temperaturePin);
	/*scale it, taking into account the arduino's return range and the sensor's specs*/
 Serial.print("Temperature: ");
 Serial.println((voltage * 500.0) / 1023.0);
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
    
float ExternalData::readDistance(int sensor, float temperature) {
	// send the trigger pulse
	pulseOut(ultrasonicSensorPins[sensor][0], 10);
	// read the response pulse
	unsigned long pulseWidth = pulseIn(ultrasonicSensorPins[sensor][1], HIGH);
  Serial.print("Pulse width: ");
  Serial.println(pulseWidth);
	// compute the speed of sound
	float speedOfSound = 20000.0 / (331.5 + (0.6 * temperature));
	// compute the distance
	float distance = ((float)pulseWidth) / (speedOfSound);
	//return max value if no objects are detected in range
	if (distance > DIST_MAX) {
		return DIST_MAX;
	}
 Serial.print("Distance: ");
 Serial.println(distance);
	return distance;
}


