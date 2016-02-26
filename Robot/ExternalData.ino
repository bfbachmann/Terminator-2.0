
/*Input pins to be read from*/
#define TEMPERATURE_PIN 5
#define DIST_SENSOR_ECHO_PIN1 12
#define DIST_SENSOR_ECHO_PIN2 7
#define DIST_SENSOR_ECHO_PIN3 6

/*Output pins for sensors*/
#define DIST_SENSOR_TRIGGER_PIN1 11
#define DIST_SENSOR_TRIGGER_PIN2 8
#define DIST_SENSOR_TRIGGER_PIN3 9

/*the pin for writing to the servo motor*/
#define SERVO_PIN

#define DIST_MAX 200
#define DIST_MIN 0
    
/*
* The constructor for the class
*/
ExternalData::ExternalData(int receivedTemperaturePin, int receivedNumberOfUltrasonicSensors, int** ultrasonicSensors) {
	// save pins
	temperaturePin = receivedTemperaturePin;
	numberOfUltrasonicSensors = receivedNumberOfUltrasonicSensors;
	ultrasonicSensorPins = ultrasonicSensors;
    
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

void ExternalData::initializePins() {
	// initialize pins
	pinMode(temperaturePin, INPUT);

	int i;
	for (i = 0; i < numberOfUltrasonicSensors; i++) {
		pinMode(ultrasonicSensorPins[i][0], OUTPUT);
		pinMode(ultrasonicSensorPins[i][1], INPUT);
	}
}
        
float ExternalData::readTemperature() {
	//Serial.print("calling read_temperature()\n");     //for debugging
	/*read the voltage on the temperature pin*/
	float voltage = (float)analogRead(TEMPERATURE_PIN);
	/*scale it, taking into account the arduino's return range and the sensor's specs*/
	return (voltage * 500.0) / 1023.0;
}
    
void ExternalData::pulseOut(uint8_t pin, int microseconds) {
	//Serial.print("calling pulseOut()\n");         //for debugging
	// set the pin to high
	digitalWrite(pin, HIGH);
	// wait for the prescribed time
	delayMicroseconds(microseconds);
	// set the pin to low
	digitalWrite(pin, LOW);
}
    
float ExternalData::readDistance(uint8_t pulseOutPin, uint8_t pulseInPin, float temperature) {
	//Serial.print("calling readDistance()\n");           //for debugging
	// send the trigger pulse
	pulseOut(pulseOutPin, 10);
	// read the response pulse
	unsigned long pulseWidth = pulseIn(pulseInPin, HIGH);
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

