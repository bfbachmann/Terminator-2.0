#include <Servo.h>

#include "Robot.h"

#define TEMPERATURE_SENSOR_PIN A5

// ultrasonic sensors
#define NUMBER_OF_ULTRASONIC_SENSORS 2

#define DIST_SENSOR_1_ECHO_PIN 2
#define DIST_SENSOR_2_ECHO_PIN 8
#define DIST_SENSOR_1_TRIGGER_PIN 3
#define DIST_SENSOR_2_TRIGGER_PIN 9

// servo to orient the rotatable distance sensor
#define SERVO_PIN 9

// mode swtich
#define MODE_SWITCH_PIN 13

// reflectivity sensors
#define NUMBER_OF_REFLECTIVITY_SENSORS 4

#define REFLECTIVITY_SENSOR_1_PIN A0
#define REFLECTIVITY_SENSOR_2_PIN A1
#define REFLECTIVITY_SENSOR_3_PIN A2
#define REFLECTIVITY_SENSOR_4_PIN A3

// variables required to initialize ExternalData
uint8_t ultrasonicSensorPins[4] = {
	DIST_SENSOR_1_TRIGGER_PIN, DIST_SENSOR_1_ECHO_PIN,
	DIST_SENSOR_2_TRIGGER_PIN, DIST_SENSOR_2_ECHO_PIN
};

uint8_t reflectivitySensors[4] = {
	REFLECTIVITY_SENSOR_1_PIN,
	REFLECTIVITY_SENSOR_2_PIN,
	REFLECTIVITY_SENSOR_3_PIN,
	REFLECTIVITY_SENSOR_4_PIN
};

ExternalData externalData(TEMPERATURE_SENSOR_PIN, NUMBER_OF_ULTRASONIC_SENSORS, ultrasonicSensorPins, NUMBER_OF_REFLECTIVITY_SENSORS, reflectivitySensors, MODE_SWITCH_PIN);
Control control((uint8_t)SERVO_PIN);
AI ai(&externalData, &control);

State state;
Vector destination;

void setup() {
   Serial.begin(9600);
   externalData.initializePins();
   control.attachRangeFinder();
   control.initializePins();
   state.x = 0;
   state.y = 0;
   state.v = 61;
   state.w = 0;
   state.dt = 1;
   state.heading = M_PI/2;
   state.l_PWM = 0;
   state.r_PWM = 0;
   destination.x = 0;
   destination.y = -100;
   pinMode(A0, INPUT);
   pinMode(A1, INPUT);
   pinMode(A2, INPUT);
   pinMode(A3, INPUT);
   delay(1000);
   Serial.println("Done setting up");
}

void loop() {
      float *reflectivities = externalData.reflectivity(true);
      control.followLine(reflectivities, &state);
      free(reflectivities);
	// Serial.println(externalData.distance(0,true));
}
