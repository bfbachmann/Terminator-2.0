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
#define SERVO_PIN 9

// mode swtich
#define MODE_SWITCH_PIN 3

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

ExternalData externalData(TEMPERATURE_SENSOR_PIN, NUMBER_OF_ULTRASONIC_SENSORS, (uint8_t**)ultrasonicSensorPins, NUMBER_OF_REFLECTIVITY_SENSORS, (uint8_t*)reflectivitySensors, MODE_SWITCH_PIN);
Control control((uint8_t)0, (uint8_t)0, (uint8_t)0, (uint8_t)0, (uint8_t)SERVO_PIN);
<<<<<<< HEAD
AI ai(externalData, control);

void setup() {
	externalData.initializePins();
  pinMode(SERVO_PIN, OUTPUT);
  control.attachRangeFinder();
	Serial.begin(9600);
}

void loop() {

=======
State state;
Vector destination;

void setup() {
   externalData.initializePins();
   state.x = 0;
   state.y = 0;
   state.v = 0;
   state.w = 0;
   state.dt = 1;
   state.heading = M_PI/2;
   destination.x = 0;
   destination.y = 100;
   Serial.begin(9600);
   pinMode(A0, INPUT);
   pinMode(A1, INPUT);
   pinMode(A2, INPUT);
   delay(1000);
   
}

void loop() {
  long start = 0;
  float s1 = analogRead(A0);
  float s2 = analogRead(A1);
  float s3 = analogRead(A2);
  bool atDestination = false;
  /*
  Serial.print(s1);   Serial.print(",");
  Serial.print(s2);Serial.print(",");
  Serial.println(s3);
  */
  // control.followLine(s1,s2,s3);
  while(!atDestination) {
    start = millis();
    atDestination = control.go(&state, &destination, true);
    state.dt = millis() - start;
    // Serial.println(state.dt);
    // Serial.print(state.x); Serial.print(","); Serial.println(state.y);
  }

  delay(10000);
  
>>>>>>> 931d909c678d6fb172633b78d01572820f52a1d0
}
