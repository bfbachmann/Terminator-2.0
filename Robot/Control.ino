/*
Robot differential control system
  
By Chad Lagore
02/13/16
*/

#include <math.h>
#include "Robot.h"

// pins for the motor shield
#define E1 5  
#define M1 4 
#define E2 6                      
#define M2 7 

// Geometric constants of robot vehicle.
// Wheel radius (R) and wheel to wheel length (L).
#define R 6.5
#define L 16

#define SERVO_DELAY_PER_DEGREE 3

#define _USE_MATH_DEFINES

#pragma mark Begin Control implementation

Control::Control(uint8_t receivedRangeFinder) {
	rangeFinderPin = receivedRangeFinder;
}

Control::~Control() {
	// TODO: implement
}

void Control::initializePins() {
	pinMode(E1, OUTPUT);
	pinMode(E2, OUTPUT);
	pinMode(M1, OUTPUT);
	pinMode(M2, OUTPUT);
	pinMode(rangeFinderPin, OUTPUT);

}

void Control::attachRangeFinder() {
	rangeFinderServo.attach(rangeFinderPin);
}


/*  go(State state, Vector destination, bool StopAtDestination = false)
    
Control flow for a single time step in moving the robot towards its
destination. 
*/
void Control::go(State * state, Vector * destination, bool stopAtDestination) {
	float distance, MAX_RPM;
	state->heading = M_PI/2;
	MAX_RPM = 160.0;
	float factor = 8.0;
        
        /* Adjust heading */
        adjustHeading(state, destination, false);
        
	/* Determine desired distance */
	distance = sqrt(destination->x*destination->x + destination->y*destination->y);
	
	/* Implement desired wheel speeds */
	state->l_PWM = state->v / MAX_SPEED * 255;
	state->r_PWM = state->v / MAX_SPEED * 255;
	wheelControl(state, true, true);
        
	/* Drive straight  */
        if(stopAtDestination) {
      	    while(distance > 2) {
      		distance = distance - (factor * MAX_RPM / 60000 * 2 * R * M_PI/2);
      		delay(10);
      	    }
            stop();
        }
        
}

/*  Applies PWM signal to wheel motors. This function will require access
*  to pin numbers.
*/
void Control::wheelControl(State * state, bool left, bool right) {
        digitalWrite(M1, right ? HIGH : LOW); 
        digitalWrite(M2, left ? HIGH : LOW);
        analogWrite(E1, state->r_PWM); 
        analogWrite(E2, 0.98*state->l_PWM);
}

void Control::adjustHeading(State * state, Vector * destination, bool hard) {
        float desired_heading, error, arc, MAX_RPM;
	MAX_RPM = 160.0;
	float factor = 10.0;
        state->heading = M_PI/2;
        
	arc = 3.0 * MAX_RPM / 60000 * 2 * M_PI * R * 2;
        arc = hard ? arc : arc / 2;
        
	/* Determine desired heading, velocity and angular velocity */
	desired_heading = atan2(destination->y, destination->x); 
	error = state->heading - desired_heading;
	error = atan2(sin(error), cos(error));
	// Serial.println(error);
        
	/* Correct heading */
	while(abs(error) > 0.05) {
                if(hard) {
                    state->l_PWM = 255;
		    state->r_PWM = 255;
		    wheelControl(state, error < 0, error > 0);
                }
                else { 
                    state->l_PWM = error < 0 ? 0 : 255;
                    state->r_PWM = error > 0 ? 0 : 255;
                    wheelControl(state, true, true); 
                }
		delay(10);
		state->heading = (error > 0) ? state->heading - arc/L : state->heading + arc/L;
		error = state->heading - desired_heading;
		error = atan2(sin(error), cos(error));
		// Serial.println(error);
	}
}

/*
* Set the servo motor to point in the direction 
* specified by the orientation parameter.
*/
void Control::orientRangeFinder(int orientation) {
	if (_currentRangeFinderOrientation != orientation) {

		rangeFinderServo.write(orientation);
		// give the servo a chance to actually move - 3ms per degree moved
		delay(SERVO_DELAY_PER_DEGREE * abs(_currentRangeFinderOrientation - orientation));
		_currentRangeFinderOrientation = orientation;
	}
}


/*
*  Completes one time step in the control flow of a line-following behaviour.
*/
void Control::followLine(State * state) {
	const float factor = 2.0;
	digitalWrite(M1, HIGH); digitalWrite(M2, HIGH);
	analogWrite(E1, factor*state->l_PWM); analogWrite(E2, factor*state->r_PWM);
}

void Control::stop() {
	digitalWrite(M1, HIGH); digitalWrite(M2, HIGH);
	analogWrite(E1, 0); analogWrite(E2, 0);
}

void Control::slowDown(State *state) {
	state->v = state->v - 2;
	
	if (state->v < 25) {
		state->v = 25;
	}
	
	// Serial.print("New velocity: ");
	// Serial.println(state->v);
	
	Vector destination;
	destination.x = 0;
	destination.y = 1;
	go(state, &destination, false);
	//decrement current wheel speed by some about and write this new value back to the wheels
}

void Control::sendByteToSlave(char command) {
	if (command == _last_command) {
		if (command == 'b' || command == 'd') {
			return;
		}
	}
	
	_last_command = command;
	
	// write to the first device
	Wire.beginTransmission(WIRE_DEVICE_1);
	Wire.write(command);
	Wire.endTransmission();
	
	// write to the second device
	Wire.beginTransmission(WIRE_DEVICE_2);
	Wire.write(command);
	Wire.endTransmission();
}

