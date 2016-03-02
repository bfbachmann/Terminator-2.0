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

// PID controller tuning.
#define k_p 1.5
#define k_i 1
#define k_d 0

// Max speed in cm/s. 
#define MAX_SPEED 61

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
	float desired_heading, error, velocity, distance, arc, MAX_RPM;
        state->heading = M_PI/2;
        MAX_RPM = 160.0;
        float factor = 10.0;
        
        arc = 4.2 * MAX_RPM / 60000 * 2 * M_PI * R;
        
        /* Determine desired heading, velocity and angular velocity */
        distance = sqrt(destination->x*destination->x + destination->y*destination->y);
        desired_heading = atan2(destination->y, destination->x); 
        error = state->heading - desired_heading;
        error = atan2(sin(error), cos(error));
        // Serial.println(error);
        
        /* Correct heading */
        while(abs(error) > 0.05) {
            state->l_PWM = (error < 0) ? 255 : 0;
            state->r_PWM = (error > 0) ? 255 : 0;
            wheelControl(state);
            delay(10);
            state->heading = (error > 0) ? state->heading - arc/L : state->heading + arc/L;;
            error = state->heading - desired_heading;
            error = atan2(sin(error), cos(error));
            // Serial.println(error);
        }
        
        /* Implement desired wheel speeds */
            state->l_PWM = state->v / MAX_SPEED * 255;
            state->r_PWM = state->v / MAX_SPEED * 255;
            wheelControl(state);
            // Serial.print(state->l_PWM); Serial.print(","); Serial.println(state->l_PWM);
        
        /* Drive straight  */
        while(distance > 2) {
            distance = distance - (factor * MAX_RPM / 60000 * 2 * R * M_PI/2);
            delay(10);
            // Serial.println(distance);
            // Serial.print(state->l_PWM); Serial.print(","); Serial.println(state->l_PWM);
        }
        
        if(stopAtDestination)
          stop();
        
}

/*  Applies PWM signal to wheel motors. This function will require access
 *  to pin numbers.
 */
void Control::wheelControl(State * state) {
      digitalWrite(M1, HIGH); digitalWrite(M2, HIGH);
      analogWrite(E1, state->r_PWM); analogWrite(E2, state->l_PWM);
 }


/*
 * Set the servo motor to point in the direction 
 * specified by the orientation parameter.
 */
void Control::orientRangeFinder(int orientation) {
	rangeFinderServo.write(orientation);
}
/*
*  Completes one time step in the control flow of a line-following behaviour.
*/
void Control::followLine(float *reflectivities, State * state) {
  const int thresh = 100;
  float factor = 2;

  // turn left
  if(reflectivities[0] > thresh) { state->l_PWM = 75; state->r_PWM = 100;
  // or turn right
  } else if(reflectivities[3] > thresh) { state->l_PWM = 100; state->r_PWM = 75;
  // or use state to remember in which direction the lost line is
  } else if(reflectivities[0] < thresh && reflectivities[1] < thresh && reflectivities[2] < thresh && reflectivities[3] < thresh) {
    if (state->r_PWM > state->l_PWM) { state->l_PWM = 30; }
    else { state->r_PWM = 30; }
  // or else drive straight
  } else { state->r_PWM = 100; state->l_PWM = 100; }
    
  wheelControl(state);
//  Serial.print(factor*state->l_PWM);   Serial.println(factor*state->r_PWM); 
}

void Control::stop() {
      digitalWrite(M1, HIGH); digitalWrite(M2, HIGH);
      analogWrite(E1, 0); analogWrite(E2, 0);
}

void Control::slowDown(State *state, float aggressiveness) {
  //decrement current wheel speed by some about and write this new value back to the wheels
}

