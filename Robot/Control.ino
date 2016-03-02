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
bool Control::go(State * state, Vector * destination, bool stopAtDestination) {
	float v, w, desired_heading, right_w, left_w, error, velocity, distance;
        Vector goal;
        
        /* Create goal vector */ 
        goal.x = (destination->x - state->x);
        goal.y = (destination->y - state->y);
        
        /* Determine desired heading, velocity and angular velocity */
        distance = sqrt(goal.x*goal.x + goal.y*goal.y);
        state->v = MAX_SPEED;
        desired_heading = atan2(goal.y,goal.x); 
        error = desired_heading - state->heading;
        error = atan2(sin(error), cos(error));
        state->w = k_p * error + k_i * error * state->dt;
        // Serial.println(error);
        
        /* Calculate wheel velocities from craft velocity */
        left_w = wheelVelocity(state->w,state->v,0);
        right_w = wheelVelocity(state->w,state->v,1);
        //Serial.print(state->x); Serial.print(",");Serial.println(state->y);
        
        /* Implement wheel velocities */
        wheelControl(state, left_w, right_w);
        // Serial.print(left_w); Serial.print(",");Serial.println(right_w);
        // Serial.println(distance);
      
        /* Update state variables */
        calculateOdometry(state, left_w, right_w);
      
        /* Return true if within threshold of destination */
        return (distance < 5);
        
}

/*  Computes left or right wheel angular velocity given craft velocity and
*  angular velocity. Documentation on these formulae is availabile in the
*  accompanying report.
*  
*  Parameters: angular_velocity, velocity and wheel. Wheel determines
*  which wheel is being calculated. 0 is for left, 1 is for right.
*  Returns: A wheel velocity.
*/
float Control::wheelVelocity(float w, float v, int wheel) {
	if(wheel)
		return (2*v + w*L)/(2*R);
	else
		return (2*v - w*L)/(2*R);
}

/*  Applies PWM signal to wheel motors. This function will require access
 *  to pin numbers.
 */
void Control::wheelControl(State * state, float left_w, float right_w) {
  state->l_PWM = left_w/(left_w+right_w)*2*255;
  state->r_PWM = right_w/(left_w+right_w)*2*255;
  
  digitalWrite(M1, HIGH); digitalWrite(M2, HIGH);
  analogWrite(E1, state->r_PWM); analogWrite(E2, state->l_PWM);
 }

/*  Calculates distance travelled in time step and updates state variables.
*  
*  Parameters:   Pointers to State struct, left wheel & right wheel angular 
*          velocities.
* 
*/
void Control::calculateOdometry(State * state, float left_w, float right_w) {
	float factor = 2;
        // Serial.println(factor);
        float MAX_RPM = 160 ;    // Maximum wheel RPM in ms.
        float circumference = 2 * M_PI * R;
        float r_distance = circumference * ((float)state->r_PWM / 255.0) * MAX_RPM * state->dt / 60000 / factor;
        float l_distance = circumference * ((float)state->l_PWM / 255.0) * MAX_RPM * state->dt / 60000 / factor;
        float distance = (l_distance + r_distance) / 2;
    
        // Serial.println(MAX_RPms);
        // Serial.print(r_distance); Serial.print(","); Serial.println(l_distance);
    
	state->x = state->x + distance * cos(state->heading);
	state->y = state->y + distance * sin(state->heading);
	state->heading = state->heading + (r_distance-l_distance)/L;
        Serial.println(state->heading);
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
  float factor = 1.4;
  int left, right = 0;

  // turn left
  if(reflectivities[0] > thresh) {
    state->l_PWM = 55;
    state->r_PWM = 100;
  // or turn right
  } else if(reflectivities[3] > thresh) {
    state->l_PWM = 100;
    state->r_PWM = 55;
    // hard turn, depending on previous situation
  } else if(reflectivities[0] < thresh && reflectivities[1] < thresh && reflectivities[2] < thresh && reflectivities[3] < thresh) {
    if (state->r_PWM > state->l_PWM) { state->l_PWM = 30; }
    else { state->r_PWM = 30; }
    // or drive straight
  } else {
     left = 100;
     right = 100;
  }

  // Serial.print(reflectivities[0]);Serial.print(',');Serial.print(reflectivities[1]);Serial.print(',');
  // Serial.print(reflectivities[2]);Serial.print(',');Serial.println(reflectivities[3]);

  digitalWrite(M1, HIGH); digitalWrite(M2, HIGH);
  analogWrite(E1, factor*state->l_PWM); analogWrite(E2, factor*state->r_PWM);
  Serial.print(factor*state->l_PWM);   Serial.println(factor*state->r_PWM); 
}

void Control::stop() {
      digitalWrite(M1, HIGH); digitalWrite(M2, HIGH);
      analogWrite(E1, 0); analogWrite(E2, 0);
}

void Control::slowDown(State *state, float aggressiveness) {
  float v = state->v;
	state->v = v * aggressiveness;
  Vector destination;
  destination.x = 0;
  destination.y = 1;
  go(state, &destination, false);
  //decrement current wheel speed by some about and write this new value back to the wheels
}

