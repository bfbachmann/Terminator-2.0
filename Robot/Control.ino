/*
Robot differential control system
  
By Chad Lagore
02/13/16
*/

#include <math.h>
#include "Robot.h"

// Geometric constants of robot vehicle.
// Wheel radius (R) and wheel to wheel length (L).
#define R 6.5
#define L 16

// PID controller tuning.
#define k_p 10
#define k_i 0
#define k_d 0

// Finite time step in ms.
#define dt 1

// Max speed in cm/s. 
#define MAX_SPEED 61

#define _USE_MATH_DEFINES

#pragma mark Begin Control implementation

Control::Control(uint8_t receivedIn1, uint8_t receivedIn2, uint8_t receivedIn3, uint8_t receivedIn4, uint8_t receivedRangeFinder) {
	// save the pins for later
	in1 = receivedIn1;
	in2 = receivedIn2;
	in3 = receivedIn3;
	in4 = receivedIn4;
	rangeFinderPin = receivedRangeFinder;
	
	rangeFinderServo.attach(rangeFinderPin);
}

Control::~Control() {
	// TODO: implement
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
        // Serial.print(goal.x); Serial.print(","); Serial.println(goal.y);
        
	/* Determine desired heading, velocity and angular velocity */
	distance = sqrt(goal.x * goal.x + goal.y * goal.y);
	state->v = distance > 10 ? MAX_SPEED : 0;
	desired_heading = atan2(goal.y,goal.x); 
	error = desired_heading - state->heading;
	error = atan2(sin(error), cos(error));
        // Serial.print(state->heading); Serial.print(","); Serial.print(desired_heading); Serial.print(","); Serial.println(error); 
	state->w = k_p * error + k_i * error * dt;
        // Serial.println(state->w);
    
	/* Calculate wheel velocities from craft velocity */
	left_w = wheelVelocity(state->w,state->v,0);
	right_w = wheelVelocity(state->w,state->v,1);
        // Serial.print(left_w); Serial.print(","); Serial.println(right_w);
    
	/* Implement wheel velocities */
	wheelControl(state, left_w, right_w);
        // Serial.print(state->l_PWM); Serial.print(","); Serial.println(state->r_PWM);
        
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
    int E1 = 5;  
    int M1 = 4; 
    int E2 = 6;                      
    int M2 = 7;     
    
    state->l_PWM = (left_w >= MAX_SPEED/R) ? 255 : ceil(left_w*255/(MAX_SPEED/R));
    state->r_PWM = (right_w >= MAX_SPEED/R) ? 255 : ceil(right_w*255/(MAX_SPEED/R));
    
    digitalWrite(M1, HIGH); digitalWrite(M2, HIGH);
    analogWrite(E1, state->r_PWM); analogWrite(E2, state->r_PWM);
  }

/*  Calculates distance travelled in time step and updates state variables.
*  
*  Parameters:   Pointers to State struct, left wheel & right wheel angular 
*          velocities.
* 
*/
void Control::calculateOdometry(State * state, float left_w, float right_w) {
	// float l_distance = left_w * dt * R / 1000;
	// float r_distance = right_w * dt * R / 1000;
        float MAX_RPM = 160 ;    // Maximum wheel RPM in ms.
        double circumference = 2 * M_PI * R;
        float r_distance = circumference * ((float)state->r_PWM / 255.0) * MAX_RPM * dt / 6000;
        float l_distance = circumference * ((float)state->l_PWM / 255.0) * MAX_RPM * dt / 6000;
	float distance = (l_distance + r_distance) / 2;
    
        // Serial.println(MAX_RPms);
        // Serial.print(r_distance); Serial.print(","); Serial.println(l_distance);
    
	state->x = state->x + distance * cos(state->heading);
	state->y = state->y + distance * sin(state->heading);
	state->heading = state->heading + (r_distance-l_distance)/L;
}

void Control::orientRangeFinder(int orientation) {
	rangeFinderServo.write(orientation);
}
/*
*  Completes one time step in the control flow of a line-following behaviour.
*/
void Control::followLine(float *reflectivities) {
  const int thresh = 100;
  float factor = 1.2;
  int E1 = 5;
  int E2 = 6;
  int M1 = 4;
  int M2 = 7;
  int left, right = 0;

  // turn left
  if(reflectivities[0] > thresh) {
    left = 65;
    right = 100;
  // or turn right
  } else if(reflectivities[3] > thresh) {
    left = 100;
    right = 65;
  // or drive straight
  } else {
     left = 100;
     right = 100;
  }

  Serial.print(reflectivities[0]);Serial.print(',');Serial.print(reflectivities[1]);Serial.print(',');
  Serial.print(reflectivities[2]);Serial.print(',');Serial.println(reflectivities[3]);

  
  digitalWrite(M1, HIGH); digitalWrite(M2, HIGH);
  analogWrite(E1, factor*left); analogWrite(E2, factor*right);
  // Serial.print(factor*left);   Serial.println(factor*right); 
}
