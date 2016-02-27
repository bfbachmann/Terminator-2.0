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
#define k_p 1.5
#define k_i 1
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
	float v, w, desired_heading, right_w, left_w, error, velocity;
	Vector goal;
    
	/* Create goal vector */ 
	goal.x = (destination->x - state->x);
	goal.y = (destination->y - state->y);
    
	/* Determine desired heading, velocity and angular velocity */
	state->v = sqrt(goal.x * goal.x + goal.y * goal.y);
	state->v = (state->v > MAX_SPEED) ? MAX_SPEED : state->v;
	desired_heading = atan2(goal.y,goal.x); 
	error = desired_heading - state->heading;
	error = atan2(sin(error), cos(error));
	state->w = k_p * error + k_i * error * dt;
    
	/* Calculate wheel velocities from craft velocity */
	left_w = wheelVelocity(state->w,state->v,0);
	right_w = wheelVelocity(state->w,state->v,1);
    
	/* Implement wheel velocities */
	wheelControl(left_w, right_w);

	/* Update state variables */
	calculateOdometry(state, left_w, right_w);

	/* Return true if within threshold of destination */
	return (state->v < 5);
        
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
void Control::wheelControl(float left_w, float right_w) {
    int E1 = 5;
    int E2 = 6;
    int M1 = 4;
    int M2 = 7;
    int left_norm, right_norm = 0;
    
    left_norm = (left_w >= 6.66) ? 255 : left_w*255/6.66;
    right_norm = (right_w >= 6.66) ? 255 : right_w*255/6.66;
    
    digitalWrite(M1, HIGH); digitalWrite(M2, HIGH);
    analogWrite(E1, left_norm); analogWrite(E2, right_norm);
  }

/*  Calculates distance travelled in time step and updates state variables.
*  
*  Parameters:   Pointers to State struct, left wheel & right wheel angular 
*          velocities.
*  
*  Disclaimer: There are two ways to compute left & right wheel distance.
*  The method below, D = w*R*dt, or using some sort of wheel encoder info
*  given by the magnets and hall sensors. IE D = 2*pi*R*(t/N) where
*  N is the number of ticks in a total wheel revolution, and t is the 
*   number of ticks in the given time step. We can and should try both.
*/
void Control::calculateOdometry(State * state, float left_w, float right_w) {
	float l_distance = left_w * dt * R / 1000;
	float r_distance = right_w * dt * R / 1000;
	float distance = (l_distance + r_distance) / 2;
    
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
void Control::followLine(float s0, float s1, float s2) {
  const int thresh = 100;
  float factor = 1.2;
  int E1 = 5;
  int E2 = 6;
  int M1 = 4;
  int M2 = 7;
  int left, right = 0;
  
  if(s1 > thresh) {
    left = 65;
    right = 100;//slightRight
  }
  else if(s2 > thresh) {
    left = 100;
    right = 65; // slightLeft
  }
  else {
     left = 100;
     right = 100;
  }
     
  digitalWrite(M1, HIGH); digitalWrite(M2, HIGH);
  analogWrite(E1, factor*left); analogWrite(E2, factor*right);
  // Serial.println(factor*left);
}
