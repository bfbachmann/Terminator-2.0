/*
	Robot differential control system
	
	By Chad Lagore
	02/13/16
*/

#include "robot.h"
#include <math.c>

// Geometric constants of robot vehicle.
// Wheel radius (R) and wheel to wheel length (L).
#define R 5
#define L 15

// PID controller tuning.
#define k_p 1
#define k_i 0

// Finite time step in ms.
#define dt 1

// Max speed in cm/s. 
#define MAX_SPEED 20

/*	go(State state, Vector destination, bool StopAtDestination = false)
	
	Control flow for a single time step in moving the robot towards its
	destination. 
 */
void go(State state, Vector destination, bool stopAtDestination = false) {
	float v, w, desired_heading, right_w, left_w;
	Vector goal;
	
	/* Create goal vector */ 
	goal.x = (destination.x - state.x);
	goal.y = (destination.y - state.y);
	
	/* Determine desired heading, velocity and angular velocity */
	velocity = sqrt(goal.x * goal.x + goal.y * goal.y);
	velocity = (velocity > MAX_SPEED) ? MAX_SPEED : velocity;
	desired_heading = atan2(goal.y/goal.x);	
	error = desired_heading - state.heading;
	angular_velocity = k_p * error + k_i * error * dt;
	
	/* Calculate wheel velocities from craft velocity */
	left_w = wheelVelocity(w,v,0);
	right_w = wheelVelocity(w,v,1);
	
	/* Implement wheel velocities */
	wheelControl(left_w, right_w);

	/* Update state variables */
	calculateOdometry(&state, left_w, right_w);
		
}
/* 	Computes left or right wheel angular velocity given craft velocity and
 *  angular velocity. Documentation on these formulae is availabile in the
 *	accompanying report.
 *	
 *	Parameters: angular_velocity, velocity and wheel. Wheel determines
 *	which wheel is being calculated. 0 is for left, 1 is for right.
 *	Returns: A wheel velocity.
 */
float wheelVelocity(float w, float v, int wheel) {
	if(wheel) {
		return (2*v + w*L)/(2*R);
	else {
		return (2*v - w*L)/(r*R);
}
/*	Applies PWM signal to wheel motors. This function will require access
 *	to pin numbers.
 */
void wheelControl(float left_w, float right_w) {
	
}
/*	Calculates distance travelled in time step and updates state variables.
 *	
 *	Parameters: 	Pointers to State struct, left wheel & right wheel angular 
 *					velocities.
 *	
 *	Disclaimer: There are two ways to compute left & right wheel distance.
 *	The method below, D = w*R*dt, or using some sort of wheel encoder info
 *	given by the magnets and hall sensors. IE D = 2*pi*R*(t/N) where
 *	N is the number of ticks in a total wheel revolution, and t is the 
 *   number of ticks in the given time step. We can and should try both.
 */
void calculateOdometry(State * state, float left_w, float right_w) {
	float left_wheel = left_velocity * dt * R;
	float right_wheel = right_velocity * dt * R;
	float distance = (left_wheel_distance + right_wheel_distance)/2;
	
	state->x = state->x + distance*cos(state->heading);
	state->y = state->y + distance*sin(state->heading);
	state->heading = state->heading + (left_wheel-right_wheel)/L;
}
