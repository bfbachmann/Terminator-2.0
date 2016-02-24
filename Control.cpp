/*
	Robot differential control system
	
	By Chad Lagore
	02/13/16
*/

//
#include "robot.h"
#include <math.h>

// Geometric constants of robot vehicle.
// Wheel radius (R) and wheel to wheel length (L).
#define R 3
#define L 15

// PID controller tuning.
#define k_p 1.5
#define k_i 0
#define k_d 0

// Finite time step in ms.
#define dt 2

// Max speed in cm/s. 
#define MAX_SPEED 20

#pragma mark Begin Robot::Control implementation

/*	go(State state, Vector destination, bool StopAtDestination = false)
    
    Control flow for a single time step in moving the robot towards its
    destination. 
 */
bool Robot::Control::go(State * state, Vector * destination, bool stopAtDestination) {
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
/* 	Computes left or right wheel angular velocity given craft velocity and
 *  angular velocity. Documentation on these formulae is availabile in the
 *	accompanying report.
 *	
 *	Parameters: angular_velocity, velocity and wheel. Wheel determines
 *	which wheel is being calculated. 0 is for left, 1 is for right.
 *	Returns: A wheel velocity.
 */
float Robot::Control::wheelVelocity(float w, float v, int wheel) {
    if(wheel)
        return (2*v + w*L)/(2*R);
    else
        return (2*v - w*L)/(2*R);
}
/*	Applies PWM signal to wheel motors. This function will require access
 *	to pin numbers.
 */
void Robot::Control::wheelControl(float left_w, float right_w) {
    
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
void Robot::Control::calculateOdometry(State * state, float left_w, float right_w) {
    float l_distance = left_w * dt * R / 1000;
    float r_distance = right_w * dt * R / 1000;
    float distance = (l_distance + r_distance) / 2;
    
    state->x = state->x + distance * cos(state->heading);
    state->y = state->y + distance * sin(state->heading);
    state->heading = state->heading + (r_distance-l_distance)/L;
}
