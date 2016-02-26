/*
  Robot differential control system
  
  By Chad Lagore
  02/13/16
*/

#include <math.h>

typedef struct {
    float x;
    float y;
} Vector;

typedef struct {
    float x;
    float y;
    float heading;
    float v;
    float w; 
    float dt;   
} State;

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

// Goal coordinates.
#define GOAL_X -500.0
#define GOAL_Y -500.0

#define _USE_MATH_DEFINES

State state;
Vector destination;
bool atDestination = false;
long last_time;

bool go(State * state, Vector * destination, bool stopAtDestination);
float wheelVelocity(float w, float v, int wheel);
void wheelControl(float left_w, float right_w);
void calculateOdometry(State * state, float left_w, float right_w);

void setup() {
  Serial.begin(9600);
  destination.x = GOAL_X;
  destination.y = GOAL_Y;
  state.x = 0;
  state.y = 0;
  state.heading = M_PI / 2;
  state.v = 0;
  state.w = 0;
  state.dt = 1;
  delay(5000);
}

void loop() {
  int last_time;
  while(!atDestination) {
    last_time = millis();
    atDestination = go(&state, &destination, true);
    state.dt = millis() - last_time;
  }	
}

bool go(State * state, Vector * destination, bool stopAtDestination) {
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
  state->w = k_p * error + k_i * error * state->dt;
  
  /* Calculate wheel velocities from craft velocity */
  left_w = wheelVelocity(state->w,state->v,0);
  right_w = wheelVelocity(state->w,state->v,1);
  //Serial.print(state->x); Serial.print(",");Serial.println(state->y);
  
  /* Implement wheel velocities */
  wheelControl(left_w, right_w);
  Serial.print(state->x); Serial.print(",");Serial.println(state->y);

  /* Update state variables */
  calculateOdometry(state, left_w, right_w);

  /* Return true if within threshold of destination */
  return (state->v < 5);
      
}

float wheelVelocity(float w, float v, int wheel) {
  if(wheel)
      return (2*v + w*L)/(2*R);
  else
      return (2*v - w*L)/(2*R);
}

void wheelControl(float left_w, float right_w) {
  int E1 = 5;
  int E2 = 6;
  int M1 = 4;
  int M2 = 7;
  int left_norm, right_norm = 0;
  
  left_norm = (left_w >= 6.66) ? 255 : left_w*255/6.66;
  right_norm = (right_w >= 6.66) ? 255 : right_w*255/6.66;
  
  digitalWrite(M1, HIGH); digitalWrite(M2, HIGH);
  analogWrite(E1, left_norm); analogWrite(E2, right_norm);
  //Serial.print(left_norm); Serial.print(",");Serial.println(right_norm);
}

void calculateOdometry(State * state, float left_w, float right_w) {
  float l_distance = left_w * state->dt * R / 1000;
  float r_distance = right_w * state->dt * R / 1000;
  float distance = (l_distance + r_distance) / 2;
  
  state->x = state->x + distance * cos(state->heading);
  state->y = state->y + distance * sin(state->heading);
  state->heading = state->heading + (r_distance-l_distance)/L;
}

