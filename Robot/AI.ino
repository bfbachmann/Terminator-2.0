#include "Robot.h"

#define RANDOM_SWEEP_DELAY_CYCLES 200
#define RANDOM_SWEEP 1

#define FREE_DRIVE_SLOW_DISTANCE 55
#define FREE_DRIVE_HALT_DISTANCE 20
#define THRESH 300 

/*
 * Constructor for the AI class
 */
AI::AI(ExternalData *externalData, Control *control) {
	_externalData = externalData;
	_control = control;
 timeSinceLastRandomSweep = 0;
 _currentMode = Uninitialized;
}


/*
 * Deconstructor for the AI class
 */
AI::~AI() {
	
}

/*
 * Make a descision based on the mode the AI is currently in
 * and its orientation and surroundings.
 * Gets necessary data from externalData and uses Control
 * to take action.
 */
void AI::decide(State *state) {
	_externalData->clearCache();
	
	updateMode();

//if we are in line we we should just call control to follow line
  if (_currentMode == FollowLine) {
		bool lostLine = false;
		
    float a = _externalData->reflectivity(0);
    float b = _externalData->reflectivity(1); 
    float c = _externalData->reflectivity(2); 
    float d = _externalData->reflectivity(3); 

		if ( a > THRESH) { // turn left
			state->l_PWM = 55;	state->r_PWM = 100;
		} else if ( d > THRESH) { // or turn right
			state->l_PWM = 100; state->r_PWM = 55;
		} else if ( a < THRESH && b < THRESH &&	c < 800 && d < THRESH) {
			// or use state to remember in which direction the lost line is
			lostLine = true;
			if (state->r_PWM > state->l_PWM) {
				state->l_PWM = 30; state->r_PWM = 100;
			} else {
				state->r_PWM = 30; state->l_PWM = 100;
			}
		} else if ( b > THRESH && c > 800) { // or drive straight
			state->r_PWM = 100; state->l_PWM = 100;
		} else if ( c > 800) { // or slight right
			state->l_PWM = 100; state->r_PWM = 85;
		} else if ( b > THRESH) { // or slight left
			state->l_PWM = 85; state->r_PWM = 100;
		}
		
		if (lostLine) {
			control.sendByteToSlave('j');
		} else {
			control.sendByteToSlave('n');
		}

    // send state with updated wheel speeds for next time step to control
   	_control->followLine(state);
  }

//if we are in free drive mode we need to look for nearby obstancles
//and slow down or stop depending on how close they are
  else if (_currentMode == FreeDrive) {
        _control->orientRangeFinder(90);
        float straightAheadDistance = _externalData->distance(0, false, (0.5 * state->v));
        Vector shortTermGoal;
        timeSinceLastRandomSweep++;
        
        if (straightAheadDistance < FREE_DRIVE_HALT_DISTANCE) {
    			_control->stop();
                        control.sendByteToSlave('h');
                        shortTermGoal.y = 0.0;
    			if (sweep() == Right) {
    				control.sendByteToSlave('g');
    				shortTermGoal.x = 1.0;

    			} else {
    				control.sendByteToSlave('f');
    				shortTermGoal.x = -1.0;
    			}
                        control.adjustHeading(state, &shortTermGoal, true);
                        shortTermGoal = {0.0, 1.0};
        }
        else if (straightAheadDistance < FREE_DRIVE_SLOW_DISTANCE) {
    			control.sendByteToSlave('d');
          control.slowDown(state);
    			return;
        }
        
        else {
                //check if we should do a random sweep
              // if (timeSinceLastRandomSweep > RANDOM_SWEEP_DELAY_CYCLES) {
              //           timeSinceLastRandomSweep = 0;
              //           control.sendByteToSlave('c');
              //           Direction avoidanceDirection = sweep();
              //           if(avoidanceDirection == Left) { shortTermGoal = {-1,1.5}; }
              //           else if(avoidanceDirection == Right) { shortTermGoal = {1,1.5}; }
              //           else { shortTermGoal = {0,1}; }
              //   } else { // we are just driving forward
            	        control.sendByteToSlave('b');
                        shortTermGoal = {0.0,1.0}; 
                // }
        }	
        state->v = MAX_SPEED;
        control.go(state, &shortTermGoal, false);
  }
}

/*
 * Pan the servo motor from 0 to 180 degrees and return the angle (in radians)
 * corresponding to the furthest distance read (0 degrees being 
 * left of forward relative to heading of robot, 180 being right).
 */
Direction AI::sweep() {
  _control->orientRangeFinder(0);
	float rightDistance = _externalData->distance(0, true);
        delay(100);
	
	_control->orientRangeFinder(180);
	float leftDistance = _externalData->distance(0, true);

        if (abs(leftDistance - rightDistance) < 10) {
                return Straight;
        }
	else if (leftDistance > rightDistance) {
		return Left;
	} else {
		return Right;
	} 
}

Vector *AI::sweep(uint8_t offset) {
  int i;
  Vector *avoidanceVector = (Vector *) malloc(sizeof(Vector));
  avoidanceVector->x = 0;
  avoidanceVector->y = 0;
  float reading;
  
  for (i = 0; i <= 180; i += offset) {
     _control->orientRangeFinder(i);
     delay(200);
     reading = _externalData->distance(0, true);
     avoidanceVector->x += reading*cos(i*PI/180);
     avoidanceVector->y += reading*sin(i*PI/180);
  }

  for (i = 180 + offset; i < 360; i += offset) {
     avoidanceVector->x += DIST_MAX*cos(i*PI/180);
     avoidanceVector->y += DIST_MAX*sin(i*PI/180);
  }

  return avoidanceVector;
}

void AI::updateMode() {
  Mode mode = _externalData->mode();
	
	if (mode != _currentMode) { // if the mode has changed, let the slaves know
		if (mode == FreeDrive) {
			control.sendByteToSlave('a');
		} else if (mode == FollowLine) {
			control.sendByteToSlave('i');
		}
	}
	
	_currentMode = mode;
}
