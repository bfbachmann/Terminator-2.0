#include "Robot.h"

#define RANDOM_SWEEP_DELAY_CYCLES 15
#define RANDOM_SWEEP 1

#define FREE_DRIVE_SLOW_DISTANCE 75
#define FREE_DRIVE_HALT_DISTANCE 15
#define THRESH 300 

/*
 * Constructor for the AI class
 */
AI::AI(ExternalData *externalData, Control *control) {
	_externalData = externalData;
	_control = control;
 timeSinceLastRandomSweep = 0;
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
    float a = _externalData->reflectivity(0);
    float b = _externalData->reflectivity(1); 
    float c = _externalData->reflectivity(2); 
    float d = _externalData->reflectivity(3); 

    // turn left
		if ( a > THRESH) { state->l_PWM = 55;	state->r_PWM = 100;
    // or turn right
		} else if ( d > THRESH) { state->l_PWM = 100; state->r_PWM = 55;
    // or use state to determine direction of the line
		} else if ( a < THRESH && b < THRESH &&	c < 800 && d < THRESH) {
			// or use state to remember in which direction the lost line is
			if (state->r_PWM > state->l_PWM) { state->l_PWM = 30; state->r_PWM = 100;
			} else { state->r_PWM = 30; state->l_PWM = 100;	}
    // or drive straight
		} else if ( b > THRESH && c > 800) { state->r_PWM = 100; state->l_PWM = 100;
    // or slight right
		} else if ( c > 800) { state->l_PWM = 100; state->r_PWM = 85;
    // or slight left
		} else if ( b > THRESH) { state->l_PWM = 85; state->r_PWM = 100; }

    // send state with updated wheel speeds for next time step to control
   	_control->followLine(state);
  }

//if we are in free drive mode we need to look for nearby obstancles
//and slow down or stop depending on how close they are
  else if (_currentMode == FreeDrive) {
    
		_control->orientRangeFinder(90);
    float straightAheadDistance = _externalData->distance(0, false, (0.5 * state->v));
#ifdef DEBUG
		Serial.print("Straight ahead distance: ");
		Serial.println(straightAheadDistance);
#endif
    Vector shortTermGoal;
    timeSinceLastRandomSweep++;
#ifdef DEBUG
    Serial.print("Time since last sweep: ");
    Serial.println(timeSinceLastRandomSweep);
#endif
    
    if (straightAheadDistance < FREE_DRIVE_HALT_DISTANCE) {
      _control->stop();
      shortTermGoal.y = 0.0;
			if (sweep() == Right) {
#ifdef DEBUG
				Serial.println("Turning right to avoid object");
#endif
				shortTermGoal.x = 1.0;
			} else {
#ifdef DEBUG
				Serial.println("Turning left to avoid object");
#endif
				shortTermGoal.x = -1.0;
			}
    }
    else if (straightAheadDistance < FREE_DRIVE_SLOW_DISTANCE) {
#ifdef DEBUG
			Serial.println("Slowing");
#endif
      control.slowDown(state);
			return;
    }
    else {
#ifdef DEBUG
			Serial.println("Careening");
#endif
      //check if we should do a random sweep

#ifdef RANDOM_SWEEP
      if (timeSinceLastRandomSweep > RANDOM_SWEEP_DELAY_CYCLES) {
        timeSinceLastRandomSweep = 0;
#ifdef DEBUG
        Serial.println("Doing random sweep");
#endif 
        Vector *avoidanceVector = sweep(36);

        shortTermGoal.x = avoidanceVector->x;
        shortTermGoal.y = avoidanceVector->y;
        free(avoidanceVector);

#ifdef DEBUG
       Serial.print("Avoidance vector x: ");
       Serial.println(avoidanceVector->x);
       Serial.print("Avoidance vector y: ");
       Serial.println(avoidanceVector->y);
#endif

      
      }
#else
      shortTermGoal.x = 0;
		  shortTermGoal.y = 10.0;
#endif
      state->v = MAX_SPEED;
    }
		
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
	
	_control->orientRangeFinder(180);
	float leftDistance = _externalData->distance(0, true);

  if (leftDistance - rightDistance < 10) {
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
     reading = _externalData->distance(0, true);
     avoidanceVector->x += reading*cos(i*PI/180);
     avoidanceVector->y += reading*sin(i*PI/180);
  }

  for (i = 180 + offset; i < 360; i += offset) {
     avoidanceVector->x += reading*cos(i*PI/180);
     avoidanceVector->y += reading*sin(i*PI/180);
  }

  return avoidanceVector;
}

void AI::updateMode() {
  Mode mode = _externalData->mode();
	
	_currentMode = mode;
}
