#include "Robot.h"
#define MAX_SPEED 61

/*
 * Constructor for the AI class
 */
AI::AI(ExternalData *externalData, Control *control) {
	_externalData = externalData;
	_control = control;
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
		float* reflectivities = _externalData->reflectivities();
     _control->followLine(reflectivities, state);
		 free(reflectivities);
  }

//if we are in free drive mode we need to look for nearby obstancles
//and slow down or stop depending on how close they are
  else if (_currentMode == FreeDrive) {
		_control->orientRangeFinder(90);
    float straightAheadDistance = _externalData->distance(0, false, (0.5 * state->v));
		Serial.print("Straight ahead distance: ");
		Serial.println(straightAheadDistance);
    Vector shortTermGoal;

    if (straightAheadDistance < 10) {
      _control->stop();
      shortTermGoal.y = 0.0;
			if (sweep() == Right) {
				Serial.println("Turning right to avoid object");
				shortTermGoal.x = 1.0;
			} else {
				Serial.println("Turning left to avoid object");
				shortTermGoal.x = -1.0;
			}
    }
    else if (straightAheadDistance < 60) {
			Serial.println("Slowing");
      control.slowDown(state);
			return;
    }
    else {
			Serial.println("Careening");
			shortTermGoal.x = 0;
			shortTermGoal.y = 10.0;
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
	float leftDistance = _externalData->distance(0, true);
	
	_control->orientRangeFinder(180);
	float rightDistance = _externalData->distance(0, true);
	
	if (leftDistance > rightDistance) {
		return Left;
	} else {
		return Right;
	}
}

void AI::updateMode() {
  Mode mode = _externalData->mode();
	
	if (mode != _currentMode) {
		// we have changed modes
	}
	
	_currentMode = mode;
}
