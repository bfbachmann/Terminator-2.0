#include "Robot.h"

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
     _control->followLine(_externalData->reflectivities(), state);
  }

//if we are in free drive mode we need to look for nearby obstancles
//and slow down or stop depending on how close they are
  else if (_currentMode == FreeDrive) {
		_control->orientRangeFinder(90);
    uint8_t straightAheadDistance = _externalData->distance(0);
    Vector shortTermGoal;

    if (straightAheadDistance < 5 || state->v <= 0) {
      _control->stop();
      float newDirectionAngle = sweep();
      shortTermGoal.y = 0.0;
			if (cos(newDirectionAngle) > 0) {
				shortTermGoal.x = 1.0;
			} else {
				shortTermGoal.x = -1.0;
			}
    }
    else if (straightAheadDistance < 50) {
			float aggressiveness = (50.0 - straightAheadDistance) / 45.0;
      control.slowDown(state, aggressiveness);
			return;
    }
    else {
			// TODO: move forward at max. possible velocity
    }
		
		control.go(state, &shortTermGoal, true);
  }
}

/*
 * Pan the servo motor from 0 to 180 degrees and return the angle (in radians)
 * corresponding to the furthest distance read (0 degrees being 
 * left of forward relative to heading of robot, 180 being right).
 */
float AI::sweep() {
  _control->orientRangeFinder(0);
	float leftDistance = _externalData->distance(0, true);
	
	_control->orientRangeFinder(180);
	float rightDistance = _externalData->distance(0, true);
	
	if (leftDistance > rightDistance) {
		return PI;
	} else {
		return 0;
	}
}

void AI::updateMode() {
  Mode mode = _externalData->mode();
	
	if (mode != _currentMode) {
		// we have changed modes
	}
	
	_currentMode = mode;
}
