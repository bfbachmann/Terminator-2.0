#include "Robot.h"

#define LINE_MODE 0
#define FREE_DRIVE_MODE 1

/*
 * Constructor for the AI class
 */
AI::AI(ExternalData *externalData, Control *control) {
	_externalData = externalData;
	_control = control;
	mode = _externalData->mode();
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
  mode = _externalData->mode();

//if we are in line we we should just call control to follow line
  if (mode == LINE_MODE) {
     _control->followLine(_externalData->reflectivity(true), state);
  }

//if we are in free drive mode we need to look for nearby obstancles
//and slow dont or stop depending on how close they are
  else if (mode == FREE_DRIVE_MODE) {
    uint8_t straightAheadDistance = _externalData->distance(1, true);
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
			float aggressiveness = (50.0 - straightAheadDistance) / 50.0;
      control.slowDown(state, aggressiveness);
			return;
    }

    else {

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
  
    uint8_t bestDistanceAngle = 90;
    float bestDistance = 0;
    float currentDistance;
    int i;
  
    for (i = 0; i <= 10; i++) { 
      _control->orientRangeFinder(i*18);
      currentDistance = _externalData->distance(0, true);
      if (currentDistance > bestDistance) {
        bestDistance = currentDistance;
        bestDistanceAngle = i*18;
      }
      delay(50);
    }

  return (180-bestDistanceAngle)*(PI/180);
}

