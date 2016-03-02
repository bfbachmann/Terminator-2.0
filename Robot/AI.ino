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
 * Destructor for the AI class
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

  if (mode == LINE_MODE) {
     _control->followLine(externalData.reflectivity(true), state);
  }

  else if (mode == FREE_DRIVE_MODE) {
    uint8_t straightAheadDistance = externalData.distance(1, true);
    float newDirectionAngle;
    Vector *shortTermGoal;

    if (straightAheadDistance < 10) {
      control.stop();
      newDirectionAngle = sweep();
      shortTermGoal->x = cos(newDirectionAngle);
      shortTermGoal->y = sin(newDirectionAngle);
    }
    else if (straightAheadDistance < 50) {
      control.slowDown(state);
    }
    else {
      control.go(state, shortTermGoal, true);
    }

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

