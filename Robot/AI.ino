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
void AI::decide() {
	
}

/*
 * Pan the servo motor from 0 to 180 degrees and return an array 
 * of the distances read from the distance sensor attatched to
 * the servo. 
 * NOTE: the caller of this function is reponisble for free()ing 
 * array of distance values one they are no longer in use.
 */
float *AI::sweep() {
  
  float *distances = (float *) malloc(sizeof(float)*18);
  int i;
  
  for (i = 0; i <= 10; i++) { 
    _control->orientRangeFinder(i*18);
    distances[i] = _externalData->distance(2, true);
    delay(10);
  }

  return distances;
}

