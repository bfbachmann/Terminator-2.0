#include "Robot.h"

#define LINE_MODE 0
#define FREE_DRIVE_MODE 1

/*
 * Constructor for the AI class
 */
AI::AI(ExternalData external_data, Control control_instance) {
	externalData = external_data;
	control = control_instance;
	uint8_t mode = external_data.mode();
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
    control.orientRangeFinder(i*18);
    distances[i] = externalData.distance(2, true);
    delay(10);
  }

  return distances;
}

