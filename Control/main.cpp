#include "control.h"
#include "control.cpp"
#include <stdio.h>
#include <limits>
#include <iostream>
#include <fstream>

// using namespace robot;
using namespace std;

#define GOAL_X -400.0
#define GOAL_Y -10.0
#define _USE_MATH_DEFINES

int main() {
	State state;
	Vector destination;
	bool atDestination = false;
	int count = 0;
	std::ofstream outfile;

  	outfile.open("coordinates.txt", std::ios_base::trunc);

	destination.x = GOAL_X;
	destination.y = GOAL_Y;

	state.x = 0;
	state.y = 0;
	state.heading = M_PI / 2;
	state.v = 0;
	state.w = 0;

	printf("--- BEGINNING CONTROL TEST ---\n");
	printf("Current coordinates:\t(0,0)\nGoal coordinates:\t(%f,%f)\n\n", GOAL_X, GOAL_Y);
	
	while(!atDestination) {
		// printf("Time Step:\t%d\nCurrent Coordinates:\t(%f,%f)\n", count, state.x, state.y);
		// printf("Current Heading:\t(%f)\n", state.heading*180/M_PI);
		atDestination = go(&state, &destination, true);
		// cout << "Press Enter to Continue";
		// cin.ignore(std::numeric_limits<streamsize>::max(),'\n');
		outfile << state.x << "," << state.y << "," << state.v << "," << state.w <<endl;
		// outfile << "\n"; 
		count++;
	}

	printf("--- FINISHED CONTROL TEST ---\n");
	printf("Time Step:\t%d\nCurrent Coordinates:\t(%f,%f)\n", count, state.x, state.y);
	printf("Current Heading:\t(%f)\n", state.heading*180/M_PI);

	return 0;
}
