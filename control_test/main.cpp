#include "control.h"
#include "control.cpp"
#include <stdio.h>
#include <limits>
#include <iostream>

// using namespace robot;
using namespace std;

#define GOAL_X 100.0
#define GOAL_Y 500.0
#define _USE_MATH_DEFINES

int main() {
	State state;
	Vector destination;
	int count = 0;

	destination.x = GOAL_X;
	destination.y = GOAL_Y;

	state.x = 0;
	state.y = 0;
	state.heading = M_PI/2;

	printf("--- BEGINNING CONTROL TEST ---\n");
	printf("Current coordinates:\t(0,0)\nGoal coordinates:\t(%f,%f)\n\n", GOAL_X, GOAL_Y);
	
	while(state.x < GOAL_X && state.y < GOAL_Y) {
		// printf("Time Step:\t%d\nCurrent Coordinates:\t(%f,%f)\n", count, state.x, state.y);
		// printf("Current Heading:\t(%f)\n", state.heading*180/M_PI);
		go(&state, &destination, true);
		// cout << "Press Enter to Continue";
		// cin.ignore(std::numeric_limits<streamsize>::max(),'\n');
		count++;
	}

	printf("Time Step:\t%d\nCurrent Coordinates:\t(%f,%f)\n", count, state.x, state.y);
	printf("Current Heading:\t(%f)\n", state.heading*180/M_PI);

	return 0;
}
