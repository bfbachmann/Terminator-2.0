//
//  robot.h
//  
//
//  Created by Jordan Jeffries on 2/12/16.
//
//

#ifndef robot_h
#define robot_h

#include <Servo.h>
#include <limits.h>

// max speed in cm/s
#define MAX_SPEED 61

class AI;
class ExternalData;
class Control;

/*
* struct Vector
*
* Members:
* float x: the x-component of the vector in centimetres.
* float y: the y-component of the vector in centimetres.
*/
typedef struct {
	float x;
	float y;
} Vector;

/* struct State
*
* Structure to hold state variables.
*
* float x:       the x-component of the state coordinate in cm
* float y:       the y-component of the state coordinate in cm
* float heading: the heading of the craft. Measured in radians
*                from the x-axis.
* float v:       the velocity of the robot in cm/s
* float w:       the angular velocity of the robot in rad/sec.
*
*/
typedef struct {
	float x;
	float y;
	float heading;
	float v;
	float w;
  float dt;
  uint8_t  l_PWM;
  uint8_t  r_PWM;
} State;

/*
 * enum to define possible modes.
 */
typedef enum {
	FreeDrive,
	FollowLine
} Mode;

typedef enum {
	Left,
	Right,
  Straight
} Direction;


class AI {
public:
	/*
	* AI(ExternalData external_data, Control control)
	*
	* Initializes the AI and performs necessary setup. This function must be
	* called before any other functions.
	*
	* Parameters:
	* ExternalData external_data:  An ExternalData object from which necessary
	*                              data from the outside world will be read.
	* Control control:             A Control object with which the AI will
	*                              execute decisions that are made.
	*/
	AI(ExternalData *externalData, Control *control);
	~AI();
    
	/*
	* void decide()
	*
	* Decides what action the robot should take, and execute it.
	* This function will gather the necessary data, and then
	* execute the appropriate action.
	*/
	void decide(State *state);

private:

/*
 * Pan the servo motor from 0 to 180 
 * and return the direction corresponding to the best direction 
 * corresponding to the furthest distance read (0 degrees being 
 * left of forward relative to heading of robot, 180 being right).
 */
  Direction sweep();

  /*
   * Pan the servo motor from 0 to 180 in offset increments
   * and return the direction corresponding to the best direction 
   * corresponding to the furthest distance read (0 degrees being 
   * left of forward relative to heading of robot, 180 being right).
   */
  Vector *sweep(uint8_t offset);
	
	/*
	 * void updateMode()
	 * 
	 * Update the mode, and take any action necessary to react to the
	 * mode changing.
	 */
	void updateMode();

#pragma mark Private instance variables
	Control *_control;
	ExternalData *_externalData;
	Mode _currentMode;
  unsigned long timeSinceLastRandomSweep;
};

/*
* class Control
*
* Implements logic to output control instructions to the appropriate components.
*/
class Control {
public:
	/*
	* Control(SensorData sensor_data)
	* 
	* Initializes the object and performs necessary setup. This function
	* must be called before any other functions.
	*
	* Parameters:
	* uint8_t in1:                 The pin to which the in1 pin of the motor
	*                              shield is connected.
	* uint8_t in2:                 The pin to which the in2 pin of the motor
	*                              shield is connected.
	* uint8_t in3:                 The pin to which the in3 pin of the motor
	*                              shield is connected.
	* uint8_t in4:                 The pin to which the in4 pin of the motor
	*                              shield is connected.
    * uint8_t rangeFinder          The pin to which the servo controlling the
    *                              range finder is connected.
	*/
	Control(uint8_t rangeFinder);
	~Control();
    
    /*
     * void initializePins()
     *
     * Sets Arduino pin modes. This method should be called from setup() and before
     * any other functions.
     */
    void initializePins();
    
    /* 
     * void setExternalData(ExternalData *externalData)
     *
     * Set the ExternalData object form which data about the outside world
     * should be collected.
     */
    void setExternalData(ExternalData *externalData);
        
	/*
	* go(Vector direction, bool stopAtDestination)
	*
	* Causes the robot to proceed in the direction specified by direction.
	* 
	* Parameters:
	* vector destination:      A vector specifying the direction in which
	*                          the robot should travel. The origin of the
	*                          vector is assumed to be the center of the
	*                          robot.
	* bool stopAtDestination:  (REMOVED, OPTIONAL). Defaults to false.
	*                          Specifies whether the robot should stop once
	*                          it reaches the point specified by direction.
	*                          If false, the robot will continue in the
	*                          specified direction until instructed otherwise.
	*
	*/
	void go(State *state, Vector *destination, bool stopAtDestination);
    
	/*
	* void stop()
	*
	* Causes the robot to cease all movement immediately.
	*/
	void stop();
    
	/*
	* void orientRangeFinder(int orientation)
	*
	* Causes the ultrasonic range finder on the rotatable platter to
	* assume the specified orientation.
	*
	* Parameters:
	* int orientation: The orientation the range finder should assume,
	*                  specified in degrees. The value should be in the
	*                  range of 0 to 359, inclusive. 0 degrees will be
	*                  assumed be the straight ahead, 90 straight to
	*                  the right, 180 straight back, and 270 straight
	*                  to the left. See the figure below for details.
	*
	*                  0
	*      315 -----------------  45
	*         |      front      |
	*         |                 |
	*         |        ^        |
	*         |       / \       |
	*         |        |        |
	*     270 |      <-+->      | 90
	*         |        |        |
	*         |       \ /       |
	*         |        ˇ        |
	*         |                 |
	*         |      back       |
	*      225 -----------------  135
	*                 180
	*/
	void orientRangeFinder(int orientation);

/*
 * Decrease the speed the robot is moving at by a small amount.
 */
  void slowDown(State *state);

/*
 * Attached the servo motor to its designated input pin.
 */
  void attachRangeFinder();
	
	/*
	 * Send a byte to the slave Arduino to indicate that it should 
	 * take some action.
	 */
	void sendByteToSlave(char byte);

    
private:
	/*	void wheelControl(float right_velocity, float left_velocity)
	*
	*	Applies PWM signal to wheel motors. Must convert velocities into
	*   PWM appropriate signal (integer from 0-255)
	*	
	*	Parameters:
	*	float left_w:		  An angular velocity, specified in radians
	*						  per second, to be applied to the left wheel.
	*						  This function will assume velocities requested
	*						  are within the operating range of the motor as 
	*						  specified in the datasheet. This function must 
	*						  convert the angular velocity to some integer 
	*						  between 0 & 255 using a linear model.
	*
	*	float left_w:		  An angular velocity, specified in radians per
	*						  second, to be applied to the right wheel. Similar
	*						  conditions to the left_w parameter.
	*/
	void wheelControl(State * state, bool left, bool right);

	/*	void calculateOdometry(State * state, float left_w, float right_w)
	*
	*	Calculates distance travelled in an individual time step and updates the 
	*	state variables to match. Should be called following each time step so that
	*	the state variables in the x-y plane and the heading remain up to date.
	*	
	*	Parameters: 
	*	State * state:		A pointer to the state structure.
	*	float left_w:		(Read "left omega"). The angular velocity of the left 
	*						during the time step.
	*	float right_w:		(Read "right omega") The angular velocity of the right
	*						wheel during the time step.
	*/
	void calculateOdometry(State * state, float left_w, float right_w);
   
	/*	float wheelVelocity(float w, float v, int wheel)	
	*
	*	Computes left or right wheel angular velocity given craft desired velocity
	*  	and angular velocity. Documentation on these formulae is availabile in the
	*	accompanying report. This abstraction allows us to use a unicycle dynamical
	*	model instead of a differential drive when making incremental state changes.
	*	
	*	Parameters:		
	*	float w:		("omega") A desired angular velocity of the robot, for one 
	*					time step.
	*	float v:		A desired velocity of the robot, for one time step.
	*		
	*	Returns:		A wheel velocity in float format for the wheelControl function
	*					to apply to the wheels, and for the calculateOdometry function
	*					to use in calculating state changes during a time step.
	*/	
	float wheelVelocity(float w, float v, int wheel);

        /*	void adjustHeading(State * state, Vector * destination);	
	*
	*	Caculates an error in its heading, and stabilizes it to zero by adjusting wheel speeds.
	*	
	*	Parameters:		
	*	State * state:          The state structure. Pre-loaded is a current heading. 
	*				
	*	Vector * destination:	A vector structure with a desired destination.
	*		
	*/	
        void adjustHeading(State * state, Vector * destination);
 
public:
 /*	void followLine(float *reflectivities)	
	*
	*	Completes one time step in the control flow of a line-following behaviour.
	*	
	*	Parameters:		
	*	State *state:	 state variable representing the action to be taken by
	* 							 the robot.
	*/	
  void followLine(State *state);
    
private:
#pragma mark Pin instance variables
    uint8_t rangeFinderPin;
    
#pragma mark Servo instance variables
    Servo rangeFinderServo;
		int _currentRangeFinderOrientation;
};

/*
* class ExternalData
*
* Implements logic to acquire data from input devices. This class may use caching
* in order to reduce actual accesses to hardward devices. Every data acquisition
* method will specify its caching behaviour.
*/
class ExternalData {
public:
	// these constructors are subject to change
	/*
	* ExternalData(int temperaturePin, int numberOfUltrasonicSensors, int* ultrasonicSensors)
	*
	* Initializes ExternalData and performs necessary setup. This function
	* must be called before any other functions.
	*
	* Parameters:
	* int numberOfUltrasonicSensors:   the number of ultrasonic sensors that are
	*                                  available to the robot.
	* uint8_t** ultrasonicSensors:     pins to which each ultrasonic sensor is connected.
	*                                  The array should be formatted as follows, indexed
	*                                  as ultrasonicSensors[index]:
	* 
	*          index   value
	*          0       pin to which the trigger pin of the 0th ultrasonic
	*                  sensor is connected.
	*          1       pin to which the echo pin of the 0th ultrasonic sensor
	*                  is connected.
	*          2       pin to which the trigger pin of the 1st ultrasonic
	*                  sensor is connected.
	*          3       pin to which the echo pin of the 1st ultrasonic sensor
	*                  is connected.
	*
	*                                   And so on.
    * int numberOfReflectivitySensors:  The number of reflectivity sensors that are
    *                                   available to the robot.
    * uint8_t* reflectivitySensors:     Pins to which the reflectivity sensors are
    *                                   connected.
	*/
	ExternalData(int numberOfUltrasonicSensors, uint8_t ultrasonicSensors[], int numberOfReflectivitySensors, uint8_t reflectivitySensors[], int mode_pin);
	~ExternalData();
  
	/*
	 * void initializePins()
	 *
	 * Sets Arduino pin modes. This method should be called from setup() and before
     * any other functions.
	 */
	void initializePins();
    
	/*
	* void clearCache()
	*
	* Instructs the class to clear its data cache. Values returned immediately
	* after this method is called will always query sensors.
	*/
	void clearCache();
    
	/*
	* float temperature(bool fresh)
	* 
	* Returns the current ambient air temperature in degrees Celsuis. This function
	* may perform caching.
	*
	* Parameters:
	* bool fresh:  This parameter is optional.
	*              If true, the cache will be ignored and fresh data will be
	*              read from the appropriate sensors.
	*/
	float temperature(bool fresh = false);
    
	/*
	* float distance(bool fresh)
	*
	* Returns the current distance to the nearest object as seen by an ultrasonic
	* range finder in cm. This function may perform caching.
	*
	* Parameters:
    * int sensor:        ultrasonic sensor from which to read distance
	* bool fresh:  		 if true, the cache will be ignored and fresh data will be
	*              		 read from the appropriate sensors.
	*/
	float distance(int sensor, bool fresh = false, int maxChange = INT_MAX);
    
	/*
	* float* distances(bool fresh)
	* 
	* Returns the current distance to the nearest object in line of sight as seen by each ultrasonic
	* range finder. The units of the return value will be the same as the distance()
	* function. The return array will be indexed as follows:
	*
	*          index   value
	*          0       distance from sensor 0 to nearest object
	*          1       distance from sensor 1 to nearest object
	*          2       distance from sensor 2 to nearest object
	*
	* and so on. The value returned by this function will be dynamically allocated,
	* and must be freed by the caller.
	*
	* This function may perform caching.
	*
	* Parameters:
	* bool fresh:  	if true, the cache will be ignored and fresh data will be
	*              	read from the appropriate sensors.
	*/
	float* distances(bool fresh = false);
    
	/*
	* float reflectivity(int sensor, bool fresh)
	*
	* Returns the reflectivities seen by the reflective optical sensors. This function
	* may perform caching.
	* 
	* Parameters:
	* int sensor:  the sensor to read reflectivity from 
	* bool fresh:  if true, the cache will be ignored and fresh data will be
	*              read from the appropriate sensors. Defaults to false.
	*/
	float reflectivity(int sensor, bool fresh = false);
	
	/*
	* float *reflectivities(bool fresh)
	*
	* Returns the reflectivities seen by the reflective optical sensors. This function
	* may perform caching.
	* 
	* Parameters:
	* bool fresh:  if true, the cache will be ignored and fresh data will be
	*              read from the appropriate sensors.
	*/
	float *reflectivities(bool fresh = false);

 /*
  * Get the mode the robot is currently in by reading from a push-button
  * Returns 0 if the button is pressed corresponding to LINE_MODE for 
  * following a line on the ground, or returns 1 if the button is not 
  * pressed curresponding to FREE_DRIVE_MODE for driving around freely.
  */
  Mode mode();

  float read_temperature();
  float readDistance(int sensor, float temperature);

 
private:
#pragma mark Pin variables
	int _temperaturePin;
  int _modePin;
	int _numberOfUltrasonicSensors;
	uint8_t* _ultrasonicSensorPins;
  int _numberOfReflectivitySensors;
  uint8_t* _reflectivitySensorPins;
    
#pragma mark Caching variables
	float _lastTemperature;
	float *_lastDistances;
	float *_lastReflectivities;
    
	unsigned long _lastTemperatureTimestamp;
	bool *_distancesCached;
	bool *_reflectivitiesCached;
    
#pragma mark Data acquisition functions
	float _readTemperature();
	void _pulseOut(uint8_t pin, int microseconds);
	float _readDistance(int sensor, float temperature);
};

#endif /* robot_h */
