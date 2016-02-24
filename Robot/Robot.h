//
//  robot.h
//  
//
//  Created by Jordan Jeffries on 2/12/16.
//
//

#ifndef robot_h
#define robot_h

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
} State;


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
    AI(ExternalData external_data, Control control);
    ~AI();
    
    /*
     * void decide()
     *
     * Decides what action the robot should take, and execute it.
     * This function will gather the necessary data, and then
     * execute the appropriate action.
     */
    void decide();
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
     * ExternalData external_data:  The SensorData object from which necessary
     *                              data from the outside world will be read.
     * int in1:                     The pin to which the in1 pin of the motor
     *                              shield is connected.
     * int in2:                     The pin to which the in2 pin of the motor
     *                              shield is connected.
     * int in3:                     The pin to which the in3 pin of the motor
     *                              shield is connected.
     * int in4:                     The pin to which the in4 pin of the motor
     *                              shield is connected.
     */
    Control(ExternalData external_data, int in1, int in2, int in3, int in4);
    ~Control();
    
    /*
     * void go(float velocity, float angular_velocity)
     *
     * Causes the robot to proceed at the specified velocity and
     * angular velocity.
     *
     * Parameters:
     * float velocity:  the velocity at which the robot should proceed
     *                  in m/s.
     * float angular_velocity:  the angular velocity at which the robot
     *                          should proceed in rad/sec. Defaults to
     *                          0, indicating no turning.
     * Returns:
     * boolean:                 If the robot is within a sufficiently small
     *                          threshold of the requested destination, the go
     *                          function will return a boolean true.
     */
    bool go(float velocity, float angular_velocity = 0);
    
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
     * Returns:
     * boolean:                 If the robot is within a sufficiently small
     *                          threshold of the requested destination, the go
     *                          function will return a boolean true.
     */
    bool go(State *state, Vector *destination, bool stopAtDestination);
    
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
    void wheelControl(float left_w, float right_w);

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
};

/*
 * class ExternalData
 *
 * Implements logic to acquire data from input devices. This class may use caching
 * in order to reduce actual accesses to hardward devices. Every data acquisition
 * method will specify its caching behaviour.
 *
 *
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
     * int temperaturePin:              the pin to which the temperature sensor's Vout pin
     *                                  is connected.
     * int numberOfUltrasonicSensors:   the number of ultrasonic sensors that are
     *                                  available to the robot.
     * int** ultrasonicSensors:         pins to which each ultrasonic sensor is connected.
     *                                  The array should be formatted as follows, indexed
     *                                  as ultrasonicSensors[index][index2]:
     * 
     *          index   index2  value
     *          0       0       pin to which the trigger pin of the 0th ultrasonic
     *                          sensor is connected.
     *          0       1       pin to which the echo pin of the 0th ultrasonic sensor
     *                          is connected.
     *          1       0       pin to which the trigger pin of the 1st ultrasonic
     *                          sensor is connected.
     *          1       1       pin to which the echo pin of the 0th ultrasonic sensor
     *                          is connected.
     *
     * And so on.
     */
    ExternalData(int temperaturePin, int numberOfUltrasonicSensors, int** ultrasonicSensors);
    ~ExternalData();
    
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
     * uint8_t pulseOutPin:  pin to send the read pulse to
     * uint8_t pulseInPin:  pin to recieve the output from the sensor on
     * bool fresh:  		 if true, the cache will be ignored and fresh data will be 
     *              		 read from the appropriate sensors.
     */
    float read_distance(uint8_t pulseOutPin, uint8_t pulseInPin, bool fresh = false);
    
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
     * float* cache:	the location to in memory to store the sensor data
     */
    float* read_distances(float* cache, bool fresh = false);
    
    /*
     * void reflectivity(bool fresh)
     *
     * Returns the reflectivity seen by the reflective optical sensor. The units
     * of the return value is not yet specified. This function may perform caching.
     *
     * Parameters:
     * bool fresh:  if true, the cache will be ignored and fresh data will be
     *              read from the appropriate sensors.
     */
    float reflectivity(bool fresh = false);
    
    /*
     * 
     * Reads distances from -90 to 90 degrees from "forward" relative to the direction the 
     * robot is facing and stores each distance in the cache.
     * 
     * Parameters:
     * float* cache: 	the location to in memory to store the sensor data
     */
     float* survey(float *cache);

    
private:
#pragma mark Pin variables
    int temperaturePin;
    int numberOfUltrasonicSensors;
    int **ultrasonicSensorPins;
    
#pragma mark Caching variables
    float lastTemperature;
    float *lastDistances;
    float lastReflectivity;
    
    bool temperatureCached;
    bool *distancesCached;
    bool reflectivityCached;
    
#pragma mark Data acquisition functions
    float readTemperature();
    void pulseOut(uint8_t pin, int microseconds);
    float readDistance(uint8_t triggerPin, uint8_t echoPin, float temperature);
    
};

#endif /* robot_h */
