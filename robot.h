//
//  robot.h
//  
//
//  Created by Jordan Jeffries on 2/12/16.
//
//

#ifndef robot_h
#define robot_h

class Robot {
public:
    // these constructors are subject to change
    Robot();
    ~Robot();
    
    /*
     * void run()
     *
     * Runs the robot.
     */
    void run();
};

namespace Robot {
    /*
     * struct Vector
     *
     * Members:
     * float x: the x-component of the vector in centimetres.
     * float y: the y-component of the vector in centimetres.
     */
    typedef struct Vector {
        float x;
        float y;
    };
    
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
         */
        void go(float velocity, float angular_velocity = 0);
        
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
         * bool stopAtDestination:  Optional. Defaults to false.
         *                          Specifies whether the robot should stop once
         *                          it reaches the point specified by direction.
         *                          If false, the robot will continue in the
         *                          specified direction until instructed otherwise.
         */
        void go(Vector direction, bool stopAtDestination = false);
        
        /*
         * void stop()
         *
         * Causes the robot to cease all movement immediately.
         */
        void stop();
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
         * int sensor:  the index of the ultrasonic range finder that should be used to
         *              acquire data.
         * bool fresh:  if true, the cache will be ignored and fresh data will be 
         *              read from the appropriate sensors.
         */
        float distance(int sensor, bool fresh = false);
        
        /*
         * float* distances(bool fresh)
         * 
         * Returns the current distance to the nearest object as seen by each ultrasonic
         * range finder. The units of the return value will be the same as the distance()
         * function. The return array will be indexed as follows:
         *
         *          index   value
         *          0       distance from sensor 0 to nearest object
         *          1       distance from sensor 1 to nearest object
         *          2       distance from sensor 2 to nearest object
         *
         * and so on.
         *
         * This function may perform caching.
         *
         * Parameters:
         * bool fresh:  if true, the cache will be ignored and fresh data will be
         *              read from the appropriate sensors.
         */
        float* distances(bool fresh = false);
        
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
    };
}

#endif /* robot_h */
