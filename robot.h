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
    class AI {
    public:
        AI(SensorData sensor_data, Control control);
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
        // these constructors are subject to change
        Control(SensorData sensor_data);
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
     */
    class ExternalData {
    public:
        // these constructors are subject to change
        ExternalData();
        ~ExternalData();
        
        /*
         * void clearCache()
         *
         * Instructs the class to clear its data cache. Values returned immediately
         * after this method is called will always query sensors.
         */
        void clearCache();
        
        /*
         * float temperature()
         * 
         * Returns the current ambient air temperature. This function may perform
         * caching.
         *
         * Parameters:
         * bool fresh:  if true, the cache will be ignored and fresh data will be
         *              read from the appropriate sensors.
         */
        float temperature(bool fresh = false);
        
        /*
         * float distance()
         *
         * Returns the current distance to the nearest object as seen by an ultrasonic
         * range finder. This function may perform caching. If fresh data is required,
         * call clearCache() before calling this method.
         *
         * Parameters:
         * int sensor:  the index of the ultrasonic range finder that should be used to
         *              acquire data.
         * bool fresh:  if true, the cache will be ignored and fresh data will be 
         *              read from the appropriate sensors.
         */
        float distance(int sensor, bool fresh = false);
        
        /*
         * float* distances()
         * 
         * Returns the current distance to the nearest object as seen by each ultrasonic
         * range finder. The return array will be indexed as follows:
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
    };
}

#endif /* robot_h */
