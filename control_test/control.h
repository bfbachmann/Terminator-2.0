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
	*/
	typedef struct {
		float x;
		float y;
		float heading;	
	} State;

void go(State * state, Vector * destination, bool stopAtDestination = false);
        
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
        
       /*   void wheelControl(float right_velocity, float left_velocity)
        *
        *   Applies PWM signal to wheel motors. Must convert velocities into
        *   PWM appropriate signal (integer from 0-255)
        *   
        *   Parameters:
        *   float left_w:         An angular velocity, specified in radians
        *                         per second, to be applied to the left wheel.
        *                         This function will assume velocities requested
        *                         are within the operating range of the motor as 
        *                         specified in the datasheet. This function must 
        *                         convert the angular velocity to some integer 
        *                         between 0 & 255 using a linear model.
        *
        *   float left_w:         An angular velocity, specified in radians per
        *                         second, to be applied to the right wheel. Similar
        *                         conditions to the left_w parameter.
        */
        void wheelControl(float left_w, float right_w);

       /*   void calculateOdometry(State * state, float left_w, float right_w)
        *
        *   Calculates distance travelled in an individual time step and updates the 
        *   state variables to match. Should be called following each time step so that
        *   the state variables in the x-y plane and the heading remain up to date.
        *   
        *   Parameters: 
        *   State * state:      A pointer to the state structure.
        *   float left_w:       (Read "left omega"). The angular velocity of the left 
        *                       during the time step.
        *   float right_w:      (Read "right omega") The angular velocity of the right
        *                       wheel during the time step.
        */
        void calculateOdometry(State * state, float left_w, float right_w);
       
       /*   float wheelVelocity(float w, float v, int wheel)    
        *
        *   Computes left or right wheel angular velocity given craft desired velocity
        *   and angular velocity. Documentation on these formulae is availabile in the
        *   accompanying report. This abstraction allows us to use a unicycle dynamical
        *   model instead of a differential drive when making incremental state changes.
        *   
        *   Parameters:     
        *   float w:        ("omega") A desired angular velocity of the robot, for one 
        *                   time step.
        *   float v:        A desired velocity of the robot, for one time step.
        *       
        *   Returns:        A wheel velocity in float format for the wheelControl function
        *                   to apply to the wheels, and for the calculateOdometry function
        *                   to use in calculating state changes during a time step.
        */  
        float wheelVelocity(float w, float v, int wheel);