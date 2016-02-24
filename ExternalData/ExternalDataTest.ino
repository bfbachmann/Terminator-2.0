
/*Input pins to be read from*/
#define TEMPERATURE_PIN 5
#define DIST_SENSOR_ECHO_PIN1 12
#define DIST_SENSOR_ECHO_PIN2 7
#define DIST_SENSOR_ECHO_PIN3 6

/*Output pins for sensors*/
#define DIST_SENSOR_TRIGGER_PIN1 11
#define DIST_SENSOR_TRIGGER_PIN2 8
#define DIST_SENSOR_TRIGGER_PIN3 9

/*the pin for writing to the servo motor*/
#define SERVO_PIN

#define DIST_MAX 200
#define DIST_MIN 0

class ExternalData {

   Servo servo;
   //angle (in degrees) servo is at, 90 degrees left of straight ahead relative to the robot's heading
   int servo_angle;
    
public:  

    /*
     * The constructor for the class
     */
    ExternalData() {
      servo.attach(SERVO_PIN);
      servo_angle = servo.read();
    }
    
    float get_distance_at_angle(int angle) {
      servo.write(angle);
      servo_angle = angle;
      return read_distance(DIST_SENSOR_TRIGGER_PIN2, DIST_SENSOR_ECHO_PIN2, read_tempterature());
    }
    
    float *get_distances(float *data) {
      int i = 0;
      float temperature = read_temperature(); 
      for (i = 0; i < 3; i++) {
        data[i] = readDistance(DIST_SENSOR_TRIGGER_PIN1, DIST_SENSOR_ECHO_PIN1, temperature);
      }
      return data;
    }
    
private:
    
    float read_temperature() {
        //Serial.print("calling read_temperature()\n");     //for debugging
        /*read the voltage on the temperature pin*/
        float voltage = (float)analogRead(TEMPERATURE_PIN);
        /*scale it, taking into account the arduino's return range and the sensor's specs*/
        return (voltage * 500.0) / 1023.0;
    }
    
    void pulseOut(uint8_t pin, int microseconds) {
        //Serial.print("calling pulseOut()\n");         //for debugging
        // set the pin to high
        digitalWrite(pin, HIGH);
        // wait for the prescribed time
        delayMicroseconds(microseconds);
        // set the pin to low
        digitalWrite(pin, LOW);
    }
    
    float readDistance(uint8_t pulseOutPin, uint8_t pulseInPin, float temperature) {
        //Serial.print("calling readDistance()\n");           //for debugging
        // send the trigger pulse
        pulseOut(pulseOutPin, 10);
        // read the response pulse
        unsigned long pulseWidth = pulseIn(pulseInPin, HIGH);
        // compute the speed of sound
        float speedOfSound = 10000.0 / (331.5 + (0.6 * temperature));
        // compute the distance
        float distance = ((float)pulseWidth) / (2 * speedOfSound);
        //return max value if no objects are detected in range
        if (distance > DIST_MAX) {
          return DIST_MAX;
        }
        return distance;
    }
    
};







//these are global so they can be set up in setup() and used in loop()
ExternalData extDat;
float data[3] = {0, 0, 0};            //crude representation of the cache

void setup() {

  pinMode(DIST_SENSOR_TRIGGER_PIN1, OUTPUT);
  pinMode(DIST_SENSOR_ECHO_PIN1, INPUT);
  pinMode(TEMPERATURE_PIN, INPUT);
  Serial.begin(9600);
  extDat = ExternalData();
}

void loop() {

  extDat.get_distances(data);

  int i;
  for (i = 0; i < 3; i++) {
    Serial.print(data[i]);        //for debugging
    Serial.print("\t\t");
  }
  Serial.println("\nScanning: \n");

  for (i = 0; i < 20, i++) {
    Serial.println(exDat.get_distance_at_angle(9*i));
  }
  
  delay(100);
}

