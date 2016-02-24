#include <servo.h>

//assuming each trig_pin is in sequence, and each echo_pin is in sequence
#define trig1_pin
#define echo1_pin
#define trig2_pin
#define echo2_pin
#define trig3_pin
#define temp_pin
#define servo_pin

//create a servo object
Servo servo

void setup() {
  servo.attach(servo_pin);
  pinMode(trig1_pin, trig2_pin, trig3_pin, OUTPUT);
  pinMode(echo1_pin, echo2_pin, echo3_pin, INPUT);

}

float *poll_distance(void){
int i;
long duration;
float cm[3];

for (i = 0; i < 3; i++){
  digitalWrite(trig1_pin + i, LOW);
  delayMicroseconds(2);

  digitalWrite(trig1_pin + i, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig1_pin + i, LOW);

  duration = pulseIn(echo1_pin + i, HIGH)

  cm[i] = convert_to_cm(duration);
  delay(50);
}
}

float convert_to_cm(long microseconds){
  float pace;
  
  float temp = analogRead(temp_pin);
  temp = (temp * 5.0 * 100.0) / 1024.0;

  pace = (1 / (331.5 + (0.6 * temp)) * 10000);
  Serial.println(temp);Serial.print("*C")
  return (microseconds / pace) / 2.0;
}  
