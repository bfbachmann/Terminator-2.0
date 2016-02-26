#include <math.h>

// Max speed in cm/s. 
#define MAX_SPEED 61

// Geometric constants of robot vehicle.
// Wheel radius (R) and wheel to wheel length (L).
#define R 6.5
#define L 15

// PID controller tuning.
#define k_p 1.5
#define k_i 0
#define k_d 0

// Finite time step in ms.
#define dt 2

// Threshold
#define thresh 100

int followLine(float s1, float s2, float s3, float s4) {
  if(s1 > thresh) {
    go(1); //slightRight
  }
  else if(s4 > thresh) {
    go(2); // slightLeft
  }
  else
    go(0);
}

void go(int dir){
  float factor = 1.5;
  int E1 = 5;
  int E2 = 6;
  int M1 = 4;
  int M2 = 7;
  int left, right = 0;
  
  if(dir == 2) {
    left = 100;
    right = 65;
  }
  else if(dir == 1) {
    left = 65;
    right = 100;
  }
  else {
   left = 100;
   right = 100; 
  }
  
  digitalWrite(M1, HIGH); digitalWrite(M2, HIGH);
  analogWrite(E1, factor*left); analogWrite(E2, factor*right);
  Serial.println(factor*left);
}

void setup() {
   Serial.begin(9600); 
   pinMode(A0, INPUT);
   pinMode(A1, INPUT);
   pinMode(A2, INPUT);
   pinMode(A3, INPUT);
   delay(5000);
}

void loop() {
  float s1 = analogRead(A0);
  float s2 = analogRead(A1);
  float s3 = analogRead(A2);
  float s4 = analogRead(A3);
  followLine(s1,s2,s3,s4);
}
