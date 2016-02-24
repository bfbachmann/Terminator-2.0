#include <Math.h>

#define NOFIELD 505L

// This is used to convert the analog voltage reading to milliGauss
#define TOMILLIGAUSS 1953L  // For A1301: 2.5mV = 1Gauss, and 1024 analog steps = 5V, so 1 step = 1953mG
#define THRESHOLD 200L // Threshold for digital HIGH (magnet is detected)
// #define TOMILLIGAUSS 3756L  // For A1302: 1.3mV = 1Gauss, and 1024 analog steps = 5V, so 1 step = 3756mG

int time0;
int time1;
int timeDif;
double circumference = M_PI * 2;
long field;
long newField;

void setup() 
{
  Serial.begin(9600);
  time0 = 0;
  field = -100;
}

long getField(int pin)
{
  int raw = analogRead(pin);   // Range : 0..1024

  long compensated = raw - NOFIELD;                 // adjust relative to no applied field 
  long gauss = compensated * TOMILLIGAUSS / 1000;   // adjust scale to Gauss

  return gauss;
}

double getSpeed(){
  return circumference/timeDif*1000000.0;
}

void loop(){
  newField = getField(0);
  if(newField < field){
    time1 = micros();
    timeDif = time1-time0;
    time0 = time1;
  }
  field = newField;
}

