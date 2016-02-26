#include <Math.h>

#define NOFIELD 82L

// This is used to convert the analog voltage reading to milliGauss
#define TOMILLIGAUSS 1953L  // For A1301: 2.5mV = 1Gauss, and 1024 analog steps = 5V, so 1 step = 1953mG
#define THRESHOLD -50L // Threshold for digital HIGH (magnet is detected)

unsigned int time0;
unsigned int time1;
unsigned int timeDif;
char detected;
double circumference = M_PI * 2;
long field;
long newField;

void setup() 
{
  Serial.begin(9600);
  time0 = 0;
  field = -100;
  detected = 0;
}

long getField(int pin)
{
  int raw = analogRead(pin);   // Range : 0..1024

  long gauss = raw - NOFIELD;   // adjust relative to no applied field 

  return gauss;
}

double getSpeed(){
  return circumference/timeDif*1000.0;
}

void loop(){
  newField = getField(0);
  if(detected == 1){
    if(newField > THRESHOLD){
      detected = 0;
      Serial.println("Magnet lost");
      time1 = millis();
      timeDif = time1-time0;
      time0 = time1;
      Serial.print("Time dif: ");
      Serial.println(timeDif);
    }
  }
  else{
    if(newField < THRESHOLD){
      detected = 1;
      Serial.println("Magnet found");
    }
  }
  field = newField;
}

