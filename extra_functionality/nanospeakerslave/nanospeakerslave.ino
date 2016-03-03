/**
 * a.wav = turning left
 * b.wav = turning right
 * c.wav = going straight
 */


#include <SD.h>                      // needed for the SD card (unsurprisingly)
#include <TMRpcm.h>                  // also need to include this library...
#include <Wire.h>                    // needed for I2C connection with master arduino 
#define SD_ChipSelectPin 10


//instantiates the TMRpcm object.
TMRpcm tmrpcm;

void setup() {
  tmrpcm.speakerPin = 9;

  if (!SD.begin(SD_ChipSelectPin)) {  // see if the card is present and can be initialized...
    return;                             // ...don't do anything if not
  }

  tmrpcm.volume(1);                     //sets the relative volume (varies for different Arduinos)
  tmrpcm.play("wally.wav");             //plays this sound to let us know it initialized properly
  //delay(2000);
  Wire.begin(8);
  Wire.onReceive(receiveEvent);

  //pinMode(buttons, INPUT);
  //pinMode(trigger_switch, INPUT);
  Serial.begin(9600);
}

void loop() {
}

/*
  
  /**
   read buttons play music

   read trigger play r2 scream

*/

void receiveEvent(int howMany) {
  String x;
  char* file_name = " .wav";
  
  file_name[0] = (char)Wire.read();
  Serial.println(file_name);
  if(!tmrpcm.isPlaying())
    tmrpcm.play(file_name);
  
}


