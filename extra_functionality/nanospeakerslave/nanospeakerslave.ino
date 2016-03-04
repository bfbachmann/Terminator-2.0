/**
 * AUTONOMOUS MODE (a.wav)
 * --go forward    (b.wav)
 * --going forward and doing random sweep(c.wav)
 * --slowing down (d.wav)
 * --stops (let's check it out) (e.wav)
 * --turn left (f.wav)
 * --turn right (g.wav)
 * --something in front (h.wav)
 *  
 * 
 * LINE FOLLOWING MODE (i.wav)
 * --loses line (j.wav)
 *   
 */

#include <SD.h>                      // needed for the SD card (unsurprisingly)
#include <TMRpcm.h>                  // needed for playing .wav files from the SD card
#include <Wire.h>                    // needed for I2C connection with master arduino 


#define SD_ChipSelectPin 10
#define temp_pin A7
#define trigger_switch 7

TMRpcm tmrpcm;                        //instantiates the TMRpcm object.

char last_command;

void setup() {
  tmrpcm.speakerPin = 9;

  if (!SD.begin(SD_ChipSelectPin)) {  // see if the card is present and can be initialized...
    return;                           // ...don't do anything if not
  }

  tmrpcm.volume(1);                     //sets the relative volume
  //tmrpcm.play("k.wav");             //plays this sound to let us know it initialized properly
  
  Wire.begin(8);                        //assigns the Arduino a slave I2C address of 8
  Wire.onReceive(receiveEvent);         //when given data from the master, it performs this function
  Wire.onRequest(requestEvent);         //when requested data from the master, it performs this function

  //attachInterrupt(digitalPinToInterrupt(2), crashInterrupt, LOW);  //plays a specific sound if the robot crashes

  pinMode(trigger_switch, INPUT);
  
}

void loop() {
}

/**
 * Plays a given SD card .wav file with a given byte from the master Arduino.
 * 
 * Parameters: (optional) how many bytes of information is being sent.
 * 
 * Return: void
 */
void receiveEvent(int howMany) {
 
  char* file_name = " .wav";
  
  file_name[0] = (char)Wire.read();
  
  
  if (last_command == file_name[0]){
    if (file_name[0] == 'b' || file_name[0] == 'd') {
      return;
    }
  } else {
    last_command = file_name[0];
  }
  
  if(!tmrpcm.isPlaying()){
    tmrpcm.play(file_name);
  }
}

/**
 * Gives a temperature reading to the master Arduino on request.
 * 
 * Parameters: (optional) how many bytes of information is being sent.
 * 
 * Return: void
 */
void requestEvent(){
  Wire.write(analogRead(temp_pin));
}

/**
 * Plays a specific sound from the SD card, when the robot is interrupted.
 * 
 */
void crashInterrupt(){
  tmrpcm.play("l.wav");
}

