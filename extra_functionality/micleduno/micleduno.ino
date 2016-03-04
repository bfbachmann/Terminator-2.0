#include <Adafruit_NeoPixel.h>
#include <avr/power.h>
 
#define N_PIXELS   16  // Number of pixels you are using
#define MIC_PIN    A0  // Microphone is attached to Trinket GPIO #2/Gemma D2 (A1)
#define LED_PIN    3  // NeoPixel LED strand is connected to GPIO #0 / D0
#define DC_OFFSET  0  // DC offset in mic signal
#define NOISE      100  // Noise/hum/interference in mic signal
#define SAMPLES    60  // Length of buffer for dynamic level adjustment
#define TOP        (N_PIXELS +1) // Allow dot ('indicator' of current volume) to go slightly off scale
 
byte
  peak      = 0,      // Used for falling dot
  dotCount  = 0,      // Frame counter for delaying dot-falling speed
  volCount  = 0;      // Frame counter for storing past volume data
  
int
  vol[SAMPLES],       // Collection of prior volume samples
  lvl       = 10,     // Current "dampened" audio level
  minLvlAvg = 0,      // For dynamic adjustment of graph low & high
  maxLvlAvg = 512;
 
Adafruit_NeoPixel strip = Adafruit_NeoPixel(N_PIXELS, LED_PIN, NEO_GRB + NEO_KHZ800);
void setup(){
 // This is the auto-speed doubler line, keep it in, it will
  // automatically double the speed when 16Mhz is selected!
  if (F_CPU == 16000000) clock_prescale_set(clock_div_1);
  
  memset(vol, 0, sizeof(vol));
  strip.begin();
  strip.setBrightness(70);

}
 
void loop(){   

  uint8_t  i;
  uint16_t minLvl, maxLvl;
  int n, height;
  
  n   = analogRead(MIC_PIN);                 // Raw reading from mic 
  n   = abs(n - 512 - DC_OFFSET);            // Centre on zero
  n   = (n <= NOISE) ? 0 : (n - NOISE);      // Remove noise/hum
  lvl = ((lvl * 7) + n) >> 3;                // Dampens the reading
  
  // Calculate bar height based on dynamic min/max levels (fixed point):
  height = TOP * (lvl - minLvlAvg) / (long)(maxLvlAvg - minLvlAvg);
 
  if(height < 0L)       height = 0;      // Clip output
  else if(height > TOP) height = TOP;
  if(height > peak)     peak   = height; // Keep 'peak' dot at top
 
  // Color pixels based on rainbow gradient
  for(i=0; i<N_PIXELS; i++) {  
    if(i >= height)               
       strip.setPixelColor(i,   0,   0, 0);
    else 
       strip.setPixelColor(i,Wheel(map(i,0,strip.numPixels()-1,30,150)));
    } 
 
   strip.show(); // Update strip
 
  vol[volCount] = n;                      // Save sample for dynamic leveling
  if(++volCount >= SAMPLES) volCount = 0; // Advance/rollover sample counter
 
  // Get volume range of prior frames
  minLvl = maxLvl = vol[0];
  for(i=1; i<SAMPLES; i++) {
    if(vol[i] < minLvl)      minLvl = vol[i];
    else if(vol[i] > maxLvl) maxLvl = vol[i];
  }
  
  // minLvl and maxLvl indicate the volume range over prior frames, used
  // for vertically scaling the output graph (so it looks interesting
  // regardless of volume level).  If they're too close together though
  // (e.g. at very low volume levels) the graph becomes super coarse
  // and 'jumpy'...
  
  if((maxLvl - minLvl) < TOP) maxLvl = minLvl + TOP;
  minLvlAvg = (minLvlAvg * 63 + minLvl) >> 6;        // dampen min/max levels
  maxLvlAvg = (maxLvlAvg * 63 + maxLvl) >> 6;        // (fake rolling average)
}
 
// Input a value 0 to 255 to get a colour value.
// The colours are a transition r - g - b - back to r.
uint32_t Wheel(byte WheelPos) {
  if(WheelPos < 85) {
   return strip.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
  } else if(WheelPos < 170) {
   WheelPos -= 85;
   return strip.Color(255 - WheelPos * 3, 0, WheelPos * 3);
  } else {
   WheelPos -= 170;
   return strip.Color(0, WheelPos * 3, 255 - WheelPos * 3);
  }
}
