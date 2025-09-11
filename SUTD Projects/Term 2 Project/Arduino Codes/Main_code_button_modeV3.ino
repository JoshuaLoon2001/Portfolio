

#include <Wire.h>
#include "MAX30105.h"
#include "heartRate.h"
MAX30105 particleSensor;



const byte RATE_SIZE = 4; //Increase this for more averaging. 4 is good.
byte rates[RATE_SIZE]; //Array of heart rates
byte rateSpot = 0;
long lastBeat = 0; //Time at which the last beat occurred
float beatsPerMinute;
int beatAvg;

/////////////////////////////////// Motor Code ////////////////////////////
#include <Stepper.h>

#define IN1 19
#define IN2 18
#define IN3 5
#define IN4 17

// Defines the number of steps per rotation
const int stepsPerRevolution = 2048;

// Creates an instance of stepper class
// Pins entered in sequence IN1-IN3-IN2-IN4 for proper step sequence
Stepper myStepper = Stepper(stepsPerRevolution, IN1, IN3 ,IN2, IN4);

///////////////////////////////////voice record code////////////////////////////////////////////////


int REC_BUTTON= 12;  // Digital pin connected to the record button of ISD1820 module
int PLAY_BUTTON= 16;  // Digital pin connected to the play button of ISD1820 module


/////////////////////////////////////// Light Code/////////////////////////////////////////////

#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
 #include <avr/power.h> // Required for 16 MHz Adafruit Trinket
#endif

// Which pin on the Arduino is connected to the NeoPixels?
#define PIN        13 // On Trinket or Gemma, suggest changing this to 1

// How many NeoPixels are attached to the Arduino?
#define NUMPIXELS 30 // Popular NeoPixel ring size

// When setting up the NeoPixel library, we tell it how many pixels,
// and which pin to use to send signals. Note that for older NeoPixel
// strips you might need to change the third parameter -- see the
// strandtest example for more information on possible values.
Adafruit_NeoPixel pixels(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);
#define DELAYVAL 500 

//////////////////////////////REC PLAY CLOSE MOTOR(Beat) physical buttons/////////////////////////////////////////////////////
#define recbutton   25
#define playbutton  26
#define closebutton 27
#define beatbutton 33

/////////////////////////////////////////////////////////////////////////////////



void setup()
{
  Serial.begin(115200);

  //////////////////////////////////////Arduino I/O Declare////////////////////////////////////////////////

  pinMode(REC_BUTTON, OUTPUT);
  pinMode(PLAY_BUTTON, OUTPUT);
  
  pinMode(recbutton, INPUT_PULLUP);
  pinMode(playbutton, INPUT_PULLUP);
  pinMode(closebutton, INPUT_PULLUP);
  pinMode(beatbutton, INPUT_PULLUP);
  
  pixels.setBrightness(10); // Brightness setting
  
  Serial.println("Initializing...");
  delay(1000);
  
  // Initialize sensor
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) //Use default I2C port, 400kHz speed
  {
    Serial.println("MAX30105 was not found. Please check wiring/power. ");
    while (1);
  }
  
  Serial.println("Place your index finger on the sensor with steady pressure.");

  particleSensor.setup(); //Configure sensor with default settings
  particleSensor.setPulseAmplitudeRed(0x0A); //Turn Red LED to low to indicate sensor is running
  particleSensor.setPulseAmplitudeGreen(0); //Turn off Green LED
 
    




////////////////////////////////////////Light Code///////////////////////////////////////////////

#if defined(__AVR_ATtiny85__) && (F_CPU == 16000000)
  clock_prescale_set(clock_div_1);
#endif
  pixels.begin(); // INITIALIZE NeoPixel strip object (REQUIRED)
}
////////////////////////////////////////////////////////////////////////////////////////////////////



////////////////////////////////////////////////////////////////////////////////////////////////////



void loop()
{ 


////////////// Button/ Mode Declaration /////////////////////////////////////


    // Print test buttons on serial monitor
  int recOff= digitalRead(recbutton);
  int playOff= digitalRead(playbutton);
  int closeOff= digitalRead(closebutton);
  int beatOff= digitalRead(beatbutton);

  int alter;
  long irValue = particleSensor.getIR();


//////////////////////////////////////////MenuMode ////////////////////////////   






  ////////////////////////////////////////// Heart rate loop/////////////////////////////////
      if (checkForBeat(irValue) == true)
      {
        //We sensed a beat!
        long delta = millis() - lastBeat;
        lastBeat = millis();
    
        beatsPerMinute = 60 / (delta / 1000.0);
    
        if (beatsPerMinute < 255 && beatsPerMinute > 20)
        {
          rates[rateSpot++] = (byte)beatsPerMinute; //Store this reading in the array
          rateSpot %= RATE_SIZE; //Wrap variable
    
          //Take average of readings
          beatAvg = 0;
          for (byte x = 0 ; x < RATE_SIZE ; x++)
            beatAvg += rates[x];
          beatAvg /= RATE_SIZE;
        }
      }


   ////////////////////////////////////// Finger Not Sensed /////////////////////////////////////////
  if (irValue < 50000)
  {
      alter=0;
      Serial.print(" No finger?");
      Serial.println();
     beatsPerMinute=0;
     beatAvg=0;
    Serial.println("Red= Record. Blue = play. Black= exit record/ play mode. Yellow: HeartPulse mode. Sensor: Heart Rate mode");
  
    for(int i=0; i<NUMPIXELS; i++) { // For each pixel...

    // pixels.Color() takes RGB values, from 0,0,0 up to 255,255,255
    // pixel white color:
    pixels.setPixelColor(i, pixels.Color(255, 255, 255));

    pixels.show();   // Send the updated pixel colors to the hardware.
    }
    
  }
  //////////////////////////////////// Finger Sensed ///////////////////////////////////////////////////
  
  if (irValue >= 50000)
  {
    //////////////////////////////////Heart rate Sensor//////////////////////////////////////////////
    Serial.print("IR=");
    Serial.print(irValue);
    Serial.print(", BPM=");
    Serial.print(beatsPerMinute);
    Serial.print(", Avg BPM=");
    Serial.print(beatAvg);
    Serial.println();    
    
    if (beatsPerMinute >60 || closeOff == LOW) // When BPM> 70 or black closebutton skip is pressed)
    {
      
    for(int i=0; i<NUMPIXELS; i++) { // For each pixel...

    // pixels.Color() takes RGB values, from 0,0,0 up to 255,255,255
    // Here we're using a moderately bright green color:
    pixels.setPixelColor(i, pixels.Color(0, 150, 0));

    pixels.show();   // Send the updated pixel colors to the hardware.
   
    delay(DELAYVAL); // Pause before next pass through loop

    //Blink Light
    if (i== NUMPIXELS -1){  //Blink Light

    for(int k=0; k<2; k++) {
    for(int j=0; j<NUMPIXELS; j++) { // For each pixel...
    // pixels.Color() takes RGB values, from 0,0,0 up to 255,255,255
    // Turn off Pixel
    pixels.setPixelColor(j, pixels.Color(0, 0, 0));
    pixels.show(); 
    }
     
    delay(1000);
    
    for(int u=0; u<NUMPIXELS; u++) { // For each pixel...
    // pixels.Color() takes RGB values, from 0,0,0 up to 255,255,255
    // Here we're using a moderately bright green color:
    pixels.setPixelColor(u, pixels.Color(0, 150, 0));
    pixels.show(); 
    }
    delay(1000);
    }
    //Blink Light End      
    
      Serial.print(" Done!!!");
      Serial.println();
      delay(5000);
 
    }
    
    }
    } else if (beatsPerMinute<=60) {

      
   
          for(int i=0; i<NUMPIXELS; i++) { // For each pixel...
     
      if ( i%2==0 ){
      // Pixel turn white
      pixels.setPixelColor(i, pixels.Color(255, 255, 255));
      pixels.show();
      } else {
      // Pixel turn blue
      pixels.setPixelColor(i, pixels.Color(0, 0,127));
      pixels.show(); 
      }
      }

  
  }
  }
/////////////////////////////////////////Heart rate code end ////////////////////////////////////////


///////////////////Heart motor code////////////////////////////////////////////////////////
    if (beatOff==0)
    {
      for(int i=0; i<NUMPIXELS; i++) { // For each pixel...

    // pixels.Color() takes RGB values, from 0,0,0 up to 255,255,255
    // pixel yellow color:
    pixels.setPixelColor(i, pixels.Color(255, 255, 0));

    pixels.show();   // Send the updated pixel colors to the hardware.
    }
    
    // Blink light
    for(int i=0; i<10; i++) {
    // heartbeat motor on
    myStepper.setSpeed(17);
    myStepper.step(-stepsPerRevolution); 
    }
    
    for(int k=0; k<2; k++) {
    for(int j=0; j<NUMPIXELS; j++) { 
    pixels.setPixelColor(j, pixels.Color(0, 0, 0));
    pixels.show(); 
    }
     
    delay(1000);
    
    for(int u=0; u<NUMPIXELS; u++) { 
    pixels.setPixelColor(u, pixels.Color(255, 255, 0));
    pixels.show(); 
    }
    delay(1000);
    }      
      Serial.println();
      delay(5000);
    // Blink end
    }
 ////////////////////////Beat end/////////////////////////////////////////////////////


   




/////////////////////////////////Record on///////////////////////////////////////////////////



  if (recOff==0)
    {
        //////////////////// Set pixel light to zero /////////////////////////
    for(int i=0; i<NUMPIXELS; i++) { // For each pixel...

    // pixels.Color() takes RGB values, from 0,0,0 up to 255,255,255
    // Here we're using a moderately bright green color:
    pixels.setPixelColor(i, pixels.Color(0, 0, 0));

    pixels.show();   // Send the updated pixel colors to the hardware.
    }
    //////////////////////////////////////////////////////////////////////
    Serial.println();
    
    delay(1000);
      

  
      Serial.print(" Record voice here!!!");
      Serial.println("Recording...");
      digitalWrite(REC_BUTTON, HIGH);
      for(int i=0; i<5; i++) {
      theaterChase(pixels.Color(127, 0, 0), 50); // Red
      }
      delay(1000); // Adjust as needed
      Serial.println("Recording finished.");
      digitalWrite(REC_BUTTON, LOW);
      
      //Blink Light 
      for(int i=0; i<NUMPIXELS; i++) { // For each pixel...
      // pixels.Color() takes RGB values, from 0,0,0 up to 255,255,255
      // pixel no color:
      pixels.setPixelColor(i, pixels.Color(0, 0, 0));
      pixels.show();   // Send the updated pixel colors to the hardware.
      }
      delay(1000);
      for(int i=0; i<NUMPIXELS; i++) { // For each pixel...
      // pixels.Color() takes RGB values, from 0,0,0 up to 255,255,255
      // pixel red color:
      pixels.setPixelColor(i, pixels.Color(127, 0, 0));
      pixels.show();   // Send the updated pixel colors to the hardware.
      }
      //Blink Light End
      
      delay(5000);

    }
//////////////////////////////Voice playback Code /////////////////////////////////////
    
    if (playOff== 0)
    {
       
      Serial.println("Playing...");
      digitalWrite(PLAY_BUTTON,HIGH);
      for(int i=0; i<5; i++) {
      theaterChase(pixels.Color(0, 0, 127), 50); // Blue
      }
      delay(1000); // Adjust as needed
      Serial.println("Playback finished.");
      digitalWrite(PLAY_BUTTON, LOW);

      
      //Blink Light 
      for(int i=0; i<NUMPIXELS; i++) { // For each pixel...
      // pixels.Color() takes RGB values, from 0,0,0 up to 255,255,255
      // pixel no color:
      pixels.setPixelColor(i, pixels.Color(0, 0, 0));
      pixels.show();   // Send the updated pixel colors to the hardware.
      }
      delay(1000);
      for(int i=0; i<NUMPIXELS; i++) { // For each pixel...
      // pixels.Color() takes RGB values, from 0,0,0 up to 255,255,255
      // pixel blue color:
      pixels.setPixelColor(i, pixels.Color(0, 0, 127));
      pixels.show();   // Send the updated pixel colors to the hardware.
      }
      //Blink Light End
      
      delay(5000);
      beatAvg=0;
    }
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////
  

   

//// loop end///////////////////////////////////////////////////////////////////////////////////////////////////





////////////////////////////////////////////////////////////////////////////////////
void theaterChase(uint32_t c, uint8_t wait) {
  for (int j=0; j<10; j++) {  //do 10 cycles of chasing
    for (int q=0; q < 3; q++) {
      for (uint16_t i=0; i < pixels.numPixels(); i=i+3) {
        pixels.setPixelColor(i+q, c);    //turn every third pixel on
      }
      pixels.show();

      delay(wait);

      for (uint16_t i=0; i < pixels.numPixels(); i=i+3) {
        pixels.setPixelColor(i+q, 0);        //turn every third pixel off
      }
    }
  }
}

////////////////////////////////////////////////////////////////////////////////////
void rainbow(uint8_t wait) {
  uint16_t i, j;

  for(j=0; j<256; j++) {
    for(i=0; i<pixels.numPixels(); i++) {
      pixels.setPixelColor(i, Wheel((i+j) & 255));
    }
    pixels.show();
    delay(wait);
  }
}
void rainbowCycle(uint8_t wait) {
  uint16_t i, j;

  for(j=0; j<256*5; j++) { // 5 cycles of all colors on wheel
    for(i=0; i< pixels.numPixels(); i++) {
      pixels.setPixelColor(i, Wheel(((i * 256 / pixels.numPixels()) + j) & 255));
    }
    pixels.show();
    delay(wait);
  }
}
void theaterChaseRainbow(uint8_t wait) {
  for (int j=0; j < 256; j++) {     // cycle all 256 colors in the wheel
    for (int q=0; q < 3; q++) {
      for (uint16_t i=0; i < pixels.numPixels(); i=i+3) {
        pixels.setPixelColor(i+q, Wheel( (i+j) % 255));    //turn every third pixel on
      }
      pixels.show();

      delay(wait);

      for (uint16_t i=0; i < pixels.numPixels(); i=i+3) {
        pixels.setPixelColor(i+q, 0);        //turn every third pixel off
      }
    }
  }
}
////////////////////////////////////////////////////////////////////////////////////
uint32_t Wheel(byte WheelPos) {
  WheelPos = 255 - WheelPos;
  if(WheelPos < 85) {
    return pixels.Color(255 - WheelPos * 3, 0, WheelPos * 3);
  }
  if(WheelPos < 170) {
    WheelPos -= 85;
    return pixels.Color(0, WheelPos * 3, 255 - WheelPos * 3);
  }
  WheelPos -= 170;
  return pixels.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
}
////////////////////////////////////////////////////////////////////////////////////
