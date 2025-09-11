#define REC_BUTTON 35   // Digital pin connected to the record button of ISD1820 module
#define PLAY_BUTTON 34  // Digital pin connected to the play button of ISD1820 module
#define REC_LED 4      // Digital pin connected to the record LED of ISD1820 module
#define PLAY_LED 5     // Digital pin connected to the play LED of ISD1820 module
#define BUSY_PIN 6     // Digital pin connected to the busy pin of ISD1820 module

void setup() {
  pinMode(REC_BUTTON, INPUT_PULLUP);
  pinMode(PLAY_BUTTON, INPUT_PULLUP);
  pinMode(REC_LED, OUTPUT);
  pinMode(PLAY_LED, OUTPUT);
  pinMode(BUSY_PIN, INPUT_PULLUP);
  
  Serial.begin(9600);
}

void loop() {
  if (digitalRead(REC_BUTTON) == LOW) {
    digitalWrite(REC_LED, HIGH); // Turn on record LED
    digitalWrite(PLAY_LED, LOW); // Turn off play LED
    
    // Start recording
    Serial.println("Recording...");
    delay(1000); // Adjust as needed
    digitalWrite(REC_LED, LOW); // Turn off record LED
    Serial.println("Recording finished.");
  }
  
  if (digitalRead(PLAY_BUTTON) == LOW && digitalRead(BUSY_PIN) == LOW) {
    digitalWrite(REC_LED, LOW); // Turn off record LED
    digitalWrite(PLAY_LED, HIGH); // Turn on play LED
    
    // Start playback
    Serial.println("Playing...");
    delay(1000); // Adjust as needed
    digitalWrite(PLAY_LED, LOW); // Turn off play LED
    Serial.println("Playback finished.");
  }
}
