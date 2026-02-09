#include <FastLED.h>

#define LED_PIN     5
#define NUM_LEDS    64
#define BRIGHTNESS  8
#define LED_TYPE    WS2812B
#define COLOR_ORDER GRB
CRGB leds[NUM_LEDS];

#define UPDATES_PER_SECOND 60

#define SENSOR_MAX 1024
#define SENSOR_MIN 0

CRGBPalette16 currentPalette;
TBlendType    currentBlending;

extern CRGBPalette16 myRedWhiteBluePalette;
extern const TProgmemPalette16 myRedWhiteBluePalette_p FL_PROGMEM;

int sensorPin = A1; 
int sensorValue = 0;

int filteredValue = 0;

void setup() {
  // put your setup code here, to run once:
  delay( 3000 ); // power-up safety delay
  FastLED.addLeds<LED_TYPE, LED_PIN, COLOR_ORDER>(leds, NUM_LEDS).setCorrection( TypicalLEDStrip );
  FastLED.setBrightness(  BRIGHTNESS );
    
  currentPalette = RainbowColors_p;
  currentBlending = LINEARBLEND;
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  sensorValue = analogRead(sensorPin);
  int ledCount = mapSensorReadToLEDCount(sensorValue);
  Serial.print("Sensor value: ");
  Serial.print(sensorValue);
  Serial.print(",Mapped value: ");
  Serial.println(ledCount);

  ledCount = smooth(ledCount);

  fillLeds(ledCount);

  FastLED.show();
  FastLED.delay(1000 / UPDATES_PER_SECOND);
}

int mapSensorReadToLEDCount(int sensorValue){
 return  (sensorValue * NUM_LEDS) / (SENSOR_MAX - SENSOR_MIN);
}

void fillLeds(int numLedsToFill){
  clearLeds();
  for(int i = 0; i<numLedsToFill; i++){
    leds[i] = CRGB::Red;
  }
}

void clearLeds(){
  for(int i = 0; i<NUM_LEDS; i++){
    leds[i] = CRGB::Black; 
  }
}

int smooth(int newValue){
  filteredValue = (filteredValue * 7 + newValue) / 8;
  return filteredValue;
}
