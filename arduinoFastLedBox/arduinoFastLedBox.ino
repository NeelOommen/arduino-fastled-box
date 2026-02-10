#include <FastLED.h>

#define LED_PIN     5
#define NUM_LEDS    64
#define BRIGHTNESS  32
#define LED_TYPE    WS2812B
#define COLOR_ORDER GRB
CRGB leds[NUM_LEDS];
CRGB colorMap[NUM_LEDS];

#define UPDATES_PER_SECOND 24

#define SENSOR_MAX 1024
#define SENSOR_MIN 0
#define DEAD_ZONE 120

int sensorPin = A1; 
int sensorValue = 0;

int filteredValue = 0;

int display[8];

uint32_t lastFrameTime = 0;
const uint32_t FRAME_INTERVAL = 1000 / UPDATES_PER_SECOND;

int framePeak = 0;

void fillRainbowPalette();

void setup() {
  delay( 3000 ); // power-up safety delay
  FastLED.addLeds<LED_TYPE, LED_PIN, COLOR_ORDER>(leds, NUM_LEDS).setCorrection( TypicalLEDStrip );
  FastLED.setBrightness(  BRIGHTNESS );
  // Serial.begin(9600);
  fillRainbowPalette(0);
}

void loop() {
  sensorValue = analogRead(sensorPin);
  int ledCount = mapSensorReadToLEDCount(sensorValue, 18);
 
  ledCount = smooth(ledCount);

  if (ledCount > framePeak){
    framePeak = ledCount;
  }

  uint32_t now = millis();

  if (now - lastFrameTime >= FRAME_INTERVAL) {
    lastFrameTime = now;

    shiftInNewValueToDisp(framePeak);
    drawVisualizer();

    framePeak = 0;
  }
}

int mapSensorReadToLEDCount(int sensorValue, int maxVal) {
    if (sensorValue <= SENSOR_MIN + DEAD_ZONE)
        return 0;

    long adjusted =
        sensorValue - (SENSOR_MIN + DEAD_ZONE);

    long range =
        (SENSOR_MAX - SENSOR_MIN - DEAD_ZONE);

    long normalized = adjusted * 1024 / range;

    long curved = sqrt(normalized * 1024);

    return (curved * maxVal) / 1024;
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

int smooth(int newValue) {
    filteredValue = (filteredValue * 7 + newValue) / 8;
    return filteredValue;
}

void drawVisualizer(){
  clearLeds();

  for(int i = 0; i < 8; i++){
    for(int y = 0; y < display[i]; y++){
      setLed(i,y);
    }
  }

  FastLED.show();
}

void setLed(int x, int y){
  if(x >= 8 || y >= 8){
    return;
  }
  leds[x * 8 + y] = colorMap[x * 8 + y];
}

void shiftInNewValueToDisp(int newValue){
  for(int i = 0; i < 7; i++){
    display[i] = display[i + 1];
  }
  display[7] = newValue;
}

void fillRainbowPalette(uint8_t colorIndex){
  uint8_t brightness = 255;
    
  for( int i = 0; i < NUM_LEDS; ++i) {
    colorMap[i] = ColorFromPalette( RainbowColors_p, colorIndex, brightness, LINEARBLEND);
    colorIndex += 3;
  }
}