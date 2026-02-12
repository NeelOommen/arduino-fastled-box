#include <FastLED.h>
#include <arduinoFFT.h>

#define FFT_SIZE 64
#define SAMPLE_RATE 5000   // Hz
#define SAMPLE_INTERVAL_US (1000000 / SAMPLE_RATE)

#define LED_PIN     5
#define NUM_LEDS    64
#define BRIGHTNESS  32
#define LED_TYPE    WS2812B
#define COLOR_ORDER GRB
CRGB leds[NUM_LEDS];
CRGB colorMap[NUM_LEDS];

#define SENSOR_MAX 1024
#define SENSOR_MIN 0

#define NOISE_FLOOR 256

#define COLOR_BUTTON 7
#define VIS_BUTTON 9

enum VisualizationMode {
  VIS_AMPLITUDE,
  VIS_FFT
};

enum Palettes {
  RAINBOW,
  BLUE,
  BUTTER,
  COKE,
  CHARTREUSE,
  WHITE,
  PURPLE
};

Palettes currentPalette = BUTTER;

VisualizationMode currentMode = VIS_FFT;

int sensorPin = A1; 
int sensorValue = 0;

int sensitivityPin = A4;
int deadZonePin = A3;

int filteredValue = 0;

byte colorLastButtonState = HIGH;
byte visLastButtonState = HIGH;
unsigned long debounceDuration = 50; // millis
unsigned long colorLastTimeButtonStateChanged = 0;
unsigned long visLastTimeButtonStateChanged = 0;

int display[8];
int targetDisplay[8];

uint32_t lastFrameTime = 0;

int framePeak = 0;

double vReal[FFT_SIZE];
double vImag[FFT_SIZE];

float bandPeak[8] = {1,1,1,1,1,1,1,1};

ArduinoFFT<double> FFT = ArduinoFFT<double>(vReal, vImag, FFT_SIZE, SAMPLE_RATE);

uint16_t sampleIndex = 0;
uint32_t lastSampleMicros = 0;
bool fftReady = false;
bool colorButtonReady = true;

void fillRainbowPalette(uint8_t colorIndex);

void setup() {
  delay( 3000 ); // power-up safety delay
  pinMode(COLOR_BUTTON, INPUT_PULLUP);
  FastLED.addLeds<LED_TYPE, LED_PIN, COLOR_ORDER>(leds, NUM_LEDS).setCorrection( TypicalLEDStrip );
  FastLED.setBrightness(  BRIGHTNESS );
  fillRainbowPalette(0);
}

void loop() {
  sensorValue = analogRead(sensorPin);

  if (currentMode == VIS_FFT) {
  uint32_t now = micros();

    if (now - lastSampleMicros >= SAMPLE_INTERVAL_US) {
      lastSampleMicros += SAMPLE_INTERVAL_US;

      vReal[sampleIndex] = analogRead(sensorPin) - 512;
      vImag[sampleIndex] = 0;

      sampleIndex++;

      if (sampleIndex >= FFT_SIZE) {
        sampleIndex = 0;
        fftReady = true;
      }
    }
  }

  updatePaletteChoice();
  // updateVisChoice();

  updateVisualization();

  if (frameReady()) {

    if (currentMode == VIS_AMPLITUDE) {
      shiftInNewValueToDisp(framePeak);
    }

    if (currentMode == VIS_FFT) {
      applySmoothingAndDecay();
    }

    drawVisualizer();
  }
}

void updatePaletteChoice(){
  if (millis() - colorLastTimeButtonStateChanged > debounceDuration) {
    byte colorButtonState = digitalRead(COLOR_BUTTON);
    if (colorButtonState != colorLastButtonState) {
      colorLastTimeButtonStateChanged = millis();
      colorLastButtonState = colorButtonState;
      if(colorButtonState == LOW){
        rotatePalette();
      }
    }
  }
}

void updateVisChoice(){
  if (millis() - visLastTimeButtonStateChanged > debounceDuration) {
    byte visButtonState = digitalRead(VIS_BUTTON);
    if (visButtonState != visLastButtonState) {
      visLastTimeButtonStateChanged = millis();
      visLastButtonState = visButtonState;
      if(visButtonState == LOW){
        rotateVis();
      }
    }
  }
}

void rotateVis(){
  if(currentMode == VIS_FFT){
    currentMode = VIS_AMPLITUDE;
  }
  else if(currentMode == VIS_AMPLITUDE){
    currentMode = VIS_FFT;
  }
}

void rotatePalette(){
  if(currentPalette == BUTTER){
    currentPalette = RAINBOW;
  }
  else if(currentPalette == RAINBOW){
    currentPalette = BLUE;
  }
  else if(currentPalette == BLUE){
    currentPalette = COKE;
  }
  else if(currentPalette == COKE){
    currentPalette = CHARTREUSE;
  }
  else if(currentPalette == CHARTREUSE){
    currentPalette = WHITE;
  }
  else if(currentPalette == WHITE){
    currentPalette = PURPLE;
  }
  else if(currentPalette == PURPLE){
    currentPalette = BUTTER;
  }
}

void updateVisualization() {
  switch (currentMode) {
    case VIS_AMPLITUDE:
      updateAmplitude();
      break;

    case VIS_FFT:
      updateFFT();
      break;
  }


}

void updateAmplitude() {
  int ledCount = smooth(mapSensorReadToLEDCount(sensorValue, getSensitivity()));
  if (ledCount > framePeak){
    framePeak = ledCount;
  }
}

void updateFFT() {
  if (!fftReady){
    return;
  }

  fftReady = false;

  FFT.windowing(vReal, FFT_SIZE, FFT_WIN_TYP_HAMMING, FFT_FORWARD);

  FFT.compute(vReal, vImag, FFT_SIZE, FFT_FORWARD);
  FFT.complexToMagnitude(vReal, vImag, FFT_SIZE);

  computeFFTBands();
}

int mapSensorReadToLEDCount(int sensorValue, int maxVal) {
    if (sensorValue <= SENSOR_MIN + getDeadzone())
        return 0;

    long adjusted =
        sensorValue - (SENSOR_MIN + getDeadzone());

    long range =
        (SENSOR_MAX - SENSOR_MIN - getDeadzone());

    long normalized = adjusted * 1024 / range;

    long curved = sqrt(normalized * 1024);

    return (curved * maxVal) / 1024;
}

void computeFFTBands() {

  uint8_t bandFloors[8] = {500, 500, 500, 500, 200, 200, 200, 200};

  for (int band = 0; band < 8; band++) {

    double sum = 0;

    for (int i = band; i < band + 4;i++) {
      double toAdd = vReal[i];
      if(i == 0 || i == 1){
        toAdd = vReal[2];
      }
      sum += toAdd;
    }

    float value = sum / 4.0;

    if(value < bandFloors[band]){
      value=0;
    }

    int sens = getSensitivity();
    value *= (float) sens + (((8-band) / 2) * 0.2);
    value *= (1.0 + band * 0.15); 

    // --- update peak (fast attack)
    if (value > bandPeak[band]) {
      bandPeak[band] = bandPeak[band] * 0.8 + value * 0.2;
    }

    // --- slow decay of peak (auto gain control)
    int dead = getDeadzone();
    // dead -= (((8 - band) / 2) * 1);
    bandPeak[band] *= dead / 1000.0;

    // prevent collapse
    if (bandPeak[band] < 1){
      bandPeak[band] = 1;
    }

    // --- normalize to display height
    value = (value / (bandPeak[band] * 0.75)) * 8;

    if (value > 8) value = 8;
    if (value < 0) value = 0;

    targetDisplay[band] = (targetDisplay[band] * 3 + value) / 4;
  }
}

void applySmoothingAndDecay() {

  const int riseSpeed = 3;  // fast rise
  const int fallSpeed = 1;  // slow decay

  for (int i = 0; i < 8; i++) {

    if (targetDisplay[i] > display[i]) {
      // rise quickly
      display[i] += riseSpeed;
      if (display[i] > targetDisplay[i])
        display[i] = targetDisplay[i];
    }
    else if (targetDisplay[i] < display[i]) {
      // decay slowly
      display[i] -= fallSpeed;
      if (display[i] < 0)
        display[i] = 0;
    }
    else{
      display[i] = targetDisplay[i];
    }
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

int getSensitivity(){
  int sensitivityRawValue = analogRead(sensitivityPin);
  int mappedValue = 0;

  switch (currentMode) {
    case VIS_AMPLITUDE:{
      mappedValue = mapValue(sensitivityRawValue, 0, 1024, 8, 32);
      break;
    }
    case VIS_FFT:{
      mappedValue = mapValue(sensitivityRawValue, 0, 1024, 1, 20);
      break;
    }
    default: {
      mappedValue=0;
    }
  }
  return mappedValue;
}

int getDeadzone(){
  int deadZoneRawValue = analogRead(deadZonePin);
  int mappedValue = 0;

  switch (currentMode) {
    case VIS_AMPLITUDE:{
      mappedValue = mapValue(deadZoneRawValue, 0, 1024, 40, 800);
      break;
    }
    case VIS_FFT:{
      mappedValue = mapValue(deadZoneRawValue, 0, 1024, 900, 999);
      break;
    }
    default: {
      mappedValue=0;
    }
  }

  return mappedValue;
}

int mapValue(int raw, int inMin, int inMax, int outMin, int outMax){
  if (raw < inMin) raw = inMin;
  if (raw > inMax) raw = inMax;
  if (inMax == inMin) return outMin;

  long value = (long)(raw - inMin) * (outMax - outMin);
  value /= (inMax - inMin);
  return (int)(value + outMin);
}


/* VISUALIZATION FUNCTIONS */

void drawVisualizer(){
  clearLeds();

  fillColorPalette();

  for(int i = 0; i < 8; i++){
    for(int y = 0; y < display[i]; y++){
      setLed(i,y);
    }
  }

  FastLED.show();
}

void fillColorPalette(){
  if(currentPalette == RAINBOW){
    fillRainbowPalette(0);
  }
  else if(currentPalette == BLUE){
    fillBluePalette();
  }
  else if(currentPalette == COKE){
    fillCokePalette();
  }
  else if(currentPalette == BUTTER){
    fillButterPalette();
  }
  else if(currentPalette == WHITE){
    fillWhitePalette();
  }
  else if(currentPalette == PURPLE){
    fillPurplePalette();
  }
  else if(currentPalette == CHARTREUSE){
    fillChartreusePalette();
  }
}

void setLed(int x, int y){
  if(x >= 8 || y >= 8){
    return;
  }
  leds[(7-x) * 8 + (7-y)] = colorMap[x * 8 + y];
}

void shiftInNewValueToDisp(int newValue){
  for(int i = 0; i < 7; i++){
    display[i] = display[i + 1];
  }
  display[7] = newValue;
  framePeak = 0;
}

bool frameReady(){
  uint32_t now = millis();
  int frameRate = 1;

  switch (currentMode) {
    case VIS_AMPLITUDE:{
      frameRate = 30;
      break;
    }
    case VIS_FFT:{
      frameRate = 60;
      break;
    }
    default: {
      frameRate=1;
    }
  }

  if(now - lastFrameTime >= (1000 / frameRate)){
    lastFrameTime = now;
    return true;
  }
  return false;
}

/*PALETTES*/

void fillRainbowPalette(uint8_t colorIndex){
  uint8_t brightness = 255;
    
  for( int i = 0; i < NUM_LEDS; ++i) {
    colorMap[i] = ColorFromPalette( RainbowColors_p, colorIndex, brightness, LINEARBLEND);
    colorIndex += 3;
  }
}

void fillBluePalette(){
  for( int i = 0; i < NUM_LEDS; ++i) {
    colorMap[i] = CRGB(0,16,250);
  }
}

void fillButterPalette(){
  for( int i = 0; i < NUM_LEDS; ++i) {
    colorMap[i] = CRGB(255,180,25);
  }
}

void fillCokePalette(){
  for( int i = 0; i < NUM_LEDS; ++i) {
    colorMap[i] = CRGB(170,7,1);
  }
}

void fillChartreusePalette(){
  for( int i = 0; i < NUM_LEDS; ++i) {
    colorMap[i] = CRGB(168,238,0);
  }
}

void fillWhitePalette(){
  for( int i = 0; i < NUM_LEDS; ++i) {
    colorMap[i] = CRGB(255,255,255);
  }
}

void fillPurplePalette(){
  for( int i = 0; i < NUM_LEDS; ++i) {
    colorMap[i] = CRGB(106,0,255);
  }
}