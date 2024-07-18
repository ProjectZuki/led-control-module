#include <FastLED.h>

#define NUM_LEDS 5
#define LED_PIN 6

CRGB led[NUM_LEDS];

void setup() {
  FastLED.addLeds<NEOPIXEL, LED_PIN>(led, NUM_LEDS);
  for (int i = 0; i < NUM_LEDS; i++){
    led[i] = CRGB(0, 0, 255);
  }


  FastLED.show();
}

void loop() {
  int i = 0;
  while (i <= 255) {
    setRed(i);
    delay(1);
    i++;
  }

  while (i >= 0){
    setRed(i);
    delay(1);
    i--;
  }

  while (i <= 255){
    setGreen(i);
    delay(1);
    i++;
  }

  while (i >= 0){
    setGreen(i);
    delay(1);
    i--;
  }

  while (i <= 255){
    setBlue(i);
    delay(1);
    i++;
  }

  while (i >= 0){
    setBlue(i);
    delay(1);
    i--;
  }
}


void setRed(int val){
  for (int i = 0; i < NUM_LEDS; i++){
    led[i] = CRGB(val, 0, 0);
  }

  FastLED.show();
}

void setGreen(int val){
  for (int i = 0; i < NUM_LEDS; i++){
    led[i] = CRGB(0, val, 0);
  }

  FastLED.show();
}

void setBlue(int val){
  for (int i = 0; i < NUM_LEDS; i++){
    led[i] = CRGB(0, 0, val);
  }

  FastLED.show();
}