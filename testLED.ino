
#include <Adafruit_NeoPixel.h>

#define BUTTON_PIN    2   // define button pin
#define NEOPIXEL_PIN  6   // define led strip pin (neopixel)
#define NUM_PIXELS    8   // number of LEDs in strip
// #define LED_PIN       13

// initialize Adafruit NeoPixel library
Adafruit_NeoPixel strip(NUM_PIXELS, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);

int buttonState = 0;

void setup() {
  pinMode(BUTTON_PIN, INPUT);
  // pinMode(LED_PIN, OUTPUT);
  strip.begin();  // initialize NeoPixel strip
  strip.show();   // initialize all pixels to 'off'
}

void loop() {
  buttonState = digitalRead(BUTTON_PIN);
  if (buttonState == HIGH) {
    // digitalWrite(LED_PIN, HIGH);  // Turn on the LED

    // Turn on NeoPixel strip
    for (int i = 0; i < NUM_PIXELS; i++) {
      strip.setPixelColor(i, 255, 0, 0);  // Set pixel color (red)
    }
    strip.show();  // Display the colors
    delay(10);
  } else {
    // digitalWrite(LED_PIN, LOW);  // Turn off the LED

    // Turn off NeoPixel strip
    for (int i = 0; i < NUM_PIXELS; i++) {
      strip.setPixelColor(i, 0);  // Set pixel color (off)
    }
    strip.show();  // Display the colors
  }
  delay(10);  // Delay a little bit to improve simulation performance
}


// =============================================================================

// sample code

// #define BUTTON_PIN 2
// #define LED_PIN 13

// int buttonState = 0;

// void setup()
// {
//   pinMode(BUTTON_PIN, INPUT);
//   pinMode(LED_PIN, OUTPUT);
// }

// void loop()
// {
//   // read the state of the pushbutton value
//   buttonState = digitalRead(BUTTON_PIN);
//   // check if pushbutton is pressed.  if it is, the
//   // buttonState is HIGH
//   if (buttonState == HIGH) {
//     // turn LED on
//     digitalWrite(LED_PIN, HIGH);
//   } else {
//     // turn LED off
//     digitalWrite(LED_PIN, LOW);
//   }
//   delay(10); // Delay a little bit to improve simulation performance
// }