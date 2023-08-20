#include <Adafruit_NeoPixel.h>

#define PIXEL_PIN    7     // DIN pin of the NeoPixel strip
#define NUM_PIXELS   26    // Number of NeoPixel LEDs in your strip
#define BUTTON_PIN   2     // Pin number for the button

Adafruit_NeoPixel strip(NUM_PIXELS, PIXEL_PIN, NEO_GRB + NEO_KHZ800);

int colorIndex = 5;       // Index to track the current color, starting 5 so it begins on red
bool ledOn = false;       // Flag to track if the LED is on

void setup() {
  strip.begin(); // Initialize the NeoPixel strip

  pinMode(BUTTON_PIN, INPUT_PULLUP); // Initialize the button pin as an input with internal pull-up resistor
}

void loop() {
  // Check if the button is pressed
  if (digitalRead(BUTTON_PIN) == LOW && !ledOn) {
    colorIndex = (colorIndex + 1) % 6; // Cycle through 6 colors
    setNeoPixelColorByIndex(colorIndex);
    flashAndTurnOff();
  }
}

// Function to set the NeoPixel color by index
void setNeoPixelColorByIndex(int index) {
  switch (index) {
    case 0: // Red
      setColor(255, 0, 0);
      break;
    case 1: // Green
      setColor(0, 255, 0);
      break;
    case 2: // Blue
      setColor(0, 0, 255);
      break;
    case 3: // Yellow (Red + Green)
      setColor(255, 255, 0);
      break;
    case 4: // Purple (Red + Blue)
      setColor(255, 0, 255);
      break;
    case 5: // Cyan (Green + Blue)
      setColor(0, 255, 255);
      break;
  }
}

// Function to set the NeoPixel color
void setColor(int redValue, int greenValue, int blueValue) {
  for (int i = 0; i < NUM_PIXELS; i++) {
    strip.setPixelColor(i, strip.Color(redValue, greenValue, blueValue));
  }
  strip.show();
}

// Function to turn off the NeoPixel LEDs
void turnOff() {
  for (int i = 0; i < NUM_PIXELS; i++) {
    strip.setPixelColor(i, strip.Color(0, 0, 0));
  }
  strip.show();
}

// Function to flash the NeoPixel strip in the selected color and then turn off
void flashAndTurnOff() {
  setNeoPixelColorByIndex(colorIndex); // Set the selected color
  delay(10); // Flash duration
  turnOff(); // Turn off the strip
//   delay(100); // Delay before re-enabling button press
}
