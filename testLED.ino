#include <Adafruit_NeoPixel.h>

#define LED_PIN     2  // Pin connected to the data input of the RGB LED strip
#define BUTTON_PIN  3  // Pin connected to the button
#define NUM_LEDS    8  // Number of LEDs in your strip (adjust as needed)

// Create a NeoPixel object for the LED strip
Adafruit_NeoPixel strip = Adafruit_NeoPixel(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);

// Variable to store the previous button state
bool buttonWasPressed = false;

void setup() {
  strip.begin();       // Initialize the strip
  strip.clear();       // Ensure all LEDs are off initially
  strip.show();        // Apply changes to make sure LEDs are off
  
  Serial.begin(9600);  // Start serial communication at 9600 baud rate
  pinMode(BUTTON_PIN, INPUT_PULLUP); // Configure button with internal pull-up
}

void loop() {
  // Read the current state of the button
  bool buttonPressed = (digitalRead(BUTTON_PIN) == LOW);

  // Check if the button is pressed (transition from HIGH to LOW)
  if (buttonPressed && !buttonWasPressed) { 
    Serial.println("Button pressed!");  // Print to Serial Monitor once
  }

  // Continuous flashing logic
  // Turn on all LEDs (example: red)
  for (int i = 0; i < NUM_LEDS; i++) {
    strip.setPixelColor(i, strip.Color(255, 0, 0)); // Red color
  }
  strip.show(); // Apply the changes to light up the strip

  delay(500); // Delay for half a second (controls flashing speed)

  // Turn off all LEDs
  strip.clear(); // Set all LEDs to black (off)
  strip.show(); // Apply changes to turn off the strip

  delay(500); // Delay for half a second

  // Update the previous state of the button
  buttonWasPressed = buttonPressed; // Keep track of the last state
}
