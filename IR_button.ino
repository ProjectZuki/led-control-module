// C++ code
//
#include "Adafruit_LEDBackpack.h"

#include <IRremote.h>
#include <Adafruit_NeoPixel.h>

#define IR_PIN        2   // define IR reciever pin
#define BUTTON_PIN    3   // define button pin
#define NEOPIXEL_PIN  6   // define led strip pin (neopixel)
#define NUM_PIXELS    48  // number of LEDs in strip

// initialize Adafruit NeoPixel library
Adafruit_NeoPixel strip(NUM_PIXELS, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);

int buttonState = 0;
int button      = 0;
int RED         = 0;
int GREEN       = 0;
int BLUE        = 0;

Adafruit_7segment led_display1 = Adafruit_7segment();

// Map the IR code to the corresponding remote button.
// The buttons are in this order on the remote:
//    0   1   2
//    4   5   6
//    8   9  10
//   12  13  14
//   16  17  18
//   20  21  22
//   24  25  26
//
// Return -1, if supplied code does not map to a key.
int mapCodeToButton(unsigned long code) {
  // For the remote used in the Tinkercad simulator,
  // the buttons are encoded such that the hex code
  // received is of the format: 0xiivvBF00
  // Where the vv is the button value, and ii is
  // the bit-inverse of vv.
  // For example, the power button is 0xFF00BF000

  // Check for codes from this specific remote
  if ((code & 0x0000FFFF) == 0x0000BF00) {
    // No longer need the lower 16 bits. Shift the code by 16
    // to make the rest easier.
    code >>= 16;
    // Check that the value and inverse bytes are complementary.
    if (((code >> 8) ^ (code & 0x00FF)) == 0x00FF) {
      return code & 0xFF;
    }
  }
  return -1;
}

int readInfrared() {
  int result = -1;
  // Check if we've received a new code
  if (IrReceiver.decode()) {
    // Get the infrared code
    unsigned long code = IrReceiver.decodedIRData.decodedRawData;
    // Map it to a specific button on the remote
    result = mapCodeToButton(code);
    // Enable receiving of the next value
    IrReceiver.resume();
  }
  return result;
}

void setColor() {
  if (button == 16) {
      RED = 255;
      GREEN = 0;
      BLUE = 0;
    } else if (button == 17) {
      RED = 0;
      GREEN = 255;
      BLUE = 0;
    } else if (button == 18) {
      RED = 0;
      GREEN = 0;
      BLUE = 255;
    }
}

void flash(){
  //  light LED to set color
  for (int i = 0; i < NUM_PIXELS; i++) {
      strip.setPixelColor(i, RED, GREEN, BLUE);  // Set pixel color
  }
  strip.show();  // Display the colors
  delay(10);    // keep lit for duration
  for (int i = 0; i < NUM_PIXELS; i++) {
      strip.setPixelColor(i, 0);  // Set pixel color
  }
  strip.show();
}

void setup()
{
  // initialize serial communication
  Serial.begin(9600);

  // initialize button pin
  pinMode(BUTTON_PIN, INPUT);
  
  // initialize neopixel strip
  strip.begin();
  strip.show();   // initialize all pixels to 'off'

  // initialize IR receiver with LED feedback
  IrReceiver.begin(IR_PIN, ENABLE_LED_FEEDBACK);
}

int prev_button = 0;

void loop()
{

  button = readInfrared();
  setColor();
  buttonState = digitalRead(BUTTON_PIN);

  if (button >= 0) {
    // TODO this is where the LED changes color
    // FOR SAMPLE: 16 = red, 17 - green, 18 = blue

    // set color

    if (button != prev_button) {
      Serial.println("Color set to:\n" +
                  String("Red: ") + String(RED) + "\n" +
                  String("Green:") + String(GREEN) + "\n" +
                  String("Blue:") + String(BLUE) + "\n");
    }
  }
	
  if (buttonState == HIGH) {
    flash();
  } 
  // else {
  //   // Turn off NeoPixel strip
  //   for (int i = 0; i < NUM_PIXELS; i++) {
  //     strip.setPixelColor(i, 0);  // Set pixel color (off)
  //   }
  //   strip.show();  // Display the colors
  // }
  delay(10);

  prev_button = button;

  delay(10); // Delay a little bit to improve simulation performance
}