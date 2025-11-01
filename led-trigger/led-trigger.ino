
/******************************************************************************
 * @file       led-trigger.ino
 * @brief      This source code file programs an Arduino Nano Every based on the
 *             ATMega4809 AVR processor to flash an ARGB LED strip on impact of
 *             a piezoelectric sensor. The device will be programmed to modify
 *             LED colors based on an RGB IR remote.
 *
 * @author     Willie Alcaraz ([Project]Zuki)
 * @date       August 2025
 * 
 * @note       Atmega328p
 *
 * @copyright  
 * © 2025 [Project]Zuki. All rights reserved.
 * 
 * This project and all files within this repository are proprietary software:
 * you can use it under the terms of the [Project]Zuki License. You may not use
 * this file except in compliance with the License. You may obtain a cop  y of the
 * License by contacting [Project]Zuki.
 * 
 * This software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
 * CONDITIONS OF ANY KIND, either express or implied. See the License for the
 * specific language governing permissions and limitations under the License.
 *
 * For more information, contact [Project]Zuki at:
 * willie.alcaraz@gmail.com
 * https://github.com/projectzuki
 * https://williealcaraz.dev
 *****************************************************************************/

#include <Arduino.h>          // standard Arduino functions
#include <stdint.h>           // fixed width integer types for compatibility
#include <IRremote.h>         // IR remote
#include <FastLED.h>          // NeoPixel ARGB
#include <EEPROM.h>           // save ROM data durong off state
#include <avr/pgmspace.h>     // PROGMEM
// #include <cppQueue.h>         // queue for RGB color states
// #include <stdint.h>


// ================================ IR Receiver ================================

#define IR_RECEIVER_PIN A4    // 18 -> A4

// IR

volatile unsigned long lastIRTime = 0;       // prev debounce time
unsigned long IRDebounceDelay = 500;        // debounce delay for IR

// Store known IR hex codes in Flash memory
const uint8_t known_hex_codes[] PROGMEM = {
  0x04, 0x05, 0x06, 0x07,
  0x08, 0x09, 0x0A, 0x0B,
  0x0C, 0x0D, 0x0E, 0x0F,
  0x10, 0x11, 0x12, 0x13,
  0x14, 0x15, 0x16, 0x17,
  0x18, 0x19, 0x1A, 0x1B,
  0x1C, 0x1D, 0x1E, 0x1F,
  0x40, 0x41, 0x44, 0x45,
  0x48, 0x49, 0x4C, 0x4D,
  0x50, 0x51, 0x54, 0x55,
  0x58, 0x59, 0x5C, 0x5D
};

#define CODE_COUNT (sizeof(known_hex_codes) / sizeof(known_hex_codes[0]))

// =============================== ARGB LED Strip ==============================

// ARGB pin
#define NUM_LEDS      170    // maximum number of LEDs in one given strip (confirmed 170 max for Tenors)
#define LED_PIN       10
#define MAX_INTENSITY 255    // 255 / 128 / 64 / 32 / 16 / 8
CRGB led[NUM_LEDS];

// delay threshold for flash duration in ms
unsigned int DELAY_THRESHOLD = 100;

// =================================== RGB LED =================================

#define LED_RED           5
#define LED_GREEN         6
#define LED_BLUE          9

// ================================ Color Data =================================

uint8_t RED         = 0;
uint8_t GREEN       = 0;
uint8_t BLUE        = 0;

// Color array for rainbow effect
int color_index = 0;

CRGB rainbowColors[] = {
  CRGB::Red,
  CRGB::Orange,
  CRGB::Yellow,
  CRGB::Green,
  CRGB::Blue,
  CRGB::Indigo,
  CRGB::Violet
};

CRGB rainbowColors2[] = {
  CRGB::Pink,
  CRGB::Cyan,
  CRGB::Magenta,
  CRGB::Purple,
  CRGB::Teal,
  CRGB::Lime,
  CRGB::Aqua
};

#define RAINBOW_COUNT (sizeof(rainbowColors) / sizeof(rainbowColors[0]))
#define RAINBOW2_COUNT (sizeof(rainbowColors2) / sizeof(rainbowColors2[0]))

// create a queue of CRGB values
// cppQueue CRGBQueue(sizeof(CRGB), 5, FIFO);
CRGB CRGBArr[6] = {CRGB{0, 0, 0}};
// queue for multicolor effect
// cppQueue multicolorQueue(sizeof(CRGB), 5, FIFO);
CRGB multicolorArr[6] = {CRGB{0, 0, 0}};
unsigned int multicolor_index = 0;
unsigned int multicolor_count = 0;

// =================================== BUTTON =================================

#define BUTTON_PIN        A3  // 21 -> A3

// debounce
int lastButtonState = LOW;                  // prev button state
int buttonState;                            // current button state
volatile unsigned long buttonPressTime = 0;   // prev debounce time
unsigned long buttonDebounceDelay = 1000;     // debounce delay for button

// ================================ EEPROM DATA ================================

// EEPROM addresses
#define RED_ADDR          0
#define GREEN_ADDR        1
#define BLUE_ADDR         2
#define JUMP3_ADDR        3
#define JUMP7_ADDR        4
#define PIEZO_THRESH_ADDR 5

// ================================ PIEZO SENSOR ===============================

// piezo pin
#define PIEZO_PIN     A0
unsigned int PIEZO_THRESH = 20;

// ================================= MODIFIERS =================================

// custom effect modifiers
volatile bool ledonrx = false;         // on/flash mode
volatile bool rainboweffectrx = false; // rainbow effect
volatile bool jump3 = false;           // rainbow colors
volatile bool jump7 = false;           // rainbow2 colors
volatile bool multicolor = false;      // multicolor effect
bool DIY1 = false;                     // ripple effect
volatile bool fade3 = false;           // fade off
volatile bool fade7 = false;           // fade on AND off

// ================================ Trail Effect ===============================

// For trail ripple effect
const int TRAIL_LENGTH = 15;
const int TRAIL_MAX = 30;       // Maximum number of simultaneous trails

struct Trail {
  int position;
  bool active;
  CRGB color;
};

Trail trails[TRAIL_MAX];
int nextTrailIndex = 0;   // Next available slot for a new trail

// =============================================================================

void setup() {
  // built-in LED
  // pinMode(LED_BUILTIN, OUTPUT);

  // button
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  // ARGB
  FastLED.addLeds<NEOPIXEL, LED_PIN>(led, NUM_LEDS);
  FastLED.setBrightness(MAX_INTENSITY);
  FastLED.show();

  // RGB LED
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_BLUE, OUTPUT);

  // adjust colors in the rainbowColors array to adhere to MAX_INTENSITY
  for (int i = 0; i < sizeof(rainbowColors) / sizeof(rainbowColors[0]); i++) {
    rainbowColors[i].r = scale8(rainbowColors[i].r, MAX_INTENSITY);
    rainbowColors[i].g = scale8(rainbowColors[i].g, MAX_INTENSITY);
    rainbowColors[i].b = scale8(rainbowColors[i].b, MAX_INTENSITY);
  }
  for (int i = 0; i < sizeof(rainbowColors2) / sizeof(rainbowColors2[0]); i++) {
    rainbowColors2[i].r = scale8(rainbowColors2[i].r, MAX_INTENSITY);
    rainbowColors2[i].g = scale8(rainbowColors2[i].g, MAX_INTENSITY);
    rainbowColors2[i].b = scale8(rainbowColors2[i].b, MAX_INTENSITY);
  }

  // piezo
  pinMode(PIEZO_PIN, INPUT);
  // debug
  Serial.begin(9600);

  // add delay to stabilize components
  delay(500);

  // IR
  // irrecv.enableIRIn();  // Old version of IRremote initialization
  IrReceiver.begin(IR_RECEIVER_PIN, ENABLE_LED_FEEDBACK);

  // restore color values
  eeprom_read();

  // Initialize all trails as inactive
  for (int i = 0; i < TRAIL_MAX; i++) {
    trails[i].active = false;
    trails[i].position = -1; // Set initial position to -1
  }
}

void loop() {

  check_button();

  // always-on LED indicates current color
  onLED();

  // check for either IR or transmitter data
  validate_IR();

  if (ledonrx) {
    rainboweffectrx = false;
    toggleOnOff();
  } else if (rainboweffectrx) {
    ledonrx = false;
    rainbow_effect();
  }

  // check for piezo sensor input
  piezo_trigger();
  // update any active non-blocking trails
  updateTrails();
}

/**
 * @brief Checks the button for input
 * 
 * This function will check the button for input. On input, the color from the queue
 * will be popped and set as the current color.
 * 
 * @return N/A
 */
void check_button() {
  // check button for input
  int currentButtonState = digitalRead(BUTTON_PIN);

  // check button state change
  if(currentButtonState != lastButtonState) {
    lastButtonState = currentButtonState;

    lastButtonState = currentButtonState;

    // check for press
    if (currentButtonState == LOW) {
      // Serial.println("Button pressed");
      if ((millis() - buttonPressTime) >= buttonDebounceDelay) {
        // reset cooldown
        buttonPressTime = millis();

        /// TODO: Optimized, check functionality 
        // // check for color queue
        // if (!CRGBQueue.isEmpty()) {
        //   CRGB color;
        //   CRGBQueue.pop(&color);
        //   RED = color.r;
        //   GREEN = color.g;
        //   BLUE = color.b;
        //   onLED();
        // }
        
        // Check CRGB array
        if (!(CRGBArr[0] == CRGB{0, 0, 0})) {
          // Serial.println("CRGB array is not empty");
          CRGB color = CRGBArr[0];
          RED = color.r;
          GREEN = color.g;
          BLUE = color.b;
          onLED();
          // "pop" from the array
          for (int i = 0; i < sizeof(CRGBArr) / sizeof(CRGBArr[0]) - 1; i++) {
            CRGBArr[i] = CRGBArr[i + 1];
          }
        }

        ///
      }
      

    }
  }
}

/**
 * @brief Checks if the hex code is a known IR signal
 * 
 * This function will check if the hex code is a known IR signal.
 * 
 * @param hex_code the hex code to check if it is a known IR signal
 * @return true if the hex code is a known IR signal, false otherwise
 */
bool isKnownCode(uint8_t hex_code) {
  uint8_t low = 0, high = CODE_COUNT - 1;

  while (low <= high) {
      uint8_t mid = low + (high - low) / 2;
      uint8_t mid_value = pgm_read_byte(&known_hex_codes[mid]);  // Read from Flash

      if (hex_code == mid_value) return true;
      (hex_code < mid_value) ? high = mid - 1 : low = mid + 1;
  }

  return false;
}

/**
 * @brief Processes the IR state
 * 
 * This function will check if an IR signal was received, check if the hex code
 * is valid, and process the hex code.
 * 
 * @return True if valid hex code, else False
 */
bool IRState() {
  if (IrReceiver.decode() && isKnownCode(IrReceiver.decodedIRData.command)) {
      processHexCode(IrReceiver.decodedIRData.command);
      IrReceiver.resume();
      return true;
  }
  return false;
}

/**
 * @brief Validates infrared signal
 * 
 * This function checks the IR receiver (global IrReceiver) for input. On input,
 * the hex code retrieved will be validated. On bad input, it will print the
 * signal code to serial.
 *
 * @return N/A
 */
bool validate_IR() {
  // IR remote instructions
  if (IrReceiver.decode()) {
    // Serial.println("Received IR signal: " + String(IrReceiver.decodedIRData.command, HEX));
    // store IR command
    uint16_t command = IrReceiver.decodedIRData.command;
    unsigned long currentMillis = millis();

    if ((currentMillis - lastIRTime) >= IRDebounceDelay) {

      if (IrReceiver.decodedIRData.protocol == UNKNOWN) {
        // Serial.println(F("Received noise or an unknown (or not yet enabled) protocol"));
        // // We have an unknown protocol here, print extended info
        // IrReceiver.printIRResultRawFormatted(&Serial, true);
        IrReceiver.resume(); // Do it here, to preserve raw data for printing with printIRResultRawFormatted()
        return false;
      } else 
      if (IrReceiver.decodedIRData.protocol == NEC) {
        // IrReceiver.printIRResultShort(&Serial);
        // IrReceiver.printIRSendUsage(&Serial);

        if (isKnownCode(IrReceiver.decodedIRData.command)) {
          // process IR signal
          // Serial.println("IR signal recieved: " + String(IrReceiver.decodedIRData.command));
          processHexCode(IrReceiver.decodedIRData.command);
        } else {
          IrReceiver.resume();
          return false;
        }

        IrReceiver.resume(); 
        // update IR signal time
        lastIRTime = currentMillis;
        return true;
      }
    }

    IrReceiver.resume();
  }

  return false;
}

/**
 * @brief Reads from EEPROM (Electrically Erasable Programmable Read-Only Memory)
 * 
 * This function reads and retrieves saved data from EEPROM
 * 
 * @return N/A
 */
void eeprom_read() {
  // read from EEPROM
  RED = EEPROM.read(RED_ADDR);
  GREEN = EEPROM.read(GREEN_ADDR);
  BLUE = EEPROM.read(BLUE_ADDR);
  jump3 = EEPROM.read(JUMP3_ADDR);
  jump7 = EEPROM.read(JUMP7_ADDR);
  PIEZO_THRESH = EEPROM.read(PIEZO_THRESH_ADDR);
}

/**
 * @brief Saves to EEPROM
 * 
 * This function saves the current color settings to EEPROM to load on startup.
 * 
 * @param red, green, blue the RGB colors to be saved to EEPROM
 * @return N/A
 */
void eeprom_save(int red, int green, int blue) {
  // write to EEPROM
  if (!jump3 && !jump7) {
    if (EEPROM.read(RED_ADDR) != red) EEPROM.write(RED_ADDR, red);
    if (EEPROM.read(GREEN_ADDR) != green) EEPROM.write(GREEN_ADDR, green);
    if (EEPROM.read(BLUE_ADDR) != blue) EEPROM.write(BLUE_ADDR, blue);
  }
  if (EEPROM.read(JUMP3_ADDR) != jump3) EEPROM.write(JUMP3_ADDR, jump3);
  if (EEPROM.read(JUMP7_ADDR) != jump7) EEPROM.write(JUMP7_ADDR, jump7);
  if (EEPROM.read(PIEZO_THRESH_ADDR) != PIEZO_THRESH) EEPROM.write(PIEZO_THRESH_ADDR, PIEZO_THRESH);
}

/**
 * @brief Checks for analog input from the piezoelectric sensor and flashes LED strip
 * 
 * This function will be called on loop checking for input from the piezoelectric sensor.
 * On input, will briefly flash the ARGB LED strip for a duration of DELAY_THRESHOLD
 * 
 * @return N/A
 */
void piezo_trigger() {
  if (analogRead(PIEZO_PIN) > PIEZO_THRESH) { // Piezo reads analog
      // multicolor effect
      if (multicolor) {
        CRGB color;

        // // rotate between selected colors
        // multicolorQueue.pop(&color);
        // RED = color.r;
        // GREEN = color.g;
        // BLUE = color.b;
        // multicolorQueue.push(&color);

        // rotate between selected colors
        color = multicolorArr[multicolor_index];
        RED = color.r;
        GREEN = color.g;
        BLUE = color.b;
        multicolor_index = (multicolor_index + 1) % (multicolor_count);
      }

      // Serial.println("Piezo triggered");

      // If DIY1 (ripple mode) is enabled, add a trail instead of blocking flash
      if (DIY1) {
        addTrail();
      } else {
        // Flash LED
        onARGB();
        delay(DELAY_THRESHOLD);
        offARGB();
      }
  }
}

// Add a new trail starting at position 0 (head of strip)
void addTrail() {
  for (int i = 0; i < TRAIL_MAX; i++) {
    if (!trails[i].active) {
      trails[i].position = 0;
      trails[i].active = true;
      trails[i].color = getColor();
      return;
    }
  }
}

// Update all active trails and render LEDs. Non-blocking; call frequently from loop().
void updateTrails() {
  // Clear LEDs
  fill_solid(led, NUM_LEDS, CRGB(0, 0, 0));

  bool anyActive = false;
  for (int t = 0; t < TRAIL_MAX; t++) {
    if (trails[t].active) {
      anyActive = true;
      for (int j = 0; j < TRAIL_LENGTH; j++) {
        int pos = trails[t].position - j;
        if (pos >= 0 && pos < NUM_LEDS) {
          led[pos] = trails[t].color;
        }
      }

      trails[t].position++;
      if (trails[t].position >= NUM_LEDS + TRAIL_LENGTH) {
        trails[t].active = false;
      }
    }
  }

  if (anyActive) {
    FastLED.show();
  }
}

/**
 * @brief Sets the active LED to the current color value
 * 
 * This function will set the built-in LED to the color value.
 * 
 * @return N/A
 */
void onLED() {
  // built-in LED
    analogWrite(LED_RED, RED / 4);       // 50% brightness
    analogWrite(LED_GREEN, BLUE / 4);    // Swap for LED using GBR order
    analogWrite(LED_BLUE, GREEN / 4);    // 50% brightness
}

/**
 * @brief Turn off active LED
 * 
 * This function turns off the active LED
 * 
 * @return N/A
 */
void offLED() {
  // built-in LED
  digitalWrite(LED_RED, 0);
  digitalWrite(LED_GREEN, 0);
  digitalWrite(LED_BLUE, 0);
}

/**
 * @brief Turn on LEDs
 * 
 * This function will activate the ARGB stip with the current color setting for
 *  RED, GREEN, BLUE
 * 
 * @return N/A
 */
void onARGB() {
  // do the thing but ARGB
    if (fade7) {
    for (int i = 0; i <= MAX_INTENSITY; i += 5) {
      fill_solid(led, NUM_LEDS, CRGB(RED, GREEN, BLUE).fadeLightBy(MAX_INTENSITY - i));
      FastLED.show();
      delay(1);  // Short delay for quicker fade-in

      // on trigger reset fade
      if (analogRead(PIEZO_PIN) > PIEZO_THRESH) {
        i = MAX_INTENSITY/4;
      }
    }
  } else {
    if (jump3) {
      fill_solid(led, NUM_LEDS, rainbowColors[(color_index++) % RAINBOW_COUNT]);
    } else if (jump7) {
      fill_solid(led, NUM_LEDS, rainbowColors2[(color_index++) % RAINBOW2_COUNT]);
    } else {
      fill_solid(led, NUM_LEDS, CRGB(RED, GREEN, BLUE));
    }
    FastLED.show();
  }

}

// // Variable to control the maximum total RGB value (scaled for 3A limit)
// int maxRGBValue = 75;  // Adjust this value to change the intensity for maintaining 3A limit

// void onARGB() {
//   // Ensure that the sum of RGB does not exceed maxRGBValue
//   int totalRGB = RED + GREEN + BLUE;

//   // Scale down if the totalRGB exceeds the maxRGBValue
//   if (totalRGB > maxRGBValue) {
//     float scaleFactor = (float)maxRGBValue / totalRGB;
//     RED = (int)(RED * scaleFactor);
//     GREEN = (int)(GREEN * scaleFactor);
//     BLUE = (int)(BLUE * scaleFactor);
//   }

//   // Perform the actual color change with the adjusted RGB values
//   if (fade7) {
//     for (int i = 0; i <= MAX_INTENSITY; i += 5) {
//       fill_solid(led, NUM_LEDS, CRGB(RED, GREEN, BLUE).fadeLightBy(MAX_INTENSITY - i));
//       FastLED.show();
//       delay(1);  // Short delay for quicker fade-in

//       // on trigger reset fade
//       if (analogRead(PIEZO_PIN) > PIEZO_THRESH) {
//         i = MAX_INTENSITY / 4;
//       }
//     }
//   } else {
//     fill_solid(led, NUM_LEDS, jump3 ? rainbowColors[(color_index++) % sizeof(rainbowColors)] : jump7 ? rainbowColors2[(color_index++) % sizeof(rainbowColors2)] : CRGB(RED, GREEN, BLUE));
//     FastLED.show();
//   }
// }


/**
 * @brief Turn off LEDs
 * 
 * This function turns off the ARGB strip
 * 
 * @return N/A
 */
void offARGB() {
  // do the off thing
  if (fade3 || fade7) {
    for (int i = MAX_INTENSITY; i >= 0; i-=5) {
      fill_solid(led, NUM_LEDS, CRGB(RED, GREEN, BLUE).fadeToBlackBy(MAX_INTENSITY - i));
      FastLED.show();
      delay(1);

      // on trigger reset fade
      if (analogRead(PIEZO_PIN) > PIEZO_THRESH) {
        i = MAX_INTENSITY;
      }
    }
  } else {
    fill_solid(led, NUM_LEDS, CRGB(0, 0, 0));
    FastLED.show();
  }
}

/**
 * @brief Toggle on/off for play/pause button
 * 
 * This function continously activates the ARGB strip until the play/pause button
 * is pressed again to deactivate the strip.
 * 
 * @return N/A
 */
// void toggleOnOff() {
//   bool ledon = true;
//   // toggle on/off for play/pause button
//   onARGB();
//   IrReceiver.resume();
//   while (ledon) {
//     // Serial.println("LED on");
//     if (IrReceiver.decode()) {

//       unsigned long currentMillis = millis();

//       if ((currentMillis - lastIRTime) >= IRDebounceDelay) {

//         if (IrReceiver.decodedIRData.protocol == UNKNOWN) {
//           // Serial.println(F("Received noise or an unknown (or not yet enabled) protocol"));
//           // We have an unknown protocol here, print extended info
//           // DEBUG
//           // IrReceiver.printIRResultRawFormatted(&Serial, true);
//           // IrReceiver.resume(); // Do it here, to preserve raw data for printing with printIRResultRawFormatted()
//           IrReceiver.resume();
//         } else {
//           IrReceiver.resume(); // Early enable receiving of the next IR frame
//           // DEBUG
//           // IrReceiver.printIRResultShort(&Serial);
//           // IrReceiver.printIRSendUsage(&Serial);

//           lastIRTime = currentMillis;
//         }
//         // Serial.println();

//         if (IrReceiver.decodedIRData.command == 0x41) {
//           // Serial.println("LED off");
//           ledon = false;
//           offARGB();
//           offLED();
//           break;
//         } else {
//           // apply modifications to color
//           processHexCode(IrReceiver.decodedIRData.command);
//         }
//         // update color in case of change
//         onARGB();
//         onLED();
//         delay(200);  // delay to reduce multiple inputs
//         IrReceiver.resume();
//       }
//     }
//   }
// }

void toggleOnOff() {
  bool ledon = true;
  const int tempRED = RED;
  const int tempGREEN = GREEN;
  const int tempBLUE = BLUE;
  
  int maxRGBValue = 150;  // Adjust this value to change the intensity for maintaining 3A limit
  // Ensure that the sum of RGB does not exceed maxRGBValue
  int totalRGB = RED + GREEN + BLUE;

  // Scale down if the totalRGB exceeds the maxRGBValue
  if (totalRGB > maxRGBValue) {
    float scaleFactor = (float)maxRGBValue / totalRGB;
    RED = (int)(RED * scaleFactor);
    GREEN = (int)(GREEN * scaleFactor);
    BLUE = (int)(BLUE * scaleFactor);
  }

  // toggle on/off for play/pause button
  onARGB();
  IrReceiver.resume();
  while (ledon) {
    // Serial.println("LED on");
    if (IrReceiver.decode()) {

      unsigned long currentMillis = millis();

      if ((currentMillis - lastIRTime) >= IRDebounceDelay) {

        if (IrReceiver.decodedIRData.protocol == UNKNOWN) {
          IrReceiver.resume();
        } else {
          IrReceiver.resume(); // Early enable receiving of the next IR frame
          lastIRTime = currentMillis;
        }

        if (IrReceiver.decodedIRData.command == 0x41) {
          // LED off command
          ledon = false;
          offARGB();
          offLED();
          break;
        } else {
          // Apply modifications to color
          processHexCode(IrReceiver.decodedIRData.command);
          
          // Ensure that the sum of RGB does not exceed maxRGBValue
          totalRGB = RED + GREEN + BLUE;

          // Scale down if the totalRGB exceeds the maxRGBValue
          if (totalRGB > maxRGBValue) {
            float scaleFactor = (float)maxRGBValue / totalRGB;
            RED = (int)(RED * scaleFactor);
            GREEN = (int)(GREEN * scaleFactor);
            BLUE = (int)(BLUE * scaleFactor);
          }
        }

        // Update color in case of change
        onARGB();
        onLED();
        delay(200);  // delay to reduce multiple inputs
        IrReceiver.resume();
      }
    }
  }

  // replace RGB values
  RED = tempRED;
  GREEN = tempGREEN;
  BLUE = tempBLUE;
}


/**
 * @brief Sets the values for RED, GREEN, BLUE
 * 
 * This function will set the values for RED, GREEN, BLUE to the CRGB color according
 *  to the input color.
 * 
 * @param color the CRGB color to set the values for RED, GREEN, BLUE
 * @return N/A
 */
void setColor(CRGB color) {
  // set new RGB values, constrain to max intensity value
  RED = scale8(color.r, MAX_INTENSITY);
  GREEN = scale8(color.g, MAX_INTENSITY);
  BLUE = scale8(color.b, MAX_INTENSITY);
}

/**
 * @brief Adjusts the color value
 * 
 * This function will adjust the color value based on the scale factor provided.
 * 
 * @param color the color value to adjust
 * @param scale the scale factor to adjust the color value
 * 
 * @return N/A
 */
void adj_color(uint8_t& color, int scale) {
  // Adjust color value
  int newColor = color + scale;

  // Constrain new color value to be within 1 and MAX_INTENSITY
  newColor = constrain(newColor, 0, MAX_INTENSITY);

  // Set the adjusted color value
  color = newColor;

  // Debug
  // Serial.println("Adjusted color: " + String(color));
  // Serial.println("Colors: " + String(RED) + ", " + String(GREEN) + ", " + String(BLUE));
}

CRGB getColor() {
  if (jump3) {
    return rainbowColors[(color_index++) % sizeof(rainbowColors)];
  } else if (jump7) {
    return rainbowColors2[(color_index++) % sizeof(rainbowColors2)];
  } else {
    return CRGB(RED, GREEN, BLUE);
  }
}

/**
 * @brief Creates a ripple effect on impact
 * 
 * This function will create a ripple effect on the ARGB LED strip each time the
 *  piezo sensor is hit.
 * 
 * TODO: Find way to end ripple effect or change color during effect.
 * 
 * @return N/A
 */
void ripple() {
  // Deprecated blocking ripple kept for reference. Use non-blocking addTrail()/updateTrails().
  return;
}

/**
 * @brief Creates a rainbow effect
 * 
 * This function will create a rainbow effect on the ARGB LED strip.
 * 
 * NOTE: This function will run indefinitely. Device must be powered off to reset.
 * 
 * @return N/A
 */
void rainbow_effect() {

  /// TODO: Allow effect ONLY on piezo trigger (saves battery, saves components)

  static unsigned long previousMillis = 0; // Static to retain value between calls
  const int interval = 20; // Interval for color update

  IrReceiver.resume(); // Ready to receive IR signals

  bool flag = false;

  while (!flag) {
    unsigned long currentMillis = millis();

    // Update colors at the specified interval
    if (currentMillis - previousMillis >= interval) {
      previousMillis += interval;

      // Calculate and update LED colors
      for (int i = 0; i < NUM_LEDS; i++) {
        led[i] = CHSV((i * 256 / NUM_LEDS) + (previousMillis / 10) % 256, 255, 255);
      }
      FastLED.show();
    }

      // Check for IR input
      flag = validate_IR();
  }
  offARGB();
  return;
}

/**
 * @brief Pushes RGB color to queue
 * 
 * This function will push the RGB color to the queue.
 * 
 * @param red, green, blue the RGB colors to be pushed to the queue
 * @return N/A
 */
void pushback(CRGB arr[], int red, int green, int blue) {
  // Save CRGB value to stack
  CRGB color = CRGB(red, green, blue);
  size_t size = 6;

  // Set next non-zero index to color
  for (int i = 0; i < size; i++) {
      if (arr[i] == CRGB{0, 0, 0}) {
          arr[i] = color;
          multicolor_count++;
          break; // Exit loop after adding the color
      }
  }

  // Show contents of array
  for (int i = 0; i < 3; i++) { // Flash colors three times
      for (int j = 0; j < size; j++) {
          led[j] = arr[j]; // Set LED colors based on array
      }
      FastLED.show();
      delay(200);

      fill_solid(led, NUM_LEDS, CRGB(0, 0, 0)); // Turn off all LEDs
      FastLED.show();
      delay(200);
  }
}

/**
 * @brief Visualizes the color queue
 * 
 * This function will flash the LED strip to show the colors in the queue.
 * 
 * @return N/A
 */
void check_colorQueue(CRGB arr[]) {

  unsigned int size = 6;

  if (arr[0] == CRGB{0, 0, 0}) {
    for (int i = 0; i <= MAX_INTENSITY; i += 5) {
      analogWrite(LED_RED, 255 * i / MAX_INTENSITY);
      analogWrite(LED_GREEN, 0 * i / MAX_INTENSITY);
      analogWrite(LED_BLUE, 0 * i / MAX_INTENSITY);
      delay(10);  // Gradual on
    }
    for (int i = MAX_INTENSITY; i >= 0; i -= 5) {
      analogWrite(LED_RED, 255 * i / MAX_INTENSITY);
      analogWrite(LED_GREEN, 0 * i / MAX_INTENSITY);
      analogWrite(LED_BLUE, 0 * i / MAX_INTENSITY);
      delay(10);  // Gradual off
    }
    return;
  }

  // Show contents of array
  for (int i = 0; i < 3; i++) { // Flash colors three times
    for (int j = 0; j < size; j++) {
        led[j] = arr[j]; // Set LED colors based on array
    }
    FastLED.show();
    delay(1000);

    fill_solid(led, NUM_LEDS, CRGB(0, 0, 0)); // Turn off all LEDs
    FastLED.show();
  }
}

/**
 * @brief Flashes the LED strip to confirm a save
 * 
 * This function will flash the LED strip to confirm a save to EEPROM.
 * 
 * @return N/A
 */
void flashConfirm(int val) {
  for (int i = 0; i < val; i ++) {
    // offLED();
    // delay(150);

    // onLED();
    // delay(150);

    for (int i = MAX_INTENSITY; i >= 0; i -= 5) {
      analogWrite(LED_RED, RED * i / MAX_INTENSITY);
      analogWrite(LED_GREEN, BLUE * i / MAX_INTENSITY);
      analogWrite(LED_BLUE, GREEN * i / MAX_INTENSITY);
      delay(4);  // Gradual off
    }
    for (int i = 0; i <= MAX_INTENSITY; i += 5) {
      analogWrite(LED_RED, RED * i / MAX_INTENSITY);
      analogWrite(LED_GREEN, BLUE * i / MAX_INTENSITY);
      analogWrite(LED_BLUE, GREEN * i / MAX_INTENSITY);
      delay(4);  // Gradual on
    }
  }
}

/**
 * @brief Process IR hex code
 * 
 * This function will process the IR hex code recieved from the IR remote, setting
 *  the appropriate colors for RED, GREEN, BLUE according to the hex code from the IR remote.
 * 
 * @param IRvalue the hex code recieved from the IR remote
 * @return -1 if the IR hex code is invalid
 */
int processHexCode(int IRvalue) {
  /*
  * process codes
  */
  switch(IRvalue) {
    // ==================== row 1 - Brightness UP/DOWN, play/pause, power ==========

    // increase delay (slower flash)
    case 0x5C:
      // FastLED.setBrightness(constrain(FastLED.getBrightness() +20, 1, 255));
      DELAY_THRESHOLD += 10;
      if (DELAY_THRESHOLD >= 1000) {
        DELAY_THRESHOLD = constrain(DELAY_THRESHOLD, 10, 1000);
        // indicate max brightness reached
        flashConfirm(2);
      }
      break;
    // decrease delay (quicker flash)
    case 0x5D:
      // FastLED.setBrightness(constrain(FastLED.getBrightness() -20, 1, 255));
      DELAY_THRESHOLD -= 10;
      if (DELAY_THRESHOLD <= 10) {
        DELAY_THRESHOLD = constrain(DELAY_THRESHOLD, 10, 1000);
        // indicate min brightness reached
        flashConfirm(2);
      }
      break;
    // play/pause
    case 0x41:
      // reverse lit status
      toggleOnOff();
      break;
    // PWR
    case 0x40:
      // disable IR Receiver
      IrReceiver.disableIRIn();
      // flash RGB LED red
      for (int i = 0; i < 3; i++) {
        analogWrite(LED_RED, 255);
        analogWrite(LED_GREEN, 0);
        analogWrite(LED_BLUE, 0);
        delay(200);
        analogWrite(LED_RED, RED);
        analogWrite(LED_GREEN, BLUE);
        analogWrite(LED_BLUE, GREEN);
        delay(200);
      }
      break;

    // ==================== row 2 | Color ==========================================
    case 0x58:
      setColor(CRGB::Red);
      break;
    case 0x59:
      setColor(CRGB::Green);
      break;
    case 0x45:
      setColor(CRGB::Blue);
      break;
    case 0x44:
      setColor(CRGB::White);
      break;

    // ==================== row 3 | Color ==========================================
    case 0x54:
      setColor(0xFF3F00);
      break;
    case 0x55:
      setColor(CRGB::LawnGreen);
      break;
    case 0x49:
      setColor(CRGB::Aqua);
      break;

    case 0x48:
      setColor(0xFF999A); 
      break;

    // ==================== row 4 | Color ==========================================
    case 0x50:
      setColor(0xCC3E00);
      break;
    case 0x51:
      setColor(CRGB::Cyan);
      break;
    case 0x4D:
      setColor(0xFF003A);
      break;
    case 0x4C:
      setColor(0xFF4C83);
      break;

    // ==================== row 5 | Color ==========================================
    case 0x1C:
      setColor(0x998600);
      break;
    case 0x1D:
      setColor(0x99FF33);
      break;
    case 0x1E:
      setColor(0xFF0066);
      break;
    case 0x1F:
      setColor(0xB09966);
      break;

    // ==================== row 6 | Color ==========================================
    case 0x18:
      setColor(0xFFC466);
      break;
    case 0x19:
      setColor(0x66FF33);
      break;
    case 0x1A:
      setColor(0x991493);
      break;
    case 0x1B:
      setColor(CRGB::LightSteelBlue);
      break;

    // ==================== row 7 | RED/BLUE/GREEN increase, QUICK ===================

    case 0x14:
      adj_color(RED, MAX_INTENSITY/5);
      break;
    case 0x15:
      adj_color(GREEN, MAX_INTENSITY/5);
      break;
    case 0x16:
      adj_color(BLUE, MAX_INTENSITY/5);
      break;
    // QUICK | Sensitivity down
    case 0x17:
    {
      PIEZO_THRESH -= 20;
      if (PIEZO_THRESH <= 0 || PIEZO_THRESH >= 1023) {  // unsigned int < 0 will become 65535
        PIEZO_THRESH = 20;
        // indicate max sensitivity reached
        flashConfirm(2);
      }
      break;
    }
    // ==================== row 8 | RED/BLUE/GREEN decrease, SLOW ====================

    case 0x10:
      adj_color(RED, MAX_INTENSITY/-5);
      break;
    case 0x11:
      adj_color(GREEN, MAX_INTENSITY/-5);
      break;
    case 0x12:
      adj_color(BLUE, MAX_INTENSITY/-5);
      break;
    // SLOW | Sensitivity up
    case 0x13:
    {
      PIEZO_THRESH += 20;
      if (PIEZO_THRESH >= 1023) {
        PIEZO_THRESH = constrain(PIEZO_THRESH, 0, 1023);
        // indicate min sensitivity reached
        flashConfirm(2);
      }
      break;
    }
    // ==================== row 9 | DIY 1-3, AUTO ====================================


    // DIY1: toggle non-blocking ripple mode
    case 0xC:
    {
      DIY1 = !DIY1;
      flashConfirm(4);
      return;
    }
    // DIY2
    case 0xD:
    {
      // custom multicolor
      // pushback(multicolorQueue, RED, GREEN, BLUE);
      pushback(multicolorArr, RED, GREEN, BLUE);
      multicolor = true;
      break;
    }
    //DIY3
    case 0xE:
      // add to color queue
      // pushback(CRGBQueue, RED, GREEN, BLUE);
      pushback(CRGBArr, RED, GREEN, BLUE);
      // // check current color queue
      // check_colorQueue();
      break;
    // AUTO(save) | IR lock
    case 0xF:
    {
      eeprom_save(RED, GREEN, BLUE);    // save current color
      flashConfirm(3);                   // flash to confirm save
      break;
    }
    // ==================== row 10 | DIY 4-6, FLASH ====================================

    // DIY4
    case 0x8:
      rainbow_effect();
      return;
    // DIY5
    case 0x9:
      // if (multicolor) {
      //   // check_colorQueue(multicolorQueue);
      //   check_colorQueue(multicolorArr);
      // }
      break;
    // DIY6
    case 0xA:
      // check current color queue
      // check_colorQueue(CRGBQueue);
      check_colorQueue(CRGBArr);
      break;
    // FLASH
    case 0xB:
      // modify the type of flash
      break;

    // ==================== row 11 | Jump3, Jump7, FADE3, FADE7 ========================

    // JUMP3
    case 0x4:
      // Rainbow color effect
      jump3 = true;
      return;   // return early to prevent color change
    // JUMP7
    case 0x5:
      // other rainbow effect
      jump7 = true;
      return;
      // break;
    // FADE3
    case 0x6:
      fade3 = true;
      return;
    // FADE7
    case 0x7:
      fade7 = true;
      return;
    
    // Default print error for debug
    default:
      // Serial.println("ERROR: IR recieved unknown value: " + String(IRvalue));
      // flashError(2);
      return -1;
  }

  // if (IRvalue != 0xD) {
  //   // disable multicolor mode if not adding colors
  //   multicolor = false;
  //   multicolor_index = 0;
  // }

  jump3 = false;
  jump7 = false;
  // fade3 = false;
  // fade7 = false;
  // modifier = false;
  fill_solid(led, NUM_LEDS, CRGB(0, 0, 0));
  FastLED.show();
  return IRvalue;
}
