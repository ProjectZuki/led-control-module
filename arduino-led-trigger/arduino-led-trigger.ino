
/******************************************************************************
 * @file       arduino-led-trigger.ino
 * @brief      This source code file programs an Arduino Nano Every based on the
 *             ATMega4809 AVR processor to flash an ARGB LED strip on impact of
 *             a piezoelectric sensor. The device will be programmed to modify
 *             LED colors based on an RGB IR remote.
 *
 * @author     Willie Alcaraz ([Project]Zuki)
 * @date       July 2024
 *
 * @copyright  
 * © 2024 [Project]Zuki. All rights reserved.
 * 
 * This project and all files within this repository are proprietary software:
 * you can use it under the terms of the [Project]Zuki License. You may not use
 * this file except in compliance with the License. You may obtain a copy of the
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

#include <IRremote.h>   // IR remote
#include <FastLED.h>    // NeoPixel ARGB
#include <EEPROM.h>     // save ROM data durong off state

// IR receiver pin
#define IR_RECEIVER_PIN 2

// ARGB pin
#define serialnm      [112 114 111 106 101 99 116 122 117 107 105]
#define NUM_LEDS      144
#define LED_PIN       6
#define MAX_INTENSITY 16    // 255 / 128 / 64 / 32 / 16 / 8
CRGB led[NUM_LEDS];

// EEPROM addresses
#define RED_ADDR 0
#define GREEN_ADDR 1
#define BLUE_ADDR 2
#define RAINBOW_ADDR 3

uint8_t RED = 0;
uint8_t GREEN = 0;
uint8_t BLUE = 0;

// piezo pin
#define PIEZO_PIN     A0
unsigned int PIEZO_THRESH = 500;
 
// always on mode
bool ledon = false;

// IR
IRrecv irrecv(IR_RECEIVER_PIN);
decode_results results;

// modifier tied to PWR button
bool modifier = false;
// IR lock
bool IR_lock = false;

// delay threshold for flash duration
int DELAY_THRESHOLD = 100;

// For trail ripple effect
const int TRAIL_LENGTH = 25;
const int TRAIL_MAX = 10;  // Maximum number of simultaneous trails

struct Trail {
  int position;
  bool active;
  CRGB color;
};

Trail trails[TRAIL_MAX];

// Color array for rainbow effect
bool rainbow = false;
int color_index = 0;
CRGB RainbowColors[] = {
  CRGB::Red,
  CRGB::Orange,
  CRGB::Yellow,
  CRGB::Green,
  CRGB::Blue,
  CRGB::Indigo,
  CRGB::Violet
};

void setup() {
  // built-in LED
  pinMode(LED_BUILTIN, OUTPUT);

  // ARGB
  FastLED.addLeds<NEOPIXEL, LED_PIN>(led, NUM_LEDS);
  // for (int i = 0; i < NUM_LEDS; i++){
  //   led[i] = CRGB(0, 0, MAX_INTENSITY);
  // }
  FastLED.show();

  // adjust colors in the RainbowColors array to adhere to MAX_INTENSITY
  for (int i = 0; i < sizeof(RainbowColors) / sizeof(RainbowColors[0]); i++) {
    RainbowColors[i].r = scale8(RainbowColors[i].r, MAX_INTENSITY);
    RainbowColors[i].g = scale8(RainbowColors[i].g, MAX_INTENSITY);
    RainbowColors[i].b = scale8(RainbowColors[i].b, MAX_INTENSITY);
  }

  // piezo
  pinMode(PIEZO_PIN, INPUT);
  // debug
  Serial.begin(9600);

  // IR
  // Start the receiver, set default feedback LED
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
  // turn on built in LED to confirm functionality
  // digitalWrite(LED_BUILTIN, HIGH);

  validate_IR(IrReceiver);

  piezo_trigger();
}

/**
 * @brief Validates infrared signal
 * 
 * This function takes an IRrecv object to check for IR input. On input, the hex
 * code retrieved will be validated. On bad input, it will print to serial the exact
 * signal code that was recieved.
 * 
 * @param IrReceiver the IRrecv object reading infrared signals
 * @return N/A
 */
void validate_IR(IRrecv IrReceiver) {
  // IR remote instructions
  if (IrReceiver.decode()) {
    if (IrReceiver.decodedIRData.protocol == UNKNOWN) {
      Serial.println(F("Received noise or an unknown (or not yet enabled) protocol"));
      // We have an unknown protocol here, print extended info
      IrReceiver.printIRResultRawFormatted(&Serial, true);
      IrReceiver.resume(); // Do it here, to preserve raw data for printing with printIRResultRawFormatted()
    } else {
      IrReceiver.resume(); // Early enable receiving of the next IR frame
      IrReceiver.printIRResultShort(&Serial);
      IrReceiver.printIRSendUsage(&Serial);
    }
    Serial.println();

    if (IR_lock) {
      irlock();
    } else {
      if (processHexCode(IrReceiver.decodedIRData.command) == -1) {
        Serial.println("ERROR: IR recieved unknown value: " + String(IrReceiver.decodedIRData.command));
        flashError(1);
      }
    }

    delay(1000); // delay to prevent multiple inputs
  }
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
  rainbow = EEPROM.read(RAINBOW_ADDR);
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
  EEPROM.write(RED_ADDR, red);
  EEPROM.write(GREEN_ADDR, green);
  EEPROM.write(BLUE_ADDR, blue);
}

/**
 * @brief Locks IR signal to prevent modifications to current state
 * 
 * This functoin will prevent additional IR inputs from being received until
 * unlocked using IR hex code 0xF.
 * 
 * @return N/A
 */
void irlock() {
  unsigned long previousMillis = 0;
  const long interval = 200; // interval for LED indicator
  bool ledState = false;

  while (IR_lock) {
    unsigned long currentMillis = millis();

    // built-in LED will indicate lock state
    if (currentMillis - previousMillis >= interval) {
      previousMillis = currentMillis;
      ledState = !ledState;
      digitalWrite(LED_BUILTIN, ledState ? HIGH : LOW);
    }

    // check IR signal for unlock
    /// TODO: Maybe have 0x40 (PWR) input again to modify, then lock to prevent accidentally doing more than intended
    if (IrReceiver.decode()) {
      if (IrReceiver.decodedIRData.command == 0xF) {
        IR_lock = false; // Unlock IR signal
        Serial.println("IR unlocked");
      }
      IrReceiver.resume(); // Prepare to receive the next IR signal
    }

    // Check the piezo sensor
    piezo_trigger();
  }

  // Ensure the built-in LED is turned off when unlocked
  digitalWrite(LED_BUILTIN, LOW);
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
      // Flash LED
      onARGB();
      delay(DELAY_THRESHOLD);
      offARGB();
    }
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
  fill_solid(led, NUM_LEDS, rainbow? RainbowColors[(color_index++) % sizeof(RainbowColors)] : CRGB(RED, GREEN, BLUE));

  FastLED.show();
}

/**
 * @brief Turn off LEDs
 * 
 * This function turns off the ARGB strip
 * 
 * @return N/A
 */
void offARGB() {
  // do the off thing
  fill_solid(led, NUM_LEDS, CRGB(0, 0, 0));
  FastLED.show();
}

/**
 * @brief Toggle on/off for play/pause button
 * 
 * This function continously activates the ARGB strip until the play/pause button
 * is pressed again to deactivate the strip.
 * 
 * @return N/A
 */
void toggleOnOff() {
  // toggle on/off for play/pause button
  onARGB();
  while (ledon) {
    if (IrReceiver.decode()) {
      if (IrReceiver.decodedIRData.protocol == UNKNOWN) {
        Serial.println(F("Received noise or an unknown (or not yet enabled) protocol"));
        // We have an unknown protocol here, print extended info
        IrReceiver.printIRResultRawFormatted(&Serial, true);
        IrReceiver.resume(); // Do it here, to preserve raw data for printing with printIRResultRawFormatted()
      } else {
        IrReceiver.resume(); // Early enable receiving of the next IR frame
        IrReceiver.printIRResultShort(&Serial);
        IrReceiver.printIRSendUsage(&Serial);
      }
      Serial.println();

      if (IrReceiver.decodedIRData.command == 0x41) {
        ledon = false;
        offARGB();
        break;
      } else {
        // apply modifications to color
        processHexCode(IrReceiver.decodedIRData.command);
      }
      // update color in case of change
      onARGB();
      delay(1000);  // delay to reduce multiple inputs
      IrReceiver.resume();
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

      // increase brightness
      case 0x5C:
        adj_brightness(RED, GREEN, BLUE, 20);
        break;
      // decrease brightness
      case 0x5D:
        adj_brightness(RED, GREEN, BLUE, -20);
        break;
      // play/pause
      case 0x41:
        // reverse lit status
        ledon = !ledon;
        toggleOnOff();
        break;
      // PWR
      case 0x40:
        if (!modifier) {
          modifier = true;    // trigger alt modifier for next input
          led[0] = CRGB(0, 0, MAX_INTENSITY);
          FastLED.show();
          Serial.println("Modifier ON");
        } else {
          modifier = false;
          led[0] = CRGB(0, 0, 0);
          FastLED.show();
          Serial.println("Modifier OFF");
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
        setColor(CRGB::Orange);
        break;
      case 0x55:
        setColor(CRGB::LawnGreen);
        break;
      case 0x49:
        setColor(CRGB::Aqua);
        break;

      case 0x48:
        setColor(CRGB::DeepPink);
        break;

      // ==================== row 4 | Color ==========================================
      case 0x50:
        setColor(CRGB::Gold);
        break;
      case 0x51:
        setColor(CRGB::Cyan);
        break;
      case 0x4D:
        setColor(CRGB::DarkViolet);
        break;
      case 0x4C:
        setColor(CRGB::Coral);
        break;

      // ==================== row 5 | Color ==========================================
      case 0x1C:
        setColor(CRGB::DarkGoldenrod);
        break;
      case 0x1D:
        setColor(CRGB::DarkCyan);
        break;
      case 0x1E:
        setColor(CRGB::Magenta);
        break;
      case 0x1F:
        setColor(CRGB::PowderBlue);
        break;

      // ==================== row 6 | Color ==========================================
      case 0x18:
        setColor(CRGB::Yellow);
        break;
      case 0x19:
        setColor(CRGB::DarkTurquoise);
        break;
      case 0x1A:
        setColor(CRGB::DeepPink);
        break;
      case 0x1B:
        setColor(CRGB::LightSteelBlue);
        break;

      // ==================== row 7 | RED/BLUE/GREEN increase, QUICK ===================

      case 0x14:
        adj_color(RED, 1.1);
        break;
      case 0x15:
        adj_color(GREEN, 1.1);
        break;
      case 0x16:
        adj_color(BLUE, 1.1);
        break;
      // QUICK | Sensitivity down
      case 0x17:
      {
        if (!modifier) {
          // increase sensitivity
          PIEZO_THRESH -= 50;
          if (PIEZO_THRESH <= 0 || PIEZO_THRESH >= 1023) {  // unsigned int < 0 will become 65535
            PIEZO_THRESH = 10;
          }
        } else {
          modifier = false;
          // decrease delay (quicker flash)
          DELAY_THRESHOLD -= 50;
          led[0] = CRGB(0, 0, 0);
          FastLED.show();
        }
        break;
      }
      // ==================== row 8 | RED/BLUE/GREEN decrease, SLOW ====================

      case 0x10:
        adj_color(RED, 0.9);
        break;
      case 0x11:
        adj_color(GREEN, 0.9);
        break;
      case 0x12:
        adj_color(BLUE, 0.9);
        break;
      // SLOW | Sensitivity up
      case 0x13:
      {
        if (!modifier) {
          // decrease sensitivity
          PIEZO_THRESH += 50;
          if (PIEZO_THRESH >= 1023) {
            PIEZO_THRESH = constrain(PIEZO_THRESH, 0, 1023);
          }
        } else {
          modifier = false;
          // increase delay (slower flash)
          DELAY_THRESHOLD += 50;
          led[0] = CRGB(0, 0, 0);
          FastLED.show();
        }
        break;
      }
      // ==================== row 9 | DIY 1-3, AUTO ====================================


      // DIY1
      case 0xC:
      {
        // loop until IR signal is received
        while (true) {
          // continue checking for valid IR signal
          if (IrReceiver.decode()) {
            if (processHexCode(IrReceiver.decodedIRData.command) != -1) {
              break;
            }
            IrReceiver.resume();    // resume IR input
          }
          ripple();
        }
        break;
      }
      // DIY2
      case 0xD:
      {
        while (true) {
          // continue checking for valid IR signal
          if (IrReceiver.decode()) {
            if (processHexCode(IrReceiver.decodedIRData.command) != -1) {
              break;
            }
            IrReceiver.resume();    // resume IR input
          }
          ripple2();
        }
        break;
      }
      //DIY3
      case 0xE:
        break;
      // AUTO(save) | IR lock
      case 0xF:
      {
        if (!modifier) {
          eeprom_save(RED, GREEN, BLUE);    // save current color
          EEPROM.write(RAINBOW_ADDR, rainbow);
          flashConfirm();                   // flash to confirm save
        } else {
          modifier = false;
          // lock IR signal
          IR_lock = true;
          Serial.println("IR locked");
        }
        break;
      }
      // ==================== row 10 | DIY 4-6, FLASH ====================================

      // DIY4
      case 0x8:
        break;
      // DIY5
      case 0x9:
        break;
      // DIY6
      case 0xA:
        break;
      // FLASH
      case 0xB:
        // modify the type of flash
        break;

      // ==================== row 11 | Jump3, Jump7, FADE3, FADE7 ========================

      // JUMP3
      case 0x4:
        // Rainbow color effect
        rainbow = true;
        return;   // return early to prevent color change
      // JUMP7
      case 0x5:
        // other rainbow effect
        break;
      // FADE3
      case 0x6:
        break;
      // FADE7
      case 0x7:
        break;
      
      // Default print error for debug
      default:
        Serial.println("ERROR: IR recieved unknown value: " + String(IRvalue));
        flashError(2);
        return -1;
    }
  rainbow = false;
  return IRvalue;
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
void adj_color(uint8_t& color, float scale) {
  int newColor;
  
  // Use ceil for scaling up and floor for scaling down
  if (scale > 1) {
    newColor = ceil(color * scale);
  } else {
    newColor = floor(color * scale);
    if (newColor == 0) {
      color = 0;
      return;
    }
  }

  // Debug
  Serial.println("New Color: " + String(newColor));

  // Constrain new color value 
  if (newColor >= 1 && newColor <= MAX_INTENSITY) {
    color = newColor;
  } else if (newColor > MAX_INTENSITY) {
    color = MAX_INTENSITY;   // Constrain color to max intensity
  } else if (newColor < 1) {
    color = 1;    // Constrain color to min intensity
  }

  // Debug
  Serial.println("Color: " + String(color));
}

/**
 * @brief Adjusts the brightness of the color
 * 
 * This function will adjust the brightness of the color based on the value provided.
 *  The brightness value can be positive or negative and the scale factor is determined to
 *  ensure the color value is within the range of 1 to MAX_INTENSITY.
 * 
 * @param red the red color value
 * @param green the green color value
 * @param blue the blue color value
 * @param value the amount to adjust the brightness (positive or negative)
 * 
 * @return N/A
 */
void adj_brightness(uint8_t& red, uint8_t& green, uint8_t& blue, int value) {
  // min brightness to avoid black out (specifically values that weren't previously 0)
  const unsigned int MIN_BRIGHTNESS = 1;
  
  // find max value to scale all componets
  unsigned int maxComponent = max(red, max(green, blue));
  
  // can't scale 0, return
  if (maxComponent == 0) return;

  // check for components that were previously 0
  bool wasZeroRed = (red == 0);
  bool wasZeroGreen = (green == 0);
  bool wasZeroBlue = (blue == 0);

  // new value with brightness adjustment, ensure it is in range
  int newMaxComponent = maxComponent + value;
  newMaxComponent = constrain(newMaxComponent, MIN_BRIGHTNESS, MAX_INTENSITY);

  // apply scaling factor to non-zero components
  float scaleFactor = (float)newMaxComponent / maxComponent;
  red = (wasZeroRed ? 0 : constrain(red * scaleFactor, MIN_BRIGHTNESS, MAX_INTENSITY));
  green = (wasZeroGreen ? 0 : constrain(green * scaleFactor, MIN_BRIGHTNESS, MAX_INTENSITY));
  blue = (wasZeroBlue ? 0 : constrain(blue * scaleFactor, MIN_BRIGHTNESS, MAX_INTENSITY));
}

/**
 * @brief Creates a ripple effect on impact
 * 
 * This function will create a ripple effect on the ARGB LED strip each time the
 *  piezo sensor is hit.
 * 
 * @return N/A
 */
void ripple() {
  // Read the piezo value
  int piezoValue = analogRead(PIEZO_PIN);

  if (piezoValue > PIEZO_THRESH) {
    // Add a new trail if there is room
    for (int i = 0; i < TRAIL_MAX; i++) {
      if (!trails[i].active) {
        trails[i].position = 0;  // Initialize new trail position at the beginning
        trails[i].active = true;
        trails[i].color = rainbow ? RainbowColors[color_index] : CRGB(RED, GREEN, BLUE);
        if (rainbow) {
          color_index = (color_index + 1) % (sizeof(RainbowColors) / sizeof(RainbowColors[0]));
        }
        break;
      }
    }
  }

  // Clear the LED array for each frame
  fill_solid(led, NUM_LEDS, CRGB(0, 0, 0));
  
  // Update and display the trails
  for (int t = 0; t < TRAIL_MAX; t++) {
    if (trails[t].active) {
      // Draw the current trail with a gap
      for (int j = 0; j < TRAIL_LENGTH; j++) {
        int pos = trails[t].position - j;
        if (pos >= 0 && pos < NUM_LEDS) {
          led[pos] = trails[t].color;
        }
      }

      // Clear the LED just before the trail to create a gap
      int gapPos = trails[t].position - TRAIL_LENGTH;
      if (gapPos >= 0 && gapPos < NUM_LEDS) {
        led[gapPos] = CRGB(0, 0, 0);
      }
      
      // Update the position for the next frame
      trails[t].position++;

      // Deactivate the trail if it has moved past the LED strip
      if (trails[t].position >= NUM_LEDS + TRAIL_LENGTH + 1) { // Add 1 for the gap
        trails[t].active = false;
        trails[t].position = -1; // Reset position
      }
    }
  }

  FastLED.show();
  delay(1); // Adjust the delay for the speed of the ripple
}

/**
 * @brief Creates a ripple effect without requiring impact
 * 
 * This function will create a ripple effect on the ARGB LED strip without requiring
 *  input from the piezo sensor.
 * 
 * @return N/A
 */
void ripple2() {
  // Same ripple trail without the need of piezo trigger

  // Add a new trail if there is room
  for (int i = 0; i < TRAIL_MAX; i++) {
    if (!trails[i].active) {
      trails[i].position = 0;  // Initialize new trail position at the beginning
      trails[i].active = true;
      trails[i].color = rainbow ? RainbowColors[color_index] : CRGB(RED, GREEN, BLUE);
      if (rainbow) {
        color_index = (color_index + 1) % (sizeof(RainbowColors) / sizeof(RainbowColors[0]));
      }
      break;
    }
  }

  // Clear the LED array for each frame
  fill_solid(led, NUM_LEDS, CRGB(0, 0, 0));
  
  // Update and display the trails
  for (int t = 0; t < TRAIL_MAX; t++) {
    if (trails[t].active) {
      // Draw the current trail with a gap
      for (int j = 0; j < TRAIL_LENGTH; j++) {
        int pos = trails[t].position - j;
        if (pos >= 0 && pos < NUM_LEDS) {
          led[pos] = trails[t].color;
        }
      }
      
      // Update the position for the next frame
      trails[t].position++;

      // Deactivate the trail if it has moved past the LED strip
      if (trails[t].position >= NUM_LEDS + TRAIL_LENGTH + 1) { // Add 1 for the gap
        trails[t].active = false;
        trails[t].position = -1; // Reset position
      }
    }
  }

  FastLED.show();
  delay(1); // Adjust the delay for the speed of the ripple
}

/**
 * @brief Flashes the LED strip to confirm a save
 * 
 * This function will flash the LED strip to confirm a save to EEPROM.
 * 
 * @return N/A
 */
void flashConfirm() {
  for (int i = 0; i < 3; i ++) {
    led[0] = CRGB(RED, GREEN, BLUE);
    FastLED.show();
    delay(200);
    led[0] = CRGB(0, 0, 0);
    FastLED.show();
    delay(200);
  }
}

/**
 * @brief Flashes the LED strip to indicate an error
 * 
 * This function will flash the LED strip to indicate an error based on the error code.
 * 
 * @param errorcode the error code to flash the LED strip (number of flashes)
 * @return N/A
 */
void flashError(int errorcode) {
  /*
  * Flash error codes based on specific error.
  * 1: Invalid IR remote value recieved
  * 2: Unknown protocol from IR
  */
  /// TODO: Modify. Will react with any IR signals (e.g. iPhone Face ID and any other source of IR)
  for (int i = 0; i < errorcode; i++) {
    led[0] = CRGB(MAX_INTENSITY, 0, 0);
    FastLED.show();
    delay(50);
    led[0] = CRGB(0, 0, 0);
    FastLED.show();
  }
}
