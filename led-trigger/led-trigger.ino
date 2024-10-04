
/******************************************************************************
 * @file       led-trigger.ino
 * @brief      This source code file programs an Arduino Nano Every based on the
 *             ATMega4809 AVR processor to flash an ARGB LED strip on impact of
 *             a piezoelectric sensor. The device will be programmed to modify
 *             LED colors based on an RGB IR remote.
 *
 * @author     Willie Alcaraz ([Project]Zuki)
 * @date       August 2024
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

#include <IRremote.h>         // IR remote
#include <FastLED.h>          // NeoPixel ARGB
#include <EEPROM.h>           // save ROM data durong off state
#include <cppQueue.h>         // queue for RGB color states
// #include <SoftwareSerial.h>   // HC-12 module

// IR receiver pin
#define IR_RECEIVER_PIN 18

// ARGB pin
#define NUM_LEDS      15    // maximum number of LEDs in one given strip (170)
#define LED_PIN       10
#define MAX_INTENSITY 255    // 255 / 128 / 64 / 32 / 16 / 8
CRGB led[NUM_LEDS];

#define LED_RED           5
#define LED_GREEN         6
#define LED_BLUE          9

#define BUTTON_PIN        21

// EEPROM addresses
#define RED_ADDR          0
#define GREEN_ADDR        1
#define BLUE_ADDR         2
#define JUMP3_ADDR        3
#define JUMP7_ADDR        4
#define PIEZO_THRESH_ADDR 5

uint8_t RED         = 0;
uint8_t GREEN       = 0;
uint8_t BLUE        = 0;

// create a queue of CRGB values
cppQueue CRGBQueue(sizeof(CRGB), 5, FIFO);
// queue for multicolor effect
cppQueue multicolorQueue(sizeof(CRGB), 5, FIFO);

// piezo pin
#define PIEZO_PIN     A0
unsigned int PIEZO_THRESH = 300;

// IR
IRrecv irrecv(IR_RECEIVER_PIN);
decode_results results;

// modifier tied to PWR button
// bool modifier = false;

// custom effect modifiers
bool ledonrx = false;         // on/flash mode
bool rainboweffectrx = false; // rainbow effect
bool jump3 = false;           // rainbow colors
bool jump7 = false;           // rainbow2 colors
bool multicolor = false;      // multicolor effect
bool DIY1 = false;            // ripple effect
bool fade3 = false;           // fade off
bool fade7 = false;           // fade on AND off

// delay threshold for flash duration in ms
int DELAY_THRESHOLD = 100;

// debounce
int lastButtonState = LOW;                  // prev button state
int buttonState;                            // current button state
unsigned long buttonPressTime = 0;   // prev debounce time
unsigned long buttonDebounceDelay = 1000;     // debounce delay for button

unsigned long lastIRTime = 0;       // prev debounce time
unsigned long IRDebounceDelay = 500;        // debounce delay for IR

// For trail ripple effect
const int TRAIL_LENGTH = 15;
const int TRAIL_MAX = 80;       // Maximum number of simultaneous trails

struct Trail {
  int position;
  bool active;
  CRGB color;
};

Trail trails[TRAIL_MAX];
int nextTrailIndex = 0;   // Next available slot for a new trail

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

// All known IR hex codes
const uint32_t known_hex_codes[] = {
  0x4,  0x5,  0x6,  0x7,
  0x8,  0x9,  0xA,  0xB,
  0xC,  0xD,  0xE,  0xF,
  0x10, 0x11, 0x12, 0x13,
  0x14, 0x15, 0x16, 0x17,
  0x18, 0x19, 0x1A, 0x1B,
  0x1C, 0x1D, 0x1E, 0x1F,
  0x40, 0x41, 0x44, 0x45,
  0x48, 0x49, 0x4C, 0x4D,
  0x50, 0x51, 0x54, 0x55,
  0x58, 0x59, 0x5C, 0x5D
};

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

  // IR
  // Start the receiver, set default feedback LED
  // IrReceiver.begin(IR_RECEIVER_PIN, ENABLE_LED_FEEDBACK);
  irrecv.enableIRIn();  // Old version of IRremote initialization

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
  validate_IR(IrReceiver);

  if (ledonrx) {
    rainboweffectrx = false;
    toggleOnOff();
  } else if (rainboweffectrx) {
    ledonrx = false;
    rainbow_effect();
  }

  // check for piezo sensor input
  piezo_trigger();
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
      Serial.println("Button pressed");
      if ((millis() - buttonPressTime) >= buttonDebounceDelay) {
        // reset cooldown
        buttonPressTime = millis();

        // check for color queue
        if (!CRGBQueue.isEmpty()) {
          CRGB color;
          CRGBQueue.pop(&color);
          RED = color.r;
          GREEN = color.g;
          BLUE = color.b;
        }
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
bool isKnownCode(uint32_t hex_code) {
  // find using binary search (O(log n) for the win)
  int low = 0;
  int high = sizeof(known_hex_codes) / sizeof(known_hex_codes[0]) - 1;

  while (low <= high) {
    int mid = (low + high) / 2;
    if (hex_code == known_hex_codes[mid]) {
      return true;
    } else if (hex_code < known_hex_codes[mid]) {
      high = mid - 1;
    } else {
      low = mid + 1;
    }
  }

  // not found
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
 * This function takes an IRrecv object to check for IR input. On input, the hex
 * code retrieved will be validated. On bad input, it will print to serial the exact
 * signal code that was recieved.
 * 
 * @param IrReceiver the IRrecv object reading infrared signals
 * @return N/A
 */
bool validate_IR(IRrecv IrReceiver) {
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
      } else {
        // IrReceiver.printIRResultShort(&Serial);
        // IrReceiver.printIRSendUsage(&Serial);

        if (!isKnownCode(IrReceiver.decodedIRData.command)) {
          IrReceiver.resume();
          return false;
        } else {
          // process IR signal
          processHexCode(IrReceiver.decodedIRData.command);
        }

        IrReceiver.resume(); // Move this to after processing code to prevent multiple inputs
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
  EEPROM.write(RED_ADDR, red);
  EEPROM.write(GREEN_ADDR, green);
  EEPROM.write(BLUE_ADDR, blue);
  EEPROM.write(JUMP3_ADDR, jump3);
  EEPROM.write(JUMP7_ADDR, jump7);  
  EEPROM.write(PIEZO_THRESH_ADDR, PIEZO_THRESH);
}

/**
 * @brief Pushes RGB color to queue
 * 
 * This function will push the RGB color to the queue.
 * 
 * @param red, green, blue the RGB colors to be pushed to the queue
 * @return N/A
 */
void pushback(cppQueue& q, int red, int green, int blue) {
  // save CRGB value to stack
  CRGB color = CRGB(red, green, blue);

  q.push(&color);
  // // DEBUG print stack size
  // Serial.println("Queue size: " + String(q.getCount()));
  // // print stack values
  // for (int i = 0; i < q.getCount(); i++) {
  //   CRGB color;
  //   q.peekIdx(&color, i);
  //   Serial.println("Queue value: " + String(color.r) + ", " + String(color.g) + ", " + String(color.b));
  // }

  // show contents of queue
  for (int i = 0; i < 3; i ++) {
    // queue size LEDs should flash the color based on the queue
    for (int j = 0; j < q.getCount(); j++) {
      CRGB color;
      q.peekIdx(&color, j);
      led[j] = color;
    }
    FastLED.show();
    delay(200);

    fill_solid(led, NUM_LEDS, CRGB(0, 0, 0));
    FastLED.show();
    delay(200);
  }
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
        // rotate between selected colors
        multicolorQueue.pop(&color);
        RED = color.r;
        GREEN = color.g;
        BLUE = color.b;
        multicolorQueue.push(&color);
      }

      Serial.println("Piezo triggered");

      // Flash LED
      onARGB();
      delay(DELAY_THRESHOLD);
      offARGB();
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
  digitalWrite(LED_RED, RED);
  digitalWrite(LED_GREEN, GREEN);
  digitalWrite(LED_BLUE, BLUE);
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
    fill_solid(led, NUM_LEDS, jump3? rainbowColors[(color_index++) % sizeof(rainbowColors)] : jump7? rainbowColors2[(color_index++) % sizeof(rainbowColors2)] : CRGB(RED, GREEN, BLUE));
    FastLED.show();
  }

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
void toggleOnOff() {
  bool ledon = true;
  // toggle on/off for play/pause button
  onARGB();
  while (ledon) {
    if (IrReceiver.decode()) {

      unsigned long currentMillis = millis();

      if ((currentMillis - lastIRTime) >= IRDebounceDelay) {

        if (IrReceiver.decodedIRData.protocol == UNKNOWN) {
          Serial.println(F("Received noise or an unknown (or not yet enabled) protocol"));
          // We have an unknown protocol here, print extended info
          // DEBUG
          // IrReceiver.printIRResultRawFormatted(&Serial, true);
          // IrReceiver.resume(); // Do it here, to preserve raw data for printing with printIRResultRawFormatted()
        } else {
          IrReceiver.resume(); // Early enable receiving of the next IR frame
          // DEBUG
          // IrReceiver.printIRResultShort(&Serial);
          // IrReceiver.printIRSendUsage(&Serial);

          /// TODO: Where do i reset debounce??
          lastIRTime = currentMillis;
        }
        Serial.println();

        if (IrReceiver.decodedIRData.command == 0x41) {
          ledon = false;
          offARGB();
          offLED();
          break;
        } else {
          // apply modifications to color
          processHexCode(IrReceiver.decodedIRData.command);
        }
        // update color in case of change
        onARGB();
        onLED();
        delay(200);  // delay to reduce multiple inputs
        IrReceiver.resume();
      }
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
      FastLED.setBrightness(constrain(FastLED.getBrightness() +20, 1, 255));
      break;
    // decrease brightness
    case 0x5D:
      FastLED.setBrightness(constrain(FastLED.getBrightness() -20, 1, 255));
      break;
    // play/pause
    case 0x41:
      // reverse lit status
      toggleOnOff();
      break;
    // PWR
    case 0x40:
      // if (!modifier) {
      //   modifier = true;    // trigger alt modifier for next input
      //   led[0] = CRGB(0, 0, MAX_INTENSITY);
      //   FastLED.show();
      //   Serial.println("Modifier ON");
      //   return;
      // } else {
      //   // IrReceiver.disableIRIn();
      //   // Serial.println("IR disabled");
      // }
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
      // if (!modifier) {
        // increase sensitivity
        PIEZO_THRESH -= 10;
        if (PIEZO_THRESH <= 0 || PIEZO_THRESH >= 1023) {  // unsigned int < 0 will become 65535
          PIEZO_THRESH = 10;
          // indicate max sensitivity reached
          flashConfirm(2);
        }
        // Serial.println("Sensitivity: " + String(PIEZO_THRESH));
        // showSensitivity(PIEZO_THRESH);
      // } else {
      //   modifier = false;
      //   // decrease delay (quicker flash)
      //   DELAY_THRESHOLD -= 50;
      //   led[0] = CRGB(0, 0, 0);
      //   FastLED.show();
      // }
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
      // if (!modifier) {
        // decrease sensitivity
        PIEZO_THRESH += 10;
        if (PIEZO_THRESH >= 1023) {
          PIEZO_THRESH = constrain(PIEZO_THRESH, 0, 1023);
          // indicate min sensitivity reached
          flashConfirm(2);
        }
        // Serial.println("Sensitivity: " + String(PIEZO_THRESH));
        // showSensitivity(PIEZO_THRESH);
      // } else {
      //   modifier = false;
      //   // increase delay (slower flash)
      //   DELAY_THRESHOLD += 50;
      //   led[0] = CRGB(0, 0, 0);
      //   FastLED.show();
      // }
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
      // custom multicolor
      pushback(multicolorQueue, RED, GREEN, BLUE);
      break;
    }
    //DIY3
    case 0xE:
      // add to color queue
      pushback(CRGBQueue, RED, GREEN, BLUE);
      // // check current color queue
      // check_colorQueue();
      break;
    // AUTO(save) | IR lock
    case 0xF:
    {
      /// TODO: Save all values, including jump3/7
      eeprom_save(RED, GREEN, BLUE);    // save current color
      flashConfirm(3);                   // flash to confirm save
      break;
    }
    // ==================== row 10 | DIY 4-6, FLASH ====================================

    // DIY4
    case 0x8:
      rainbow_effect();
      break;
    // DIY5
    case 0x9:
      multicolor = !multicolor;
      if (multicolor) {
        check_colorQueue(multicolorQueue);
      }
      break;
    // DIY6
    case 0xA:
      // check current color queue
      check_colorQueue(CRGBQueue);
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
      break;
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
      Serial.println("ERROR: IR recieved unknown value: " + String(IRvalue));
      // flashError(2);
      return -1;
  }

  jump3 = false;
  jump7 = false;
  // fade3 = false;
  // fade7 = false;
  // modifier = false;
  fill_solid(led, NUM_LEDS, CRGB(0, 0, 0));
  FastLED.show();
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
void adj_color(uint8_t& color, int scale) {
  // Adjust color value
  int newColor = color + scale;

  // Constrain new color value to be within 1 and MAX_INTENSITY
  newColor = constrain(newColor, 0, MAX_INTENSITY);

  // Set the adjusted color value
  color = newColor;

  // Debug
  Serial.println("Adjusted color: " + String(color));
  Serial.println("Colors: " + String(RED) + ", " + String(GREEN) + ", " + String(BLUE));
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
        // trails[i].color = jump3 ? rainbowColors[color_index] : jump7? rainbowColors[color_index] : CRGB(RED, GREEN, BLUE);
        trails[i].color = getColor();
        if (jump3) {
          color_index = (color_index + 1) % (sizeof(rainbowColors) / sizeof(rainbowColors[0]));
        }
        if (jump7) {
          color_index = (color_index + 1) % (sizeof(rainbowColors2) / sizeof(rainbowColors2[0]));
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
 * @brief Creates a rainbow effect
 * 
 * This function will create a rainbow effect on the ARGB LED strip.
 * 
 * NOTE: This function will run indefinitely. Device must be powered off to reset.
 * 
 * @return N/A
 */
void rainbow_effect() {
  while (true) {
    for (int j = 0; j < 255; j++) {
      for (int i = 0; i < NUM_LEDS; i++) {
        led[i] = CHSV(i - (j * 2), 255, 255); /* The higher the value 4 the less fade there is and vice versa */ 
      }
      FastLED.show();
      delay(25); /* Change this to your hearts desire, the lower the value the faster your colors move (and vice versa) */
      // if (IrReceiver.decode()) {
      //   // check if hex code is valid
      //   if (isKnownCode(IrReceiver.decodedIRData.command)) {
      //     // processHexCode(IrReceiver.decodedIRData.command);
      //     Serial.println("IR signal recieved: " + String(IrReceiver.decodedIRData.command));
      //     IrReceiver.resume();
      //     return;
      //   } else {
      //     Serial.print("Received Hex: 0x");
      //     Serial.println(results.value, HEX);  // Print in HEX format
      //     IrReceiver.resume();
      //   }
      // }
    }
  }
}

/**
 * @brief Visualizes the color queue
 * 
 * This function will flash the LED strip to show the colors in the queue.
 * 
 * @return N/A
 */
void check_colorQueue(cppQueue& q) {
  // queue size LEDs should flash the color based on the queue
  for (int j = 0; j < q.getCount(); j++) {
    CRGB color;
    q.peekIdx(&color, j);
    led[j] = color;
  }
  FastLED.show();
  delay(1000);

  fill_solid(led, NUM_LEDS, CRGB(0, 0, 0));
  FastLED.show();
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
    // LED strip indicator
    led[0] = CRGB(RED, GREEN, BLUE);
    FastLED.show();

    offLED();
    delay(200);

    // LED strip indicator
    fill_solid(led, NUM_LEDS, CRGB::Black);
    FastLED.show();

    onLED();
    delay(200);
  }
}

/**
  * @brief Sets the first 10 of the led[] array for visual based on the current sensitivity value
  * 
  * This function will show the sensitivity value on the LED strip.
  * 
  * @param val the current sensitivy value based on PIEZO_THRESH
  * @return N/A
  */

void showSensitivity(uint16_t val) {
  // Print current sensitivity value for debugging
  Serial.print("Sensitivity Value: ");
  Serial.println(val);

  // Clear the first 10 LEDs
  fill_solid(led, NUM_LEDS, CRGB(0, 0, 0));

  // Calculate how many LEDs to light fully based on sensitivity
  int numLEDsToLight = map(val, 1023, 0, 0, 10);
  numLEDsToLight = constrain(numLEDsToLight, 0, 10); // Ensure it stays within bounds

  // Print the number of LEDs to light for debugging
  Serial.print("Number of LEDs to Light: ");
  Serial.println(numLEDsToLight);

  // Determine brightness levels for fully lit LEDs
  for (int i = 0; i < numLEDsToLight; i++) {
    led[i] = CRGB(255, 0, 0); // Set fully lit LEDs to RED
  }

  // If the sensitivity value falls between two LEDs, fade the final lit LED
  if (numLEDsToLight < 10) {
    // Calculate brightness for the last partially lit LED
    int ledBrightness = map(val, (numLEDsToLight * 102), ((numLEDsToLight + 1) * 102), 255, 0);
    led[numLEDsToLight] = CRGB(ledBrightness, 0, 0); // Set the next LED with dimmed brightness
  }

  // Update the LED strip to reflect the changes
  FastLED.show();
  delay(300);
}

// /**
//  * @brief Flashes the LED strip to indicate an error
//  * 
//  * This function will flash the LED strip to indicate an error based on the error code.
//  * 
//  * @param errorcode the error code to flash the LED strip (number of flashes)
//  * @return N/A
//  */
// void flashError(int errorcode) {
//   /*
//   * Flash error codes based on specific error.
//   * 1: Invalid IR remote value recieved
//   * 2: Unknown protocol from IR
//   */
//   /// TODO: Modify. Will react with any IR signals (e.g. iPhone Face ID and any other source of IR)
//   for (int i = 0; i < errorcode; i++) {
//     led[0] = CRGB(MAX_INTENSITY, 0, 0);
//     FastLED.show();
//     delay(50);
//     led[0] = CRGB(0, 0, 0);
//     FastLED.show();
//   }
// }
