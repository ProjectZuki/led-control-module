
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

#include <IRremote.h>         // IR remote
#include <FastLED.h>          // NeoPixel ARGB
#include <EEPROM.h>           // save ROM data durong off state
#include <cppQueue.h>         // queue for RGB color states
#include <SoftwareSerial.h>   // HC-12 module

// IR receiver pin
#define IR_RECEIVER_PIN 18

// ARGB pin
#define serialnm      [112 114 111 106 101 99 116 122 117 107 105]
#define NUM_LEDS      144
#define LED_PIN       10
#define MAX_INTENSITY 32    // 255 / 128 / 64 / 32 / 16 / 8
CRGB led[NUM_LEDS];

#define LED_RED       5
#define LED_GREEN     6
#define LED_BLUE      9

#define BUTTON_PIN    21

// EEPROM addresses
#define RED_ADDR      0
#define GREEN_ADDR    1
#define BLUE_ADDR     2
#define RAINBOW_ADDR  3

// HC-12 module
SoftwareSerial HC12(2, 3);  // HC-12 TX Pin, HC-12 RX Pin
#define START_MARKER 0x7E
#define END_MARKER 0x7F

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
 
// // always on mode
// bool ledon = false;

// IR
IRrecv irrecv(IR_RECEIVER_PIN);
decode_results results;

// modifier tied to PWR button
bool modifier = false;
// multi effect
bool multicolor = false;

// always on
bool ledonrx = false;
bool rainbowrx = false;

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

struct dataPacket {
  uint8_t red;
  uint8_t green;
  uint8_t blue;
  bool ledon;
  bool rainbow;
  uint8_t checksum;
};

dataPacket packet;

// All known IR hex codes
const uint32_t known_hex_codes[] = {
  // ==================== row 1 - Brightness UP/DOWN, play/pause, power ==========
  0x5C, 0x5D, 0x41, 0x40,
  // ==================== row 2 | Color ==========================================
  0x58, 0x59, 0x45, 0x44,
  // ==================== row 3 | Color ==========================================
  0x54, 0x55, 0x49, 0x48,
  // ==================== row 4 | Color ==========================================
  0x50, 0x51, 0x4D, 0x4C,
  // ==================== row 5 | Color ==========================================
  0x1C, 0x1D, 0x1E, 0x1F,
  // ==================== row 6 | Color ==========================================
  0x18, 0x19, 0x1A, 0x1B,
  // ==================== row 7 | RED/BLUE/GREEN increase, QUICK ===================
  0x14, 0x15, 0x16, 0x17,
  // ==================== row 8 | RED/BLUE/GREEN decrease, SLOW ====================
  0x10, 0x11, 0x12, 0x13,
  // ==================== row 9 | DIY 1-3, AUTO ====================================
  0xC, 0xD, 0xE, 0xF,
  // ==================== row 10 | DIY 4-6, FLASH ====================================
  0x8, 0x9, 0xA, 0xB,
  // ==================== row 11 | Jump3, Jump7, FADE3, FADE7 ========================
  0x4, 0x5, 0x6, 0x7
};

void setup() {
  // built-in LED
  pinMode(LED_BUILTIN, OUTPUT);

  // button
  pinMode(BUTTON_PIN, INPUT);

  // ARGB
  FastLED.addLeds<NEOPIXEL, LED_PIN>(led, NUM_LEDS);
  FastLED.setBrightness(MAX_INTENSITY);
  FastLED.show();

  // RGB LED
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_BLUE, OUTPUT);

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

  // HC-12 module communication
  HC12.begin(9600);

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

  // check_rx();

  check_button();

  /// NOTE: uncomment for actual implementation
  // always-on LED will be 25% brightness of the current color
  // onLED();

  // check for either IR or transmitter data
  if (!validate_IR(IrReceiver)) {
    check_rx();
  }

  if (ledonrx) {
    rainbowrx = false;
    toggleOnOff();
  } else if (rainbowrx) {
    ledonrx = false;
    rainbow_effect();
  }

  piezo_trigger();
}

/**
 * @brief Receives transmitter data if available
 * 
 * This function will check transmitter data values red, green, blue when available.
 * 
 * @return N/A
 */
uint8_t calculateChecksum(const dataPacket& packet) {
  uint8_t checksum = 0;
  const uint8_t* ptr = (const uint8_t*)&packet;

  // Calculate checksum for the packet excluding the checksum field itself
  for (size_t i = 0; i < sizeof(packet) - sizeof(packet.checksum); ++i) {
    checksum += ptr[i];
  }
  return checksum;
}

/**
 * @brief Receives transmitter data when available
 * 
 * This function will check transmitter data values red, green, blue, ledon, and rainboweffect when available.
 * 
 * @return N/A
 */
bool check_rx() {
  static bool receiving = false;
  static byte buffer[sizeof(dataPacket)];
  static uint8_t bufferIndex = 0;

  while (HC12.available() > 0) {
    byte receivedByte = HC12.read();
    
    if (receivedByte == START_MARKER) {
      receiving = true;
      bufferIndex = 0;
    } else if (receivedByte == END_MARKER) {
      if (receiving && bufferIndex == sizeof(dataPacket)) {
        dataPacket packet;
        memcpy(&packet, buffer, sizeof(dataPacket));

        // Calculate the checksum of the received packet
        uint8_t calculatedChecksum = calculateChecksum(packet);

        // // Debug prints for received packet and calculated checksum
        // Serial.print("Received packet: ");
        // Serial.print(packet.red);
        // Serial.print(", ");
        // Serial.print(packet.green);
        // Serial.print(", ");
        // Serial.print(packet.blue);
        // Serial.print(", LED: ");
        // Serial.print(packet.ledon);
        // Serial.print(", Rainbow: ");
        // Serial.println(packet.rainbow);
        // Serial.print("Received checksum: ");
        // Serial.println(packet.checksum);
        // Serial.print("Calculated checksum: ");
        // Serial.println(calculatedChecksum);

        // Validate integrity
        if (packet.checksum == calculatedChecksum) {
          // Update your variables here
          RED = packet.red;
          GREEN = packet.green;
          BLUE = packet.blue;
          ledonrx = packet.ledon;
          rainbowrx = packet.rainbow;
          return true;
        } else {
          Serial.println("ERROR: checksum mismatch, possible data corruption");
        }
      }
      receiving = false; // Reset receiving state after end marker
    } else if (receiving) {
      if (bufferIndex < sizeof(dataPacket)) {
        buffer[bufferIndex++] = receivedByte;
      } else {
        // Buffer overflow, reset receiving
        receiving = false;
        bufferIndex = 0;
        Serial.println("ERROR: Buffer overflow, resetting receiving state.");
        return;
      }
    }
  }
  return false;
}

/**
 * @brief Calculates the checksum for the data packet
 * 
 * This function will calculate the checksum for the data packet.
 * 
 * @param packet the data packet to calculate the checksum
 * @return the checksum value
 */
uint8_t calculateChecksum(dataPacket& packet) {
  return packet.red + packet.green + packet.blue + packet.ledon + packet.rainbow;
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
  if (digitalRead(BUTTON_PIN) == HIGH) {
    CRGB color;
    CRGBQueue.pop(&color);
    RED = color.r;
    GREEN = color.g;
    BLUE = color.b;

    // Serial.println("Color from queue: " + String(RED) + ", " + String(GREEN) + ", " + String(BLUE));
    delay(200);  // delay to prevent multiple pops
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
bool check_hex_code(uint32_t hex_code) {
  for (int i = 0; i < sizeof(known_hex_codes) / sizeof(known_hex_codes[0]); i++) {
    if (hex_code == known_hex_codes[i]) {
      return true;
    }
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
    if (IrReceiver.decodedIRData.protocol == UNKNOWN) {
      Serial.println(F("Received noise or an unknown (or not yet enabled) protocol"));
      // We have an unknown protocol here, print extended info
      IrReceiver.printIRResultRawFormatted(&Serial, true);
      IrReceiver.resume(); // Do it here, to preserve raw data for printing with printIRResultRawFormatted()
      return false;
    } else {
      IrReceiver.resume(); // Early enable receiving of the next IR frame
      IrReceiver.printIRResultShort(&Serial);
      IrReceiver.printIRSendUsage(&Serial);
      
      if (processHexCode(IrReceiver.decodedIRData.command) == -1) {
        Serial.println("ERROR: IR recieved unknown value: " + String(IrReceiver.decodedIRData.command));
        flashError(1);
        return false;
      }
      return true;
    }
    Serial.println();


    delay(500); // delay to prevent multiple inputs
  } else {
    return false;
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
  bool ledon = true;
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
    } else if (check_rx()) {
      ledon = ledonrx;
      if (!ledon) {
        // reset
        offARGB();
        offLED();
      } else {
        // apply modifications to color
        onARGB();
        onLED();
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
      if (!modifier) {
        modifier = true;    // trigger alt modifier for next input
        led[0] = CRGB(0, 0, MAX_INTENSITY);
        FastLED.show();
        Serial.println("Modifier ON");
        return;
      } else {
        IrReceiver.disableIRIn();
        Serial.println("IR disabled");
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
      // while (true) {
      //   // continue checking for valid IR signal
      //   if (IrReceiver.decode()) {
      //     if (processHexCode(IrReceiver.decodedIRData.command) != -1) {
      //       break;
      //     }
      //     IrReceiver.resume();    // resume IR input
      //   }
      //   ripple2();
      // }

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
      eeprom_save(RED, GREEN, BLUE);    // save current color
      EEPROM.write(RAINBOW_ADDR, rainbow);
      flashConfirm();                   // flash to confirm save
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
  modifier = false;
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

  // // Debug
  // Serial.println("Adjusted color: " + String(color));
  // Serial.println("Colors: " + String(RED) + ", " + String(GREEN) + ", " + String(BLUE));
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
      
      // check for RF signal
      if (check_rx()) {
        if (!ledonrx) {
          // reset
          offARGB();
          return;
        }
      }
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
  delay(2000);

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
void flashConfirm() {
  for (int i = 0; i < 3; i ++) {
    led[0] = CRGB(RED, GREEN, BLUE);
    FastLED.show();

    onLED();

    delay(200);
    // led[0] = CRGB(0, 0, 0);
    fill_solid(led, NUM_LEDS, CRGB(0, 0, 0));
    FastLED.show();

    offLED();

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
