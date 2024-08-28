
/******************************************************************************
 * @file       hc-12-tx.ino
 * @brief      Arduino RF communication using HC-12 module as sender
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

#include <SoftwareSerial.h>
#include <IRremote.h>
#include <FastLED.h>

#define IR_RECEIVER_PIN 18
#define MAX_INTENSITY 32    // 255 / 128 / 64 / 32 / 16 / 8

SoftwareSerial HC12(2, 3);  // HC-12 TX Pin, HC-12 RX Pin
#define START_MARKER 0x7E
#define END_MARKER 0x7F

// IR
IRrecv irrecv(IR_RECEIVER_PIN);
decode_results results;

// All known IR hex codes
const uint32_t known_hex_codes[] = {
  // ==================== row 1 - Brightness UP/DOWN, play/pause, power ========
  0x5C, 0x5D, 0x41, 0x40,
  // ==================== row 2 | Color ========================================
  0x58, 0x59, 0x45, 0x44,
  // ==================== row 3 | Color ========================================
  0x54, 0x55, 0x49, 0x48,
  // ==================== row 4 | Color ========================================
  0x50, 0x51, 0x4D, 0x4C,
  // ==================== row 5 | Color ========================================
  0x1C, 0x1D, 0x1E, 0x1F,
  // ==================== row 6 | Color ========================================
  0x18, 0x19, 0x1A, 0x1B,
  // ==================== row 7 | RED/BLUE/GREEN increase, QUICK ===============
  0x14, 0x15, 0x16, 0x17,
  // ==================== row 8 | RED/BLUE/GREEN decrease, SLOW ================
  0x10, 0x11, 0x12, 0x13,
  // ==================== row 9 | DIY 1-3, AUTO ================================
  0xC, 0xD, 0xE, 0xF,
  // ==================== row 10 | DIY 4-6, FLASH ==============================
  0x8, 0x9, 0xA, 0xB,
  // ==================== row 11 | Jump3, Jump7, FADE3, FADE7 ==================
  0x4, 0x5, 0x6, 0x7
};

int hexValue = 0;

unsigned long previousMillis = 0;
const unsigned long debounceDelay = 250;

struct dataPacket {
  uint8_t red;
  uint8_t green;
  uint8_t blue;
  bool ledon;
  bool rainboweffect;
  bool jump3;
  bool jump7;
  uint8_t checksum;
};

dataPacket packet;

void setup() {
    pinMode(LED_BUILTIN, OUTPUT);
    Serial.begin(9600);  // Serial monitor for debugging
    // IR
    IrReceiver.begin(IR_RECEIVER_PIN, ENABLE_LED_FEEDBACK);
    HC12.begin(9600);      // Initialize HC-12 serial communication
}

void loop() {
    // check for IR signal
    if (validate_IR(IrReceiver)) {
      // transmit data
      transmit_data();
    }

    /// TODO Add button to re-send signal
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
      return;
    } else {
        IrReceiver.resume(); // Early enable receiving of the next IR frame
        IrReceiver.printIRResultShort(&Serial);
        IrReceiver.printIRSendUsage(&Serial);
      
        // process IR signal
        if (processHexCode(IrReceiver.decodedIRData.command) == -1) {
          Serial.println("ERROR: IR recieved unknown value: " + String(IrReceiver.decodedIRData.command));
          return false;
        } else {
          // trasnmit data on valid IR signal code, repeats allowed
          // transmit_data();
          return true;
        }

        previousMillis = millis();
      }
    }
    if (millis() - previousMillis >= debounceDelay) {
      // reset IR signal
      IrReceiver.resume();
    }
    return false;
  }

/**
 * @brief Transmit data for red, green, blue
 * 
 * This function will transmit the data for RED, GREEN, BLUE to the HC-12 module.
 * 
 * @return N/A
 */
void transmit_data() {
  packet.checksum = calculateChecksum(packet);

  // Debug print for packet
  Serial.print("Transmitting data... ");
  Serial.print("Red: "); Serial.print(packet.red);
  Serial.print(", Green: "); Serial.print(packet.green);
  Serial.print(", Blue: "); Serial.println(packet.blue);
  Serial.print(", LED: "); Serial.println(packet.ledon);
  Serial.print(", RainbowEffect: "); Serial.println(packet.rainboweffect);
  Serial.print(", Jump3: "); Serial.println(packet.jump3);
  Serial.print(", Jump7: "); Serial.println(packet.jump7);
  Serial.print(", Checksum: "); Serial.println(packet.checksum);

  // // Debug print for raw bytes
  // byte* ptr = (byte*)&packet;
  // Serial.print("Raw bytes: ");
  // for (int i = 0; i < sizeof(packet); i++) {
  //   Serial.print(ptr[i], HEX);
  //   Serial.print(" ");
  // }
  // Serial.println();

  // Transmit data with start and end markers
  for (int i = 0; i < 3; i++) {
    HC12.write(START_MARKER);
    HC12.write((byte*)&packet, sizeof(packet));
    HC12.write(END_MARKER);
    delay(100);
  }
}

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
      // FastLED.setBrightness(constrain(FastLED.getBrightness() +20, 1, 255));
      break;
    // decrease brightness
    case 0x5D:
      // FastLED.setBrightness(constrain(FastLED.getBrightness() -20, 1, 255));
      break;
    // play/pause
    case 0x41:
      // reverse lit status
      // packet.ledon =  !packet.ledon;
      setParam(packet.ledon);
      return;
    // PWR
    case 0x40:
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
      adj_color(packet.red, MAX_INTENSITY/10);
      break;
    case 0x15:
      adj_color(packet.green, MAX_INTENSITY/10);
      break;
    case 0x16:
      adj_color(packet.blue, MAX_INTENSITY/10);
      break;
    // QUICK | Sensitivity down
    case 0x17:
    {
    //   if (!modifier) {
    //     // increase sensitivity
    //     PIEZO_THRESH -= 50;
    //     if (PIEZO_THRESH <= 0 || PIEZO_THRESH >= 1023) {  // unsigned int < 0 will become 65535
    //       PIEZO_THRESH = 10;
    //     }
    //   } else {
    //     modifier = false;
    //     // decrease delay (quicker flash)
    //     DELAY_THRESHOLD -= 50;
    //     led[0] = CRGB(0, 0, 0);
    //     FastLED.show();
    //   }
      break;
    }
    // ==================== row 8 | RED/BLUE/GREEN decrease, SLOW ====================

    case 0x10:
      adj_color(packet.red, MAX_INTENSITY/-10);
      break;
    case 0x11:
      adj_color(packet.green, MAX_INTENSITY/-10);
      break;
    case 0x12:
      adj_color(packet.blue, MAX_INTENSITY/-10);
      break;
    // SLOW | Sensitivity up
    case 0x13:
    {
    //   if (!modifier) {
    //     // decrease sensitivity
    //     PIEZO_THRESH += 50;
    //     if (PIEZO_THRESH >= 1023) {
    //       PIEZO_THRESH = constrain(PIEZO_THRESH, 0, 1023);
    //     }
    //   } else {
    //     modifier = false;
    //     // increase delay (slower flash)
    //     DELAY_THRESHOLD += 50;
    //     led[0] = CRGB(0, 0, 0);
    //     FastLED.show();
    //   }
      break;
    }
    // ==================== row 9 | DIY 1-3, AUTO ====================================


    // DIY1
    case 0xC:
    {
    //   // loop until IR signal is received
    //   while (true) {
    //     // continue checking for valid IR signal
    //     if (IrReceiver.decode()) {
    //       if (processHexCode(IrReceiver.decodedIRData.command) != -1) {
    //         break;
    //       }
    //       IrReceiver.resume();    // resume IR input
    //     }
    //     ripple();
    //   }
      break;
    }
    // DIY2
    case 0xD:
    {
    //   while (true) {
    //     // continue checking for valid IR signal
    //     if (IrReceiver.decode()) {
    //       if (processHexCode(IrReceiver.decodedIRData.command) != -1) {
    //         break;
    //       }
    //       IrReceiver.resume();    // resume IR input
    //     }
    //     ripple2();
    //   }
      break;
    }
    //DIY3
    case 0xE:
      // check current color queue
    //   check_colorQueue();
      break;
    // AUTO(save) | IR lock
    case 0xF:
      break;
    // ==================== row 10 | DIY 4-6, FLASH ====================================

    // DIY4
    case 0x8:
    //   rainbow_effect();
      // break;
      // packet.rainboweffect = !packet.rainboweffect;
      setParam(packet.rainboweffect);
      return;
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
      setParam(packet.jump3);
      return;   // return early to prevent color change
    // JUMP7
    case 0x5:
      // other rainbow effect
      setParam(packet.jump7);
      return;
    // FADE3
    case 0x6:
      break;
    // FADE7
    case 0x7:
      break;
    
    // Default print error for debug
    default:
      Serial.println("ERROR: IR recieved unknown value: " + String(IRvalue));
    //   flashError(2);
      return -1;
  }

  // reset modifiers
  // packet.ledon = false;
  packet.rainboweffect = false;
  packet.jump3 = false;
  packet.jump7 = false;
//   fill_solid(led, NUM_LEDS, CRGB(0, 0, 0));
//   FastLED.show();
  return IRvalue;
}

void setParam(bool& param) {
  bool temp = param;

  packet.ledon = false;
  packet.rainboweffect = false;
  packet.jump3 = false;
  packet.jump7 = false;

  param = !temp;
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
  packet.red = scale8(color.r, MAX_INTENSITY);
  packet.green = scale8(color.g, MAX_INTENSITY);
  packet.blue = scale8(color.b, MAX_INTENSITY);
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
  newColor = constrain(newColor, 1, MAX_INTENSITY);

  // Set the adjusted color value
  color = newColor;
}