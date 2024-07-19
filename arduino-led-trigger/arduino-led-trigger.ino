/**
 * Copyright (C) [Project]Zuki - All Rights Reserved
 * 
 * This source code is protected under international copyright law.  All rights
 * reserved and protected by the copyright holders. For permisssion to use this
 * source code, please contact the copyright holders.
 */

#include <IRremote.h>
#include <FastLED.h>

// IR receiver pin
#define IR_RECEIVER_PIN 2

// ARGB pin
#define NUM_LEDS      50
#define LED_PIN       6
#define MAX_INTENSITY 16    // 255 / 128 / 64 / 32 / 16 / 8 / 4 / 2 / 1
CRGB led[NUM_LEDS];

// piezo pin
#define PIEZO_PIN     A0
unsigned int PIEZO_THRESH = 500;

// default RED to 255
/// TODO: Global -> local variables to save on dynamic memory
unsigned int RED = 0;
unsigned int GREEN = 0;
unsigned int BLUE = MAX_INTENSITY;
 
// stay lit when activated
bool ledon = false;

// IR
IRrecv irrecv(IR_RECEIVER_PIN);
decode_results results;

// modifier tied to PWR button
bool modifier = false;

// delay threshold for flash duration
int delayThreshold = 100;

void setup() {
  // built-in LED
  pinMode(LED_BUILTIN, OUTPUT);

  // ARGB
  FastLED.addLeds<NEOPIXEL, LED_PIN>(led, NUM_LEDS);
  // for (int i = 0; i < NUM_LEDS; i++){
  //   led[i] = CRGB(0, 0, MAX_INTENSITY);
  // }
  FastLED.show();

  // piezo
  pinMode(PIEZO_PIN, INPUT);
  // debug
  Serial.begin(9600);

  // IR
  // Start the receiver, set default feedback LED
  IrReceiver.begin(IR_RECEIVER_PIN, ENABLE_LED_FEEDBACK);
}

void loop() {
  // turn on built in LED to confirm functionality
  // digitalWrite(LED_BUILTIN, HIGH);

  // // piezo threshold debug
  // Serial.prinln(analogRead(A0));
  // delay(2);
  //

  // IR remote instructions
  if (IrReceiver.decode()) {
    delay(1000); // delay to prevent multiple inputs
    handleIRInput(IrReceiver.decodedIRData.command);
    // IR remote instructions
    int IRval = processHexCode(IrReceiver.decodedIRData.command);
    if (IRval == -1) {
      Serial.println("ERROR: IR recieved unknown value: " + String(IrReceiver.decodedIRData.command));
      flashError(1);
    }
  }

  // this works fine
  if (analogRead(PIEZO_PIN) > PIEZO_THRESH) {    // piezo reads analog
    // flash LED
    onARGB();
    delay(delayThreshold);
    offARGB();
  }
}

void handleIRInput(int command) {
  /*
  * Print a summary of received data
  */

  if (IrReceiver.decodedIRData.protocol == UNKNOWN) {
    Serial.println(F("Received noise or an unknown (or not yet enabled) protocol"));
    // We have an unknown protocol here, print extended info
    IrReceiver.printIRResultRawFormatted(&Serial, true);
    IrReceiver.resume(); // Do it here, to preserve raw data for printing with printIRResultRawFormatted()
    flashError(2);
  } else {
    IrReceiver.resume(); // Early enable receiving of the next IR frame
    IrReceiver.printIRResultShort(&Serial);
    // IR reciever debug
    // IrReceiver.printIRSendUsage(&Serial);
    //
  }
  Serial.println();
}

void onARGB() {
  // do the thing but ARGB
  for (int i = 0; i < NUM_LEDS; i++){
    led[i] = CRGB(RED, GREEN, BLUE);
  }
  FastLED.show();
}

void offARGB() {
  // do the off thing
  for (int i = 0; i < NUM_LEDS; i++){
    led[i] = CRGB(0, 0, 0);
  }

  FastLED.show();
}

void toggleOnOff() {
  // toggle on/off for play/pause button
  onARGB();
  while (ledon) {
    if (IrReceiver.decode()) {
      handleIRInput(IrReceiver.decodedIRData.command);
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
      IrReceiver.resume();
    }
  }
}

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

      // ==================== row 2 - Color ==========================================
      case 0x58:
        // red
        hexToRGB("#FF0000");
        break;
      case 0x59:
        // green
        hexToRGB("#00FF00");
        break;
      case 0x45:
        // blue
        hexToRGB("#0000FF");
        break;
      case 0x44:
        // white
        hexToRGB("#FFFFFF");
        break;

      // ==================== row 3 - Color ==========================================
      case 0x54:
        // static orange
        hexToRGB("#FF8000");
        break;
      case 0x55:
        // pea green
        hexToRGB("#80FF00");
        break;
      case 0x49:
        // static dark blue
        hexToRGB("#0080FF");
        break;

      case 0x48:
        // static pink
        hexToRGB("#FF80FF");
        break;

      // ==================== row 4 - Color ==========================================
      case 0x50:
        // static dark yellow
        hexToRGB("#FFD700");
        break;
      case 0x51:
        // static cyan
        hexToRGB("#00FFFF");
        break;
      case 0x4D:
        // static royal blue
        hexToRGB("#4169E1");
        break;
      case 0x4C:
        // static light pink
        hexToRGB("#FFB6C1");
        break;

      // ==================== row 5 - Color ==========================================
      case 0x1C:
        // static yellow
        hexToRGB("#FFFF00");
        break;
      case 0x1D:
        // static light blue
        hexToRGB("#ADD8E6");
        break;
      case 0x1E:
        // static light brown
        hexToRGB("#D2B48C");
        break;
        // static green white
      case 0x1F:
        hexToRGB("#F0FFF0");
        break;

      // ==================== row 6 - Color ==========================================
      case 0x18:
        // static light yellow
        hexToRGB("#FFFFE0");
        break;
      case 0x19:
        // static sky blue
        hexToRGB("#87CEEB");
        break;
      case 0x1A:
        // static violet
        hexToRGB("#EE82EE");
        break;
      case 0x1B:
        // static blue white
        hexToRGB("#F0F8FF");
        break;

      // ==================== row 7 - RED/BLUE/GREEN increase, QUICK ===================

      case 0x14:
        adj_color(RED, 1.1);
        break;
      case 0x15:
        adj_color(GREEN, 1.1);
        break;
      case 0x16:
        adj_color(BLUE, 1.1);
        break;
      // QUICK
      case 0x17:
        if (!modifier) {
          // increase sensitivity
          PIEZO_THRESH -= 50;
          if (PIEZO_THRESH <= 0 || PIEZO_THRESH >= 1023) {  // unsigned int < 0 will become 65535
            PIEZO_THRESH = 10;
          }
          Serial.println("PIEZO_THRESH: " + String(PIEZO_THRESH));
        } else {
          modifier = false;
          // decrease delay (quicker flash)
          delayThreshold -= 50;
          led[0] = CRGB(0, 0, 0);
          FastLED.show();
          Serial.println("delayThreshold: " + String(delayThreshold));
        }
        break;

      // ==================== row 8 - RED/BLUE/GREEN decrease, SLOW ====================

      case 0x10:
        adj_color(RED, 0.9);
        break;
      case 0x11:
        adj_color(GREEN, 0.9);
        break;
      case 0x12:
        adj_color(BLUE, 0.9);
        break;
      // SLOW
      case 0x13:
        if (!modifier) {
          // decrease sensitivity
          PIEZO_THRESH += 50;
          if (PIEZO_THRESH >= 1023) {
            PIEZO_THRESH = constrain(PIEZO_THRESH, 0, 1023);
          }
          Serial.println("PIEZO_THRESH: " + String(PIEZO_THRESH));
        } else {
          modifier = false;
          // increase delay (slower flash)
          delayThreshold += 50;
          led[0] = CRGB(0, 0, 0);
          FastLED.show();
          Serial.println("delayThreshold: " + String(delayThreshold));
        }
        break;

      // ==================== row 9 - DIY 1-3, AUTO ====================================

      // DIY1
      case 0xC:
        break;
      // DIY2
      case 0xD:
        break;
      //DIY3
      case 0xE:
        break;
      // AUTO / SAVE
      case 0xF:
        break;

      // ==================== row 10 DIY 4-6, FLASH ====================================

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
        break;

      // ==================== row 11 Jump3, Jump7, FADE3, FADE7 ========================

      // JUMP3
      case 0x4:
        break;
      // JUMP7
      case 0x5:
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
  return IRvalue;
}

void hexToRGB(String hexCode) {
  long num = strtol(hexCode.c_str()+1, nullptr, 16);  //+1 to ignore # hex symbol
  RED = (num >> 16) & 0xFF;
  GREEN = num >> 8 & 0xFF;
  BLUE = num & 0xFF;

  // find max component value
  int maxComp = max(RED, max(GREEN, BLUE));
  // Scale values for max intensity
  if (maxComp > MAX_INTENSITY) {
    float scaleFactor = (float)MAX_INTENSITY / maxComp;

    // set newly scaled values
    RED = round(RED * scaleFactor);
    GREEN = round(GREEN * scaleFactor);
    BLUE = round(BLUE * scaleFactor);
  }
}

void adj_color(unsigned int& color, float scale) {
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

void adj_brightness(unsigned int& red, unsigned int& green, unsigned int& blue, int value) {
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


void rainbow() {
  if (RED == MAX_INTENSITY) {
      RED = 0;
      GREEN = MAX_INTENSITY;
      BLUE = 0;
    } else if (GREEN == MAX_INTENSITY) {
      RED = 0;
      BLUE = MAX_INTENSITY;
      GREEN = 0;
    } else if (BLUE == MAX_INTENSITY) {
      RED = MAX_INTENSITY;
      GREEN = 0;
      BLUE = 0;
    }
}

void flashConfirm() {
  digitalWrite(LED_BUILTIN, LOW);
  delay(200);
  digitalWrite(LED_BUILTIN, HIGH);
}

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
