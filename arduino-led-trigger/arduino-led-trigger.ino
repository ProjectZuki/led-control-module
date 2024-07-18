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
#define IR_RECEIVE_PIN 2

// RGB LED pins
#define RED_PIN       9
#define GREEN_PIN     10
#define BLUE_PIN      11

// ARGB pin
#define NUM_LEDS      50
#define LED_PIN       6
#define MAX_INTENSITY 32    // 255 / 128 / 64 / 32 / 16 / 8 / 4 / 2 / 1

CRGB led[NUM_LEDS];

// piezo pin
#define PIEZO_PIN     A0
#define PIEZO_THRESH  500

// default RED to 255
/// TODO: Global -> local variables to save on dynamic memory
int RED = 0;
int GREEN = 0;
int BLUE = MAX_INTENSITY;
 
// stay lit when activated
bool ledon = false;

// IR
IRrecv irrecv(IR_RECEIVE_PIN);
decode_results results;

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);

  // Set RGB LED pins as outputs
  pinMode(RED_PIN, OUTPUT);
  pinMode(GREEN_PIN, OUTPUT);
  pinMode(BLUE_PIN, OUTPUT);

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
  IrReceiver.begin(IR_RECEIVE_PIN, ENABLE_LED_FEEDBACK);
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

        /*
         * process codes
         */
        switch(IrReceiver.decodedIRData.command) {
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
            Serial.println("We are here!");
            Serial.println("LED statuus: " + String(ledon));
            break;
          case 0x40:
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
            adj_color(RED, 20);
          case 0x15:
            adj_color(GREEN, 20);
          case 0x16:
            adj_color(BLUE, 20);
            break;
          // QUICK
          case 0x17:
            break;

          // ==================== row 8 - RED/BLUE/GREEN decrease, SLOW ====================

          case 0x10:
            adj_color(RED, -20);
          case 0x11:
            adj_color(GREEN, -20);
          case 0x12:
            adj_color(BLUE, -20);
            break;
          // SLOW
          case 0x13:
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
            Serial.println("ERROR: IR recieved unknown value: " + String(IrReceiver.decodedIRData.command));
            flashError(1);
        }

        flashConfirm();
  }
  /// TODO: continousARGB(); function call causes unknown behavior with IR reader

  if ((analogRead(PIEZO_PIN) > PIEZO_THRESH) || ledon) {
    continuousARGB();
    if (analogRead(PIEZO_PIN) > PIEZO_THRESH) {
      delay(100);
      offARGB();
    }
  }
  
  if (!ledon) {
    offARGB();
  }

  // if (analogRead(PIEZO_PIN) > PIEZO_THRESH) {    // piezo reads analog
  //   // flash LED
  //   continuousARGB();
  // }
}

void flashRGB() {
  // do the thing
  for (int i = 0; i < NUM_LEDS; i++){
    led[i] = CRGB(RED, GREEN, BLUE);
  }

  FastLED.show();

  // for single LED
  // analogWrite(RED_PIN, RED);
  // analogWrite(GREEN_PIN, GREEN);
  // analogWrite(BLUE_PIN, BLUE);

  delay(100);

  // for single LED
  // analogWrite(RED_PIN, 0);
  // analogWrite(GREEN_PIN, 0);
  // analogWrite(BLUE_PIN, 0);

  offARGB();
}

void continuousARGB() {
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

    Serial.println("Scale Factor: " + String(scaleFactor));
  }

  Serial.println("RGB values - RED: " + String(RED) + " GREEN: " + String(GREEN) + " BLUE: " + String(BLUE));
}

void adj_color(int& color, int value) {
  if (color+value > 0 && color+value < MAX_INTENSITY) {
    color += value;
  }
}

void adj_brightness(int& red, int& green, int& blue, int value) {
  /// TODO:

  // Calculate the current maximum value among the RGB components
  int maxComponent = max(red, max(green, blue));
  int minComponent = min(red, min(green, blue));

  Serial.println("Current RGB values - RED: " + String(RED) + " GREEN: " + String(GREEN) + " BLUE: " + String(BLUE));

  // If maxComponent is 0, we can't scale, so return early
  if (maxComponent == 0) return;

  // Calculate the new brightness level
  int newMaxComponent = maxComponent + value;

  // Ensure newMaxComponent is within the range of 0 to 255
  newMaxComponent = constrain(newMaxComponent, 0, MAX_INTENSITY);

  // Calculate the scaling factor
  float scaleFactor = (float)newMaxComponent / maxComponent;

  // Scale the RGB values
  red = constrain(red * scaleFactor, 0, MAX_INTENSITY);
  green = constrain(green * scaleFactor, 0, MAX_INTENSITY);
  blue = constrain(blue * scaleFactor, 0, MAX_INTENSITY);

  Serial.println("New RGB values - RED: " + String(RED) + " GREEN: " + String(GREEN) + " BLUE: " + String(BLUE));
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
    analogWrite(RED_PIN, MAX_INTENSITY);
    analogWrite(GREEN_PIN, 0);
    analogWrite(BLUE_PIN, 0);
    delay(100);
    analogWrite(RED_PIN, 0);
    analogWrite(GREEN_PIN, 0);
    analogWrite(BLUE_PIN, 0);
    delay(100);
  }
}