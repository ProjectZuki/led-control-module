#include <IRremote.h>

// IR receiver pin
#define IR_RECEIVE_PIN 2

// RGB LED pins
#define RED_PIN       9
#define GREEN_PIN     10
#define BLUE_PIN      11

// piezo pin
#define PIEZO_PIN     A0

#define MAX_INTENSITY 255
#define PIEZO_THRESH  500

// default RED to 255
int RED = MAX_INTENSITY;
int GREEN = 0;
int BLUE = 0;

// IR
IRrecv irrecv(IR_RECEIVE_PIN);
decode_results results;

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);

  // Set RGB LED pins as outputs
  pinMode(RED_PIN, OUTPUT);
  pinMode(GREEN_PIN, OUTPUT);
  pinMode(BLUE_PIN, OUTPUT);

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
  digitalWrite(LED_BUILTIN, HIGH);

  // // piezo threshold debug
  // Serial.prinln(analogRead(A0));
  // delay(2);

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
        } else {
            IrReceiver.resume(); // Early enable receiving of the next IR frame
            IrReceiver.printIRResultShort(&Serial);
            // IrReceiver.printIRSendUsage(&Serial);
        }
        Serial.println();

        /*
         * process codes
         */
        // if (IrReceiver.decodedIRData.command == 0x58) {
        //     // do something
        // } else if (IrReceiver.decodedIRData.command == 0x59) {
        //     // do something else
        // }

        switch(IrReceiver.decodedIRData.command) {
          case 0x58:
            // red
            hexToRGB("FF0000");
            break;
          case 0x59:
            // green
            hexToRGB("00FF00");
            break;
          case 0x45:
            // blue
            hexToRGB("0000FF");
            break;
          case 0x44:
            // white
            hexToRGB("FFFFFF");
            break;
          default:
            Serial.println("ERROR: IR recieved unknown value: " + IrReceiver.decodedIRData.command);
        }
    }

  // piezo reads analog
  if (analogRead(PIEZO_PIN) > PIEZO_THRESH) {

    // if (RED == MAX_INTENSITY) {
    //   RED = 0;
    //   GREEN = MAX_INTENSITY;
    //   BLUE = 0;
    // } else if (GREEN == MAX_INTENSITY) {
    //   RED = 0;
    //   BLUE = MAX_INTENSITY;
    //   GREEN = 0;
    // } else if (BLUE == MAX_INTENSITY) {
    //   RED = MAX_INTENSITY;
    //   GREEN = 0;
    //   BLUE = 0;
    // }

    // Button is pressed, flash the RGB LED
    flashRGB();
  } else {
    // Turn off the LED when the button is not pressed
    analogWrite(RED_PIN, 0);
    analogWrite(GREEN_PIN, 0);
    analogWrite(BLUE_PIN, 0);
  }
  
}

void flashRGB() {
  // turn on
  analogWrite(RED_PIN, RED);
  analogWrite(GREEN_PIN, GREEN);
  analogWrite(BLUE_PIN, BLUE);

  delay(100);
}

void hexToRGB(String hexCode) {
  long num = strtol(hexCode.c_str(), nullptr, 16);
  RED = num >> 22;
  GREEN = num >> 8 & 0xFF;
  BLUE = num & 0xFF;
}