#include <IRremote.h>

#define BUTTON_PIN1 2 	// Digital pin for Terminal 1A of the push button
#define BUTTON_PIN2 3 	// Digital pin for Terminal 2A of the push button
#define RED_LED 5     	// Red LED connected to digital pin 5
#define GREEN_LED 6   	// Green LED connected to digital pin 6
#define BLUE_LED 9    	// Blue LED connected to digital pin 9

#define IR_SENSOR_PIN 4 // IR sensor pin

// create IR recieer instance
IRrecv irrecv(IR_SENSOR_PIN);
// IR remote codes
decode_results results;

int previous_ir_val = -1; // Store the previous IR value
int buttonState = 1;      // initial button state (1 - unpressed, 0 - pressed)


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
//
// For this test run, we will integrate a single RGB LED and change color
//    according to the remote (will be an RGB remote)
//    button layout will be as folllows:
//    0   1   2
//    4   5   6
//    8   9  10
//   12  13  14
//   R    G   B
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
    if (irrecv.decode()) {
        // Get the infrared code
        unsigned long code = irrecv.decodedIRData.decodedRawData;
        // Map it to a specific button on the remote
        result = mapCodeToButton(code);
        // Enable receiving of the next value
        irrecv.resume();
    }
    return result;
}

void setup() {
    pinMode(BUTTON_PIN1, INPUT_PULLUP); // Set button 1 pin as input with internal pull-up resistor
    pinMode(BUTTON_PIN2, INPUT_PULLUP); // Set button 2 pin as input with internal pull-up resistor
    pinMode(RED_LED, OUTPUT);           // Set the red LED pin as output
    pinMode(GREEN_LED, OUTPUT);         // Set the green LED pin as output
    pinMode(BLUE_LED, OUTPUT);          // Set the blue LED pin as output
    
    // irrecv.enableIRIn();
    irrecv.begin(IR_SENSOR_PIN);
    // begin serial output
    Serial.begin(9600);
}

void loop() {
    int ir_val = readInfrared();

    if (ir_val >= 0 && ir_val != previous_ir_val) {
        previous_ir_val = ir_val; // Update the previous IR value
        
        // Wait for another IR input
        while (readInfrared() == ir_val) {
            // Do nothing, just wait
            switch (ir_val) {
                case 16:
                    digitalWrite(RED_LED, HIGH);
                    digitalWrite(GREEN_LED, LOW);
                    digitalWrite(BLUE_LED, LOW);
                    break;
                case 17:
                    digitalWrite(RED_LED, LOW);
                    digitalWrite(GREEN_LED, HIGH);
                    digitalWrite(BLUE_LED, LOW);
                    break;
                case 18:
                    digitalWrite(RED_LED, LOW);
                    digitalWrite(GREEN_LED, LOW);
                    digitalWrite(BLUE_LED, HIGH);
                    break;
                default:
                    break;
            }
        }

        // Process the new IR input

        delay(10);

        digitalWrite(RED_LED, LOW);
        digitalWrite(GREEN_LED, LOW);
        digitalWrite(BLUE_LED, LOW);

    }
}
