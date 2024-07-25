const int buttonPin = 2;  // Pin where the button is connected

void setup() {
  // Initialize serial communication
  Serial.begin(9600);
  
  // Initialize the button pin as an input with internal pull-up resistor
  pinMode(buttonPin, INPUT_PULLUP);
}

void loop() {
  // Read the state of the button
  int buttonState = digitalRead(buttonPin);
  
  // Check if the button is pressed
  if (buttonState == HIGH) {  // LOW means the button is pressed
    Serial.println("Button Pressed!");
    delay(500);  // Debounce delay to avoid multiple prints
  }
}
