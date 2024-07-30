
/******************************************************************************
 * @file       hc-12-rx.ino
 * @brief      Arduino RF communication using HC-12 module as receiver
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

SoftwareSerial HC12(2, 3);  // HC-12 TX Pin, HC-12 RX Pin

void setup() {
  Serial.begin(9600);  // Serial monitor for debugging
  HC12.begin(9600);    // Initialize HC-12 serial communication
}

void loop() {
  if (HC12.available()) {        // Continuously check for available data
    char receivedChar = HC12.read();  // Read incoming data
    Serial.print(receivedChar);     // Print the received char to Serial Monitor
  }
}

