
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

SoftwareSerial HC12(2, 3);  // HC-12 TX Pin, HC-12 RX Pin

void setup() {
  HC12.begin(9600);  // Initialize HC-12 serial communication
}

void loop() {
  
    send_data(255, 0, 0);  // Send red color
    delay(1000);           // Delay for 1 second
    send_data(0, 255, 0);  // Send green color
    delay(1000);           // Delay for 1 second
    send_data(0, 0, 255);  // Send blue color
    delay(1000);           // Delay for 1 second

}

void send_data(byte red, byte green, byte blue) {
    HC12.write(red);    // Send red value
    HC12.write(green);  // Send green value
    HC12.write(blue);   // Send blue value
}