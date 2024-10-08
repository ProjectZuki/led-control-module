## Table of Contents

1. [Installation](#installation)
2. [Usage](#usage)
3. [Features](#features)
4. [License](#license)
5. [Contact](#contact)

## Installation

Install [Arduino IDE](https://www.arduino.cc/en/software) (version 2.3.2 or higher)</br>
Install required Libraries (see [Install Libraries](#install-libraries))

## Usage
Supported Boards:
- Arduino Nano / Nano Every
- Arduino Uno
- Arduino Mega
- _Support for additional boards will be added in future updates._

Required Libraries:
- IRremote (version 4.4.0 or higher)
- FastLED (version 3.7.0 or higher)
- Adafruit NeoPixeo (version 1.12.3 or higher)

### Install Libraries
```bash
arduino-cli lib install "IRremote@>=4.4.0"
arduino-cli lib install "FastLED@>=3.7.0"
arduino-cli lib install "Adafruit NeoPixel@>=1.12.3"
```

## Features
- IR Remote Integration
    - Recieve and decode IR signals from a remote
    - Set and modify color preference from IR signal
- RGB LED control
    - Control RGB LEDs
    - Change colors based on IR remote commands
- Piezo sensor as input
    - Use a piezo sensor for detecting hits
    - Adjusting sensitivity and logic to only identify sharp hits
    - Sensitivity modifications

## License

### Proprietary License

This repository and its contents are proprietary to **[Project]Zuki**. All intellectual property rights are reserved.

You are not permitted to redistribute, modify, or use this code in any form without explicit written permission from **[Project]Zuki**.

For licensing inquiries or permissions, please contact **[Project]Zuki** at [willie.alcaraz@gmail.com](mailto:willie.alcaraz@gmail.com).

The full details of this license can be found in the [LICENSE.pm](lib/Software/License/LICENSE.pm) file.

For more information about **[Project]Zuki** and other projects, visit my website: [williealcaraz.dev](https://williealcaraz.dev).

Copyright **©2024 [Project]Zuki**.



## Contact

For inquiries regarding this project, please provide your contact information.

[Email](mailto:willie.alcaraz@gmail.com?subject=Github%20|%20arduino-db%20Project%20Inquiry) | 
[Github](https://github.com/ProjectZuki)

[Project]Zuki