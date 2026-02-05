# Modifying and Flashing the Joint Firmware
The joint firmware is programmed in C++ using the Arduino framework since the uStepperS32 stepper library is released for that ecosystem.


## Prerequisites
In order to flash firmware to the uStepperS32 stepper controllers some prerequisites must be met. The following guide is copied for reference from the [uStepper website](https://www.ustepper.com/getting-started-with-ustepper/).

### Install the Arduino IDE
Installing the uStepperS32 library and flashing the firmware is done through the Arduino IDE which needs to be installed on the developer's computer from [here](https://www.arduino.cc/en/software/).

### Install STM32 CUBE Programmer
The uStepper is based on an STM32F401 microcontroller. The STM32 CUBE Programmer is used behind the scenes to flash the firmware from the Arduino IDE.
Install the programmer from [here](https://www.st.com/en/development-tools/stm32cubeprog.html).

### Add the uStepperS32 library to the Arduino IDE
In the Arduino IDE, go to *File*->*preferences* and append the URL `https://raw.githubusercontent.com/uStepper/uStepperSTM32Hardware/master/package.json` to the Additional Boards Manager URLs field at the bottom and accept with OK.

Then install the uStepper board through the board manager under *Tools*->*Board*->*Boards Manager*. Search for "ustepper" and the board should show up after the board manager has refreshed. Then press INSTALL.

Finally it should now be possible to select the uStepper STM32 boards under *tools*->*boards*.
        
### Install the uStepperS32 Library
Go to *Sketch*->*Include Library*->*Manage Libraries*, search for “uStepperS32”, and press INSTALL.
Now both the hardware and the library are setup for compilation and flashing.

## Flashing the Firmware
To flash the firmware on the uStepper board, open the *lib/joint_firmware/joint/joint.ino* firmware file with the Arduino IDE, connect the uStepper board with a USB-C cable to the computer, select the correct serial port and the board as "uStepper STM32 boards" under the "board and port" tab. 

The follow the steps to flash the firmware:
1. Press and hold the BOOT button on the uStepper board
2. Keep the BOOT button is pressed and press the RESET button for at least 0.5 s
3. Release the BOOT button
4. Select UPLOAD in the Arduino IDE. The board is now in dowload mode and will load the new firmware.

Monitor the Arduino IDE output console for potential error messages.

## Other Notes
The buttons on the uStepper board are very difficult to reach on J2 if it is mounted on the robot. A thin non-conductive stick can be used to press the BOOT button, the RESET button should be reachable by hand.