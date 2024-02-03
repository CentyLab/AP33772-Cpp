## Info
AP33772 is a USB PD3.0 Sink controller that communicate via I2C. With this library, can you use the IC with any Arduino compatable board as it is based on the Wire.h library. This library currently does not support interrupt from the AP33772.

Main feature of the library:
+ Fix votlage and PPS voltage request
+ Voltage reading
+ Current reading
+ NTC temperature reading
+ Set Over Current Threshold (if sense resistor is available)
+ Set Over Temperature Threshold
+ Set/clear mask bits

## Usage
Drag and drop file from "Compiled for PicoPD" to test out the code without compiling. However, some code does has load switch turn on by default. Ensure your connected device can handle the voltage at VBUS.

Else if you would like to compile your own code, ensure to install [arlephilhower's Pico Core](https://github.com/earlephilhower/arduino-pico#installation) so that you can call Wire.setSCL() and Wire.setSDA()



