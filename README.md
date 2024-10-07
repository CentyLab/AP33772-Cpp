## Info
AP33772 is a USB PD3.0 Sink controller that communicate via I2C. With this library, can you use the IC with any Arduino compatable board as it is based on the Wire.h library. This library currently does not support interrupt from the AP33772.

Main feature of the library:
+ Fix voltage request
+ PPS voltage/current request
+ Voltage reading
+ Current reading
+ NTC temperature reading
+ Set Over Current Threshold (if sense resistor is available)
+ Set Over Temperature Threshold
+ Set/clear mask bits

## Using Wire1
The RP2040 has two different I2C physical driver, `Wire` (I2C0) and `Wire1`(I2C1). The default channel is selected to be `Wire` in AP33772 Constructor but one can overwrite it with:

```
AP33772 usbpd(Wire1);
```

## Requesting PPS Voltage/Current
When setting Programable Power Supply mode, it is best to set both Voltage and Current at the same time. With the attached load, the source will try to provide the requested voltage. However, if the given voltage result in current exceed limit, the source will lower the voltage to maintain the max current. Note that PPS mode lowest current limit is 1A.

The example code only set the voltage with the max current the charge/cable can deliver.

Example:
```
setSupplyVoltageCurrent(4200, 1000); // Requesting CV/CC supply 4.2V@1A
```

## Example code
Drag and drop file from "Compiled for PicoPD" to test out the code without compiling. However, some code does has load switch turn on by default. Ensure your connected device can handle the voltage at VBUS.

Else if you would like to compile your own code, ensure to install [arlephilhower's Pico Core](https://github.com/earlephilhower/arduino-pico#installation) so that you can call Wire.setSCL() and Wire.setSDA()


