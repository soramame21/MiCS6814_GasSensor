# multi-gas-sensor-lib
This is a mbed-os 5 driver for [MiCS-6814 Multichannel Gas Sensor (seeed)](http://wiki.seeed.cc/Grove-Multichannel_Gas_Sensor/)

A [Base Shield V2](https://www.seeedstudio.com/Base-Shield-V2-p-1378.html) is require to connect the sensor. please turn the supply voltage (VCC) switch to 3.3v.

The 7-bit I2C slave address is treated by mbed-os as 0x08, although the same sensor address is treated by Arduino as 0x04  

The difference is as following.
```
Arduino treats 7-bit address as 0XXX XXXX.
mbed-os treats 7-bit address as XXXX XXX0
```

Following function APIs are not supported.
* Change I2C address of the sensor
* Calibrate sensor
