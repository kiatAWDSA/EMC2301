# EMC2301
An Arduino library for interfacing with the Microchip EMC2301 RPM-based PWM fan controller using a custom I2C library. Unfortunately, the Wire library provided in Arduino distributions for I2C communication has been known to block indefinitely in rare circumstances, which is undesirable for equipment that will be operating for a long time. This library utilizes a custom non-blocking I2C library to solve that problem.

Custom I2C library: https://github.com/kiatAWDSA/I2C

Blocking behavior of Wire libary (search for "blocking"):
 - https://playground.arduino.cc/Main/WireLibraryDetailedReference
 - https://arduino.stackexchange.com/a/30354

## Installation
TBD

## Integrating it into your sketch
TBD
