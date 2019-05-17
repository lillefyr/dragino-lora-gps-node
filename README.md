raspinode

Implementation of Dragino Lora + GPS as a node

using Raspberry Pi port LoRaMAC in C / LoRaWAN in C http://www.research.ibm.com/labs/zurich/ics/lrsc/lmic.html

This is a port of IBM's LMIC 1.5 to the Raspberry Pi using the wiringPi library for GPIO/SPI.
It is adapted for TTN (http://thethingsnetwork.org).

It has been tested with an Dragino Lora GPS HAT on Raspberry PI 2B ( and maybe on raspberry pi zero )

Some of the changes were taken from or inspired by the arduino-lmic-v1.5 port of tftelkamp (https://github.com/tftelkamp/arduino-lmic-v1.5.git) 

The connections of the pins are defined in the main programs in the examples directory.
Standard connections are:
  WiringPi 6  == nss
  WiringPi 0 == reset (needed for RFM92/RFM95)
  WiringPi 7,4,5 == dio0, dio1, dio2
  WiringPi 12 == MOSI
  WiringPi 13 == MISO
  WiringPi 14 == SCK
  GND  == GND
  3.3V  == +3.3V
