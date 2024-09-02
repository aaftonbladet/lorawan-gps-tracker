# Lora GPS tracker

This project implements a battery-powered GPS tracker that transmits its location using LoRaWAN on The Things Network. It's compatible with my [Adafruit IO forwarder for TTN](https://github.com/aaftonbladet/ttn-adafruit-io-forwarder). You can create a map preview in your Adafruit IO dashboard to see the real-time location of your device.

## Required components

- An Adafruit Feather M0 with a RFM96 LoRa radio
- An Adafruit GPS FeatherWing
- A lithium-ion battery compatible with the M0

The pin labled EN on the GPS wing needs to be wired to a broken out pin, so we can disable the GPS to save energy.

## Configuring and building

Clone the project, then make a copy of `secrets.template.h`, name it `secrets.h`. You should edit `TTN_DevEUI`, `TTN_NwkKey` and `TTN_DevEUI` to reflect that of your device. The project requires the CayenneLPP, Adafruit_GPS, Adafruit_SleepyDog and  libraries. You can install them all through the Arduino library manager.

You should also be editing `GPS_DISABLE_PIN`, `BATTERY_VOLTAGE_PIN`, `REFERENCE_VOLTAGE`, as well as the GPS and radio constructors, all in the main file, to reflect that of your setup. 