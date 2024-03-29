# Bike DAQ Firmware
This repository contains the firmware for the bikedaq. A data acquisition system for bikes.

## Getting Started
Before getting started, make sure you have a proper Zephyr development
environment. Follow the official
[Zephyr Getting Started Guide](https://docs.zephyrproject.org/latest/getting_started/index.html).

### Initialization
```shell
west init -m git@github.com:o7-machinehum/bike-daq.git --mr main bike-daq
cd bike-daq
west update
```

### Building and Flashing
To build the application, run the following command:

## Arduino Nano BLE Board
```shell
west build -b arduino_nano_33_ble app -- -DCONF_FILE=prj.conf
## Double tap the reset button. Should get an order blinking led.
west flash --bossac=$HOME/.arduino15/packages/arduino/tools/bossac/1.9.1-arduino2/bossac
```

## nRF Devboard
```shell
west build -b nrf52dk_nrf52832 app -- -DCONF_FILE=dk.conf
west flash
```
