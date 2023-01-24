# Raspberry Pi Pico spdif_rx

## Overview

## Supported Board and Peripheral Devices
* Raspberry Pi Pico

## Pin Assignment
| Pico Pin # | GPIO | Function | Connection |
----|----|----|----
| 20 | GP15 | DATA | from SPDIF/TOS-Link Data output|

## How to build
* See ["Getting started with Raspberry Pi Pico"](https://datasheets.raspberrypi.org/pico/getting-started-with-pico.pdf)
* Build is confirmed only in Developer Command Prompt for VS 2019 and Visual Studio Code on Windows enviroment
* Put "pico-sdk", "pico-examples" (, "pico-extras" and "pico-playground") on the same level with this project folder.
* Confirmed under Pico SDK 1.4.0
```
> git clone -b master https://github.com/raspberrypi/pico-sdk.git
> cd pico-sdk
> git submodule update -i
> cd ..
> git clone -b master https://github.com/raspberrypi/pico-examples.git
> 
> git clone -b main https://github.com/elehobica/pico_spdif_rx.git
```
* Lanuch "Developer Command Prompt for VS 2019"
```
> cd pico_spdif_rx
> mkdir build
> cd build
> cmake -G "NMake Makefiles" ..
> nmake
```
* Put "pico_spdif_rx.uf2" on RPI-RP2 drive
