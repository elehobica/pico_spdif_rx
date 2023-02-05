# Raspberry Pi Pico spdif_rx

## Overview
* SPDIF decoder by rp2040 PIO function
* support 2ch, 16bit or 24bit format
* supported sampling frequency: 44.1 KHz, 48.0 KHz

## Supported Board and Peripheral Devices
* Raspberry Pi Pico (rp2040)
* SPDIF Coaxial or TOSLINK (RX178A or equivalent) receiver
* PCM5102 (spdif_to_i2s_32 project only)

## Pin Assignment
### SPDIF Rx
| Pico Pin # | GPIO | Function | Connection |
----|----|----|----
| 20 | GP15 | DATA | from SPDIF/TOSLINK Data output|

![TOSLINK_Rx_Schematic](doc/TOSLINK_Rx_Schematic.png)

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
> git clone https://github.com/raspberrypi/pico-extras.git
> cd pico-extras
> git submodule update -i
> cd ..
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

## Decode output format
* 32bit output for each SPDIF sub frame including header, audio data, AUX, VUCP
* see comments in [spdif_rx.pio](spdif_rx/spdif_rx.pio) for further detail

## Sample projects
select target project file in `CmakeLists.txt`
### detect_samp_freq
* scan candidate sampling frequencies until successfully decoded
* display sampling frequency, C bits of SPDIF frame and number of parity errors while decoded

### spdif_to_i2s_32b
* convert SPDIF input to I2S 32bit