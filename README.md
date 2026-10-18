# S/PDIF receiver library for Raspberry Pi Pico series

[![Build](https://github.com/elehobica/pico_spdif_rx/actions/workflows/build-binaries.yml/badge.svg)](https://github.com/elehobica/pico_spdif_rx/actions/workflows/build-binaries.yml)

## Overview
* S/PDIF receiver with sampling frequency detection by PIO function
* Supported format:
  * Channels: 2 (Stereo)
  * Bit resolution: 16bit and 24bit
* Supported sampling frequencies:
  * 44.1 KHz, 48.0 KHz, 88.2 KHz, 96.0 KHz, 176.4 KHz and 192.0 KHz for Raspberry Pi Pico (rp2040)
  * 44.1 KHz, 48.0 KHz, 88.2 KHz and 96.0 KHz for Raspberry Pi Pico 2 (rp2350)
  * The upper limit can be lowered by `spdif_rx_set_max_samp_freq()` (e.g. for S/PDIF Rx modules not capable of 176.4 KHz / 192.0 KHz)

## Supported Board and Peripheral Devices
* Raspberry Pi Pico (rp2040)
* Raspberry Pi Pico 2 (rp2350)
* S/PDIF Coaxial or TOSLINK Rx module (DLR1160 or equivalent)

## Pin Assignment & Connection
### S/PDIF Rx
| Pico Pin # | GPIO | Function | Connection |
----|----|----|----
| 20 | GP15 | DATA | from S/PDIF data |

![SPDIF_Rx_Schematic](doc/SPDIF_Rx_Schematic.png)

Notes:

* 74HC04 should be genuine device, otherwise (counterfeit device) it could not work at higher sampling frequencies.
* if TOSLINK only, direct connection from module output to GP15 will work as well.

## How to build with docker image
* Builds the firmware inside [pico-sdk-dev-docker:sdk-2.3.0](https://hub.docker.com/r/elehobica/pico-sdk-dev-docker) (same image used by CI). Requires Docker; no local Pico SDK setup is needed.
* `samples/build_docker.sh` drives the container build. The sample to build is taken from the current directory, so run it from inside the sample folder you want to build (`samples/xxxx`).
```
$ git clone -b main --recursive https://github.com/elehobica/pico_spdif_rx.git
$ cd pico_spdif_rx/samples/xxxx
$ ../build_docker.sh           # build both targets (default)
$ ../build_docker.sh pico      # build only Pico    -> build/xxxx.uf2
$ ../build_docker.sh pico2     # build only Pico 2  -> build2/xxxx.uf2
```
* Outputs: `build/xxxx.uf2` (Pico), `build2/xxxx.uf2` (Pico 2)
* Download "*.uf2" on RPI-RP2 or RP2350 drive

## How to build in local
* See ["Getting started with Raspberry Pi Pico"](https://datasheets.raspberrypi.org/pico/getting-started-with-pico.pdf)
* Put "pico-sdk", "pico-examples" and "pico-extras" on the same level with this project folder.
* Set environmental variables for PICO_SDK_PATH, PICO_EXTRAS_PATH and PICO_EXAMPLES_PATH
* Confirmed with Pico SDK 2.3.0
```
> git clone -b 2.3.0 https://github.com/raspberrypi/pico-sdk.git
> cd pico-sdk
> git submodule update -i
> cd ..
> git clone -b sdk-2.3.0 https://github.com/raspberrypi/pico-examples.git
>
> git clone -b sdk-2.3.0 https://github.com/raspberrypi/pico-extras.git
> 
> git clone -b main --recursive https://github.com/elehobica/pico_spdif_rx.git
```
### Windows
* Build is confirmed with Developer Command Prompt for VS 2022 and Visual Studio Code on Windows environment
* Confirmed with cmake-3.27.2-windows-x86_64 and gcc-arm-none-eabi-10.3-2021.10-win32
* Lanuch "Developer Command Prompt for VS 2022"
```
> cd pico_spdif_rx\samples\xxxxx  # sample project directory
> mkdir build && cd build
> cmake -G "NMake Makefiles" ..  ; (for Raspberry Pi Pico 1 series)
> cmake -G "NMake Makefiles" -DPICO_PLATFORM=rp2350 -DPICO_BOARD=pico2 ..  ; (for Raspberry Pi Pico 2)
> nmake
```
* Put "*.uf2" on RPI-RP2 or RP2350 drive
### Linux
* Build is confirmed with [pico-sdk-dev-docker:sdk-2.3.0]( https://hub.docker.com/r/elehobica/pico-sdk-dev-docker)
* Confirmed with cmake-3.22.1 and arm-none-eabi-gcc (15:10.3-2021.07-4) 10.3.1
```
$ cd pico_spdif_rx/samples/xxxxx  # sample project directory
$ mkdir build && cd build
$ cmake ..  # (for Raspberry Pi Pico 1 series)
$ cmake -DPICO_PLATFORM=rp2350 -DPICO_BOARD=pico2 ..  # (for Raspberry Pi Pico 2)
$ make -j4
```
* Download "*.uf2" on RPI-RP2 or RP2350 drive

## Decode output format
* 32bit output for each S/PDIF sub frame including header, audio data, AUX, VUCP
* see comments in [spdif_rx.pio](spdif_rx.pio) for further detail

## Sample projects
### detect_samp_freq
* display sampling frequency, C bits of S/PDIF frame and number of parity errors while decoding

### spdif_to_i2s_32b
* convert S/PDIF input to I2S 32bit output

## For more detail about internal code structure
* See [DeepWiki](https://deepwiki.com/elehobica/pico_spdif_rx) (powered by [Devin](https://app.devin.ai/invite/WFPByHrQP7TwsUuq))

## Application Examples
* [pico_spdif_recorder](https://github.com/elehobica/pico_spdif_recorder)
* [pico_spdif_dac_hpamp](https://github.com/elehobica/pico_spdif_dac_hpamp)
