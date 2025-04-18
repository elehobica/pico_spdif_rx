# Change Log
All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](http://keepachangelog.com/)
and this project adheres to [Semantic Versioning](http://semver.org/).

## [Unreleased]
### Changed
* Get python3 path by FindPython3 for multi platforms
* Support pico-sdk 2.1.1
### Added
* Support Raspberry Pi Pico 2 series board (up to 96.0 KHz)

## [v0.9.2] - 2023-04-05
### Changed
* Support pico-sdk 1.5.1 (previously 1.4.0)
* Change library name to pico_spdif_rx (previously spdif_rx)
* Replace PICO_SPDIF_RX_PIO, PICO_SPDIF_RX_DMA_IRQ macro with add_compile_definitions() in CmakeLists.txt
* Use alarm pool in stead of immediate hardware alarm number

### Fixed
* Fix CMakeLlists.txt filename (PR by DaveVdE)

## [v0.9.1] - 2023-03-02
### Added
* Introduce interrupt relay and callbacks, which makes main loop free from polling for detection retry

### Changed
* Faster detection for the sampling frequency by direct capturing and analysis (contributed by IDC-Dragon)
* Remove the PIO program for inverted bitstream by GPIO inversion override (remarked by IDC-Dragon)
* Expand support of C bits into whole 192 bits

### Fixed
* Fix the bugs which appear only in debug mode
* Many fixes for the wrong procedures to DMA and PIO, which could hang the process


## [v0.9.0] - 2023-03-02
* Initial release