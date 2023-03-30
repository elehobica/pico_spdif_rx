cmake_minimum_required(VERSION 3.12)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)
include($ENV{PICO_EXTRAS_PATH}/external/pico_extras_import.cmake)

set(project_name "spdif_to_i2s_32b" C CXX ASM)
project(${project_name})
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

add_subdirectory(../.. spdif_rx)
add_subdirectory(../lib/pico_audio_32b pico_audio_32b) # for extended audio.h
add_subdirectory(../lib/pico_audio_i2s_32b pico_audio_i2s_32b)

# select sample application here (choose only one)
add_executable(${PROJECT_NAME}
    main.cpp
)

target_link_libraries(${PROJECT_NAME}
    pico_stdlib
    spdif_rx
    pico_audio_32b
    pico_audio_i2s_32b
)

# set project resource assignment for PIO and DMA_IRQ to avoid confliction
add_compile_definitions(PICO_SPDIF_RX_PIO=0)
add_compile_definitions(PICO_SPDIF_RX_DMA_IRQ=0)
add_compile_definitions(PICO_AUDIO_I2S_PIO=1)
add_compile_definitions(PICO_AUDIO_I2S_DMA_IRQ=1)

pico_add_extra_outputs(${PROJECT_NAME}) # create map/bin/hex/uf2 files
