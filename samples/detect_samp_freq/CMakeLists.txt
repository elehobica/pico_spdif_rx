cmake_minimum_required(VERSION 3.13)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)
include($ENV{PICO_EXTRAS_PATH}/external/pico_extras_import.cmake)

set(project_name "detect_samp_freq" C CXX ASM)
project(${project_name})
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

add_subdirectory(../.. pico_spdif_rx)
add_subdirectory(../lib/pico_audio_i2s_32b/src/pico_audio_32b pico_audio_32b)

# select sample application here (choose only one)
add_executable(${PROJECT_NAME}
    main.cpp
)

target_link_libraries(${PROJECT_NAME}
    pico_stdlib
    pico_spdif_rx
    pico_audio_32b
)

# set core1 process i2s_callback
target_compile_definitions(${PROJECT_NAME} PRIVATE
    CORE1_PROCESS_I2S_CALLBACK
)

pico_add_extra_outputs(${PROJECT_NAME}) # create map/bin/hex/uf2 files
