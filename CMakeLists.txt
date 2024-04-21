cmake_minimum_required(VERSION 3.12)
if (NOT TARGET pico_spdif_rx)
    add_library(pico_spdif_rx INTERFACE)

    pico_generate_pio_header(pico_spdif_rx ${CMAKE_CURRENT_BINARY_DIR}/spdif_rx_48000.pio)
    pico_generate_pio_header(pico_spdif_rx ${CMAKE_CURRENT_BINARY_DIR}/spdif_rx_96000.pio)
    pico_generate_pio_header(pico_spdif_rx ${CMAKE_CURRENT_BINARY_DIR}/spdif_rx_192000.pio)
    pico_generate_pio_header(pico_spdif_rx ${CMAKE_CURRENT_LIST_DIR}/spdif_rx_capture.pio)

    find_package (Python3 COMPONENTS Interpreter)
    message(STATUS "Python: version=${Python3_VERSION} interpreter=${Python3_EXECUTABLE}")
    if(NOT Python3_FOUND)
        string(JOIN "\n" errmsg
        "  Python3 not found."
        "  - Python3_FOUND=${Python3_FOUND}"
        )
        message(FATAL_ERROR ${errmsg})
    endif()

    add_custom_command(
        OUTPUT
            ${CMAKE_CURRENT_BINARY_DIR}/spdif_rx_48000.pio
            ${CMAKE_CURRENT_BINARY_DIR}/spdif_rx_96000.pio
            ${CMAKE_CURRENT_BINARY_DIR}/spdif_rx_192000.pio
        DEPENDS ${CMAKE_CURRENT_LIST_DIR}/spdif_rx.pio
        COMMAND ${Python3_EXECUTABLE}
            ${CMAKE_CURRENT_LIST_DIR}/scripts/gen_pio_by_freq.py
            ${CMAKE_CURRENT_LIST_DIR}/spdif_rx.pio
            ${CMAKE_CURRENT_BINARY_DIR}
            48000
            96000
            192000
    )

    target_sources(pico_spdif_rx INTERFACE
        ${CMAKE_CURRENT_LIST_DIR}/spdif_rx.c
    )

    target_include_directories(pico_spdif_rx INTERFACE
        ${CMAKE_CURRENT_LIST_DIR}
    )

    target_link_libraries(pico_spdif_rx INTERFACE
        pico_stdlib
        hardware_dma
        hardware_pio
        hardware_irq
        pico_audio_32b
    )
endif()
