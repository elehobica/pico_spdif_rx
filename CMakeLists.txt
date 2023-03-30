if (NOT TARGET spdif_rx)
    add_library(spdif_rx INTERFACE)

    pico_generate_pio_header(spdif_rx ${CMAKE_CURRENT_BINARY_DIR}/spdif_rx_48000.pio)
    pico_generate_pio_header(spdif_rx ${CMAKE_CURRENT_BINARY_DIR}/spdif_rx_96000.pio)
    pico_generate_pio_header(spdif_rx ${CMAKE_CURRENT_BINARY_DIR}/spdif_rx_192000.pio)
    pico_generate_pio_header(spdif_rx ${CMAKE_CURRENT_LIST_DIR}/spdif_rx_capture.pio)

    add_custom_command(
        OUTPUT
            ${CMAKE_CURRENT_BINARY_DIR}/spdif_rx_48000.pio
            ${CMAKE_CURRENT_BINARY_DIR}/spdif_rx_96000.pio
            ${CMAKE_CURRENT_BINARY_DIR}/spdif_rx_192000.pio
        DEPENDS ${CMAKE_CURRENT_LIST_DIR}/spdif_rx.pio
        COMMAND "python"
            ${CMAKE_CURRENT_LIST_DIR}/scripts/gen_pio_by_freq.py
            ${CMAKE_CURRENT_LIST_DIR}/spdif_rx.pio
            ${CMAKE_CURRENT_BINARY_DIR}
            48000
            96000
            192000
    )

    target_sources(spdif_rx INTERFACE
        ${CMAKE_CURRENT_LIST_DIR}/spdif_rx.c
    )

    target_include_directories(spdif_rx INTERFACE
        ${CMAKE_CURRENT_LIST_DIR}
    )

    target_link_libraries(spdif_rx INTERFACE
        pico_stdlib
        hardware_dma
        hardware_pio
        hardware_irq
        pico_audio_32b
    )
endif()
