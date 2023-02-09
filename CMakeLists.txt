if (NOT TARGET spdif_rx)
    add_library(spdif_rx INTERFACE)

    pico_generate_pio_header(spdif_rx ${CMAKE_CURRENT_LIST_DIR}/spdif_rx.pio)

    target_sources(spdif_rx INTERFACE
            ${CMAKE_CURRENT_LIST_DIR}/spdif_rx.c
    )

    target_include_directories(spdif_rx INTERFACE ${CMAKE_CURRENT_LIST_DIR}/..)
    target_link_libraries(spdif_rx INTERFACE
        pico_stdlib
        pico_multicore
        hardware_dma
        hardware_pio
        hardware_irq
        my_pico_audio
        spdif_rx
    )
endif()
