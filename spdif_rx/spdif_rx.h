/*------------------------------------------------------/
/ Copyright (c) 2023, Elehobica
/ Released under the BSD-2-Clause
/ refer to https://opensource.org/licenses/BSD-2-Clause
/------------------------------------------------------*/

#pragma once

#include "pico/stdlib.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifndef PICO_SPDIF_RX_DMA_IRQ
#define PICO_SPDIF_RX_DMA_IRQ 0
#endif

#ifndef PICO_SPDIF_RX_PIO0
#define PICO_SPDIF_RX_PIO0 0
#endif

#ifndef PICO_SPDIF_RX_PIO1
#define PICO_SPDIF_RX_PIO1 1
#endif

#if !(PICO_SPDIF_RX_DMA_IRQ == 0 || PICO_SPDIF_RX_DMA_IRQ == 1)
#error PICO_SPDIF_RX_DMA_IRQ must be 0 or 1
#endif

#if !(PICO_SPDIF_RX_PIO0 == 0 || PICO_SPDIF_RX_PIO0 == 1)
#error PICO_SPDIF_RX_PIO0 ust be 0 or 1
#endif

#if !(PICO_SPDIF_RX_PIO0 == 1 || PICO_SPDIF_RX_PIO1 == 1)
#error PICO_SPDIF_RX_PIO1 ust be 0 or 1
#endif

typedef struct spdif_rx_config {
    uint8_t data_pin;
    uint8_t pio_sm;
    uint8_t dma_channel0;
    uint8_t dma_channel1;
} spdif_rx_config_t;

void spdif_rx_setup(const spdif_rx_config_t *config);
void spdif_rx_end();
void spdif_rx_check();
void spdif_rx_status();

#ifdef __cplusplus
}
#endif