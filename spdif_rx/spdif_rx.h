/*------------------------------------------------------/
/ Copyright (c) 2023, Elehobica
/ Released under the BSD-2-Clause
/ refer to https://opensource.org/licenses/BSD-2-Clause
/------------------------------------------------------*/

#pragma once

#include "pico/stdlib.h"
#include "pico/audio.h"

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

typedef struct _spdif_rx_config_t {
    uint8_t data_pin;
    uint8_t pio_sm;
    uint8_t dma_channel0;
    uint8_t dma_channel1;
} spdif_rx_config_t;

typedef enum _spdif_rx_samp_freq_t {
    SAMP_FREQ_NONE = 0,
    SAMP_FREQ_44100 = 44100,
    SAMP_FREQ_48000 = 48000
} spdif_rx_samp_freq_t;

#define SAMP_FREQ_44100 (44100)
#define SAMP_FREQ_48000 (48000)

void spdif_rx_setup(const spdif_rx_config_t *config);
void spdif_rx_end();
bool spdif_rx_status();
float spdif_rx_get_samp_freq_actual();
spdif_rx_samp_freq_t spdif_rx_get_samp_freq();
void spdif_rx_search_next();
uint32_t spdif_rx_get_c_bits();
uint32_t spdif_rx_get_parity_err_count();
uint32_t spdif_rx_get_fifo_count();
uint32_t spdif_rx_read_fifo(uint32_t** buff, uint32_t req_count);
uint32_t* spdif_rx_read_fifo_single();

#ifdef __cplusplus
}
#endif