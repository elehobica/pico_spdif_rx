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

#ifndef PICO_SPDIF_RX_PIO
#define PICO_SPDIF_RX_PIO 0
#endif

// PICO_CONFIG: SPINLOCK_ID_SPDIF_RX_PREPARED_LISTS_LOCK, Spinlock number for the audio prepared list, min=0, max=31, default=8, group=audio
#ifndef SPINLOCK_ID_SPDIF_RX_PREPARED_LISTS_LOCK
#define SPINLOCK_ID_SPDIF_RX_PREPARED_LISTS_LOCK 12
#endif

#if !(PICO_SPDIF_RX_DMA_IRQ == 0 || PICO_SPDIF_RX_DMA_IRQ == 1)
#error PICO_SPDIF_RX_DMA_IRQ must be 0 or 1
#endif

#if !(PICO_SPDIF_RX_PIO == 0 || PICO_SPDIF_RX_PIO == 1)
#error PICO_SPDIF_RX_PIO ust be 0 or 1
#endif

typedef struct _spdif_rx_config_t {
    uint8_t data_pin;
    uint pio_sm;
    uint dma_channel0;
    uint dma_channel1;
    uint alarm;
    bool full_check; // if false, no C_bits info, no parity check, no callback, but light weight processing
} spdif_rx_config_t;

typedef enum _spdif_rx_samp_freq_t {
    SAMP_FREQ_NONE = 0,
    SAMP_FREQ_44100 = 44100,
    SAMP_FREQ_48000 = 48000,
    SAMP_FREQ_88200 = 88200,
    SAMP_FREQ_96000 = 96000,
    SAMP_FREQ_176400 = 176400,
    SAMP_FREQ_192000 = 192000
} spdif_rx_samp_freq_t;

typedef enum _spdif_rx_state_t  {
    SPDIF_RX_STATE_NO_SIGNAL = 0,
    SPDIF_RX_STATE_WAITING_STABLE,
    SPDIF_RX_STATE_STABLE
} spdif_rx_state_t;

#define SPDIF_BLOCK_SIZE (384) // sub frames per block
#define NUM_BLOCKS (8) // must be >= 2
#define SPDIF_RX_FIFO_SIZE (NUM_BLOCKS * SPDIF_BLOCK_SIZE)

void spdif_rx_start(const spdif_rx_config_t *config);
void spdif_rx_end();
void spdif_rx_set_callback_on_stable(void (*func)(spdif_rx_samp_freq_t samp_freq));
void spdif_rx_set_callback_on_lost_stable(void (*func)());

spdif_rx_state_t spdif_rx_get_state();
float spdif_rx_get_samp_freq_actual();
spdif_rx_samp_freq_t spdif_rx_get_samp_freq();
uint32_t spdif_rx_get_c_bits();
uint32_t spdif_rx_get_parity_err_count();
uint32_t spdif_rx_get_fifo_count();
uint32_t spdif_rx_read_fifo(uint32_t** buff, uint32_t req_count);
uint32_t* spdif_rx_read_fifo_single();

#ifdef __cplusplus
}
#endif