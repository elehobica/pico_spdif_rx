/*------------------------------------------------------/
/ Copyright (c) 2023, Elehobica
/ Released under the BSD-2-Clause
/ refer to https://opensource.org/licenses/BSD-2-Clause
/------------------------------------------------------*/

#define PICO_SPDIF_RX_DMA_IRQ 1
#define PICO_SPDIF_RX_PIO 1

#include <stdio.h>
#include "spdif_rx.h"

#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/irq.h"
#include "spdif_rx.pio.h"

#define BUF_SIZE 1024*32
static uint32_t buff[BUF_SIZE];
uint32_t* ptr = buff;
uint8_t prevOut;
uint32_t syncSymCount = 0;
uint32_t dataCount = 0;
uint32_t totalCount = 0;
uint32_t errorCount = 0;
uint32_t frameCount = 0;
uint32_t blockCount = 0;

CU_REGISTER_DEBUG_PINS(spdif_rx_timing)

#define spdif_rx_pio __CONCAT(pio, PICO_SPDIF_RX_PIO)
#define DREQ_PIOx_RX0 __CONCAT(__CONCAT(DREQ_PIO, PICO_SPDIF_RX_PIO), _RX0)

#define dma_intsx __CONCAT(dma_hw->ints, PICO_SPDIF_RX_DMA_IRQ)
#define dma_channel_set_irqx_enabled __CONCAT(__CONCAT(dma_channel_set_irq, PICO_SPDIF_RX_DMA_IRQ),_enabled)
#define DMA_IRQ_x __CONCAT(DMA_IRQ_, PICO_SPDIF_RX_DMA_IRQ)

static uint loaded_offset = 0;
static struct {
    uint8_t pio_sm;
    uint8_t dma_channel;
} shared_state;

// irq handler for DMA
void __isr __time_critical_func(spdif_rx_dma_irq_handler)() {
#if PICO_AUDIO_I2S_NOOP
    assert(false);
#else
    uint dma_channel = shared_state.dma_channel;
    if (dma_intsx & (1u << dma_channel)) {
        dma_intsx = 1u << dma_channel;
        DEBUG_PINS_SET(spdif_rx_timing, 4);
        printf("spdif_rx_timing");
        /*
        // free the buffer we just finished
        if (shared_state.playing_buffer) {
            //give_audio_buffer(audio_i2s_consumer, shared_state.playing_buffer);
#ifndef NDEBUG
            shared_state.playing_buffer = NULL;
#endif
        }
        audio_start_dma_transfer();
        */
        DEBUG_PINS_CLR(spdif_rx_timing, 4);
    }
#endif
}

void spdif_rx_end()
{
    uint8_t sm = shared_state.pio_sm;
    pio_sm_unclaim(spdif_rx_pio, sm);
    pio_remove_program(spdif_rx_pio, &spdif_rx_program, loaded_offset);
    pio_clear_instruction_memory(spdif_rx_pio);
    uint8_t dma_channel = shared_state.dma_channel;
    dma_channel_unclaim(dma_channel);
    irq_remove_handler(DMA_IRQ_x, spdif_rx_dma_irq_handler);
    dma_channel_set_irqx_enabled(dma_channel, 0);
}

void spdif_rx_setup(const spdif_rx_config_t *config)
{
    uint8_t sm = shared_state.pio_sm = config->pio_sm;
    pio_sm_claim(spdif_rx_pio, sm);
    pio_sm_set_clkdiv(spdif_rx_pio, shared_state.pio_sm, 1);
    loaded_offset = pio_add_program(spdif_rx_pio, &spdif_rx_program);
    spdif_rx_program_init(spdif_rx_pio, sm, loaded_offset, config->data_pin);


    /*
    __mem_fence_release();
    uint8_t dma_channel = config->dma_channel;
    dma_channel_claim(dma_channel);
    shared_state.dma_channel = dma_channel;

    dma_channel_config dma_config = dma_channel_get_default_config(dma_channel);
    channel_config_set_dreq(&dma_config, DREQ_PIOx_RX0 + sm);
    enum dma_channel_transfer_size spdif_rx_dma_configure_size = DMA_SIZE_32;
    channel_config_set_transfer_data_size(&dma_config, spdif_rx_dma_configure_size);
    dma_channel_configure(
        dma_channel,
        &dma_config,
        NULL, // dest
        &spdif_rx_pio->rxf[sm],  // src
        0, // count
        false // trigger
    );

    irq_add_shared_handler(DMA_IRQ_x, spdif_rx_dma_irq_handler, PICO_SHARED_IRQ_HANDLER_DEFAULT_ORDER_PRIORITY);
    dma_channel_set_irqx_enabled(dma_channel, 1);

    dma_channel_transfer_to_buffer_now(dma_channel, &buff, 1024);
    */
}

void spdif_rx_loop()
{
    if (ptr - buff < BUF_SIZE) {
        *ptr++ = spdif_rx_program_get32(spdif_rx_pio, shared_state.pio_sm);
    } else {
        printf("done\n");
        for (int i = 0; i < BUF_SIZE; i++) {
            for (int j = 0; j < 16; j++) {
                uint8_t out = (buff[i] >> (30 - j*2)) & 0x3;
                switch (out) {
                    case 0x0:
                        printf("0");
                        syncSymCount = 0;
                        dataCount++;
                        break;
                    case 0x1:
                        printf("1");
                        syncSymCount = 0;
                        dataCount++;
                        break;
                    case 0x3:
                        if (prevOut == 0x3) {
                            printf("s");
                            syncSymCount++;
                            dataCount = 0;
                        } else {
                            printf("\ns");
                            if (dataCount != 28 && dataCount != 30) {
                                errorCount++;
                            } else if (dataCount == 28) {
                                blockCount++;
                                frameCount++;
                            } else {
                                frameCount++;
                            }
                            syncSymCount = 1;
                        }
                        break;
                    default:
                        syncSymCount = 0;
                        dataCount = 0;
                        printf("FATAL ERR\n");
                        break;
                }
                prevOut = out;
                totalCount++;
            }
        }
        printf("\n");
        printf("errorCount = %d, frameCount = %d, blockCount = %d\n", errorCount, frameCount, blockCount);
        while (true) {}
    }
}