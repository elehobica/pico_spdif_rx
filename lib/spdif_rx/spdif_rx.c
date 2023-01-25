/*------------------------------------------------------/
/ Copyright (c) 2023, Elehobica
/ Released under the BSD-2-Clause
/ refer to https://opensource.org/licenses/BSD-2-Clause
/------------------------------------------------------*/

#define PICO_SPDIF_RX_PIO 0
#define PICO_SPDIF_RX_DMA_IRQ 0

#include <stdio.h>
#include "spdif_rx.h"

#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/irq.h"
#include "spdif_rx.pio.h"

#define BUF_SIZE (44100/2)
uint32_t buff[2][BUF_SIZE];
dma_channel_config dma_config0;
dma_channel_config dma_config1;
dma_channel_config dma_config2;
uint8_t prevOut;
uint32_t syncSymCount = 0;
uint32_t dataCount = 0;
uint32_t totalCount = 0;
uint32_t errorCount = 0;
uint32_t frameCount = 0;
uint32_t blockCount = 0;
bool detectedSync = false;

// ---- select at most one ---
CU_REGISTER_DEBUG_PINS(spdif_rx_timing)

#define spdif_rx_pio __CONCAT(pio, PICO_SPDIF_RX_PIO)
#define DREQ_PIOx_RX0 __CONCAT(__CONCAT(DREQ_PIO, PICO_SPDIF_RX_PIO), _RX0)

#define dma_intsx __CONCAT(dma_hw->ints, PICO_SPDIF_RX_DMA_IRQ)
#define dma_channel_set_irqx_enabled __CONCAT(__CONCAT(dma_channel_set_irq, PICO_SPDIF_RX_DMA_IRQ),_enabled)
#define DMA_IRQ_x __CONCAT(DMA_IRQ_, PICO_SPDIF_RX_DMA_IRQ)

static uint loaded_offset = 0;
spdif_rx_config_t shared_state;

static inline uint32_t _millis(void)
{
	return to_ms_since_boot(get_absolute_time());
}

// irq handler for DMA
void __isr __time_critical_func(spdif_rx_dma_irq_handler)() {
    uint dma_channel0 = shared_state.dma_channel0;
    uint dma_channel1 = shared_state.dma_channel1;
    uint8_t sm = shared_state.pio_sm;

    if ((dma_intsx & (1u << dma_channel0))) {
        dma_intsx = 1u << dma_channel0;
        DEBUG_PINS_SET(spdif_rx_timing, 4);
        dma_channel_configure(
            dma_channel0,
            &dma_config0,
            buff[0], // dest
            &spdif_rx_pio->rxf[sm],  // src
            BUF_SIZE, // count
            false // trigger
        );
        //printf("0");
        //printf("1 %d\n", _millis());
        DEBUG_PINS_CLR(spdif_rx_timing, 4);
    }
    if ((dma_intsx & (1u << dma_channel1))) {
        dma_intsx = 1u << dma_channel1;
        DEBUG_PINS_SET(spdif_rx_timing, 4);
        spdif_rx_check();
        dma_channel_configure(
            dma_channel1,
            &dma_config1,
            buff[1], // dest
            &spdif_rx_pio->rxf[sm],  // src
            BUF_SIZE, // count
            false // trigger
        );
        //printf("1");
        DEBUG_PINS_CLR(spdif_rx_timing, 4);
    }
}

void spdif_rx_end()
{
    uint8_t sm = shared_state.pio_sm;
    pio_sm_unclaim(spdif_rx_pio, sm);
    pio_remove_program(spdif_rx_pio, &spdif_rx_program, loaded_offset);
    pio_clear_instruction_memory(spdif_rx_pio);
    uint8_t dma_channel0 = shared_state.dma_channel0;
    uint8_t dma_channel1 = shared_state.dma_channel1;
    dma_channel_unclaim(dma_channel0);
    dma_channel_unclaim(dma_channel1);
    irq_remove_handler(DMA_IRQ_x, spdif_rx_dma_irq_handler);
    dma_channel_set_irqx_enabled(dma_channel0, 0);
    dma_channel_set_irqx_enabled(dma_channel1, 0);
}

void spdif_rx_setup(const spdif_rx_config_t *config)
{
    uint8_t sm = shared_state.pio_sm = config->pio_sm;
    pio_sm_claim(spdif_rx_pio, sm);
    pio_sm_set_clkdiv(spdif_rx_pio, shared_state.pio_sm, 1);
    loaded_offset = pio_add_program(spdif_rx_pio, &spdif_rx_program);

    __mem_fence_release();
    uint8_t dma_channel0 = config->dma_channel0;
    uint8_t dma_channel1 = config->dma_channel1;
    dma_channel_claim(dma_channel0);
    dma_channel_claim(dma_channel1);
    shared_state.dma_channel0 = dma_channel0;
    shared_state.dma_channel1 = dma_channel1;

    dma_config0 = dma_channel_get_default_config(dma_channel0);
    channel_config_set_transfer_data_size(&dma_config0, DMA_SIZE_32);
    channel_config_set_read_increment(&dma_config0, false);
    channel_config_set_write_increment(&dma_config0, true);
    channel_config_set_dreq(&dma_config0, DREQ_PIOx_RX0 + sm);
    channel_config_set_chain_to(&dma_config0, dma_channel1);

    dma_channel_configure(
        dma_channel0,
        &dma_config0,
        buff[0], // dest
        &spdif_rx_pio->rxf[sm],  // src
        BUF_SIZE, // count
        false // trigger
    );

    dma_config1 = dma_channel_get_default_config(dma_channel1);
    channel_config_set_transfer_data_size(&dma_config1, DMA_SIZE_32);
    channel_config_set_read_increment(&dma_config1, false);
    channel_config_set_write_increment(&dma_config1, true);
    channel_config_set_dreq(&dma_config1, DREQ_PIOx_RX0 + sm);
    //channel_config_set_chain_to(&dma_config1, dma_channel0);

    dma_channel_configure(
        dma_channel1,
        &dma_config1,
        buff[1], // dest
        &spdif_rx_pio->rxf[sm],  // src
        BUF_SIZE, // count
        false // trigger
    );

    irq_add_shared_handler(DMA_IRQ_x, spdif_rx_dma_irq_handler, PICO_SHARED_IRQ_HANDLER_DEFAULT_ORDER_PRIORITY);
    //irq_add_shared_handler(DMA_IRQ_x, spdif_rx_dma_irq_handler, 0xff); // highest
    dma_channel_set_irqx_enabled(dma_channel0, 1);
    dma_channel_set_irqx_enabled(dma_channel1, 1);
    irq_set_enabled(DMA_IRQ_x, 1);

    dma_channel_start(dma_channel0);
    spdif_rx_program_init(spdif_rx_pio, sm, loaded_offset, config->data_pin);
    printf("2 %d\n", _millis());
}


/*
#define DATA_SIZE (44100)
uint32_t data[DATA_SIZE];
*/
void spdif_rx_check()
{
    /*
    for (int i = 0; i < DATA_SIZE; i++) {
        data[i] = spdif_rx_program_get32(spdif_rx_pio, shared_state.pio_sm);
    }
    */
    printf("done\n");
    uint32_t* data = buff[0];
    for (int i = 0; i < BUF_SIZE; i++) {
        uint32_t left  = (data[i*2+0] >> 12) & 0xffff;
        uint32_t right = (data[i*2+1] >> 12) & 0xffff;
        printf("L = %04x, R = %04x\n", left, right);
    }
    while (true) {}
}
