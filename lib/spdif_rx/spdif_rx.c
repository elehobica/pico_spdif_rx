/*------------------------------------------------------/
/ Copyright (c) 2023, Elehobica
/ Released under the BSD-2-Clause
/ refer to https://opensource.org/licenses/BSD-2-Clause
/------------------------------------------------------*/

#define PICO_SPDIF_RX_PIO0 0
#define PICO_SPDIF_RX_PIO1 1
#define PICO_SPDIF_RX_DMA_IRQ 0

#include <stdio.h>
#include "spdif_rx.h"

#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/irq.h"
#include "spdif_rx.pio.h"

#define TRANSFER_COUNTS (512)
#define BUF_SIZE (128)
uint32_t buff[BUF_SIZE];
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

#define spdif_rx_pio0 __CONCAT(pio, PICO_SPDIF_RX_PIO0)
#define spdif_rx_pio1 __CONCAT(pio, PICO_SPDIF_RX_PIO1)
#define DREQ_PIOx_RX0 __CONCAT(__CONCAT(DREQ_PIO, PICO_SPDIF_RX_PIO0), _RX0)
#define DREQ_PIOx_RX1 __CONCAT(__CONCAT(DREQ_PIO, PICO_SPDIF_RX_PIO1), _RX0)

#define dma_intsx __CONCAT(dma_hw->ints, PICO_SPDIF_RX_DMA_IRQ)
#define dma_channel_set_irqx_enabled __CONCAT(__CONCAT(dma_channel_set_irq, PICO_SPDIF_RX_DMA_IRQ),_enabled)
#define DMA_IRQ_x __CONCAT(DMA_IRQ_, PICO_SPDIF_RX_DMA_IRQ)

static uint loaded_offset0 = 0;
static uint loaded_offset1 = 0;
spdif_rx_config_t shared_state;

static inline uint32_t _millis(void)
{
	return to_ms_since_boot(get_absolute_time());
}

// irq handler for DMA
void __isr __time_critical_func(spdif_rx_dma_irq_handler)() {
    uint dma_channel0 = shared_state.dma_channel0;
    uint dma_channel1 = shared_state.dma_channel1;
    uint dma_channel2 = shared_state.dma_channel2;
    uint8_t sm0 = shared_state.pio_sm0;
    uint8_t sm1 = shared_state.pio_sm1;

    if ((dma_intsx & (1u << dma_channel0))) {
        dma_intsx = 1u << dma_channel0;
        DEBUG_PINS_SET(spdif_rx_timing, 4);
        dma_channel_configure(
            dma_channel0,
            &dma_config0,
            &spdif_rx_pio1->txf[sm1],  // dest
            &spdif_rx_pio0->rxf[sm0],  // src
            TRANSFER_COUNTS, // count
            false // trigger
        );
        //printf("0");
        //printf("1 %d\n", _millis());
        DEBUG_PINS_CLR(spdif_rx_timing, 4);
    }
    if ((dma_intsx & (1u << dma_channel1))) {
        dma_intsx = 1u << dma_channel1;
        DEBUG_PINS_SET(spdif_rx_timing, 4);
        dma_channel_configure(
            dma_channel1,
            &dma_config1,
            &spdif_rx_pio1->txf[sm1],  // dest
            &spdif_rx_pio0->rxf[sm0],  // src
            TRANSFER_COUNTS, // count
            false // trigger
        );
        //printf("1");
        DEBUG_PINS_CLR(spdif_rx_timing, 4);
    }
    if ((dma_intsx & (1u << dma_channel2))) {
        dma_intsx = 1u << dma_channel2;
        //printf("2");
        printf("2 %d\n", _millis());
        dma_channel_transfer_to_buffer_now(dma_channel2, buff, BUF_SIZE);
    }
}

void spdif_rx_end()
{
    uint8_t sm0 = shared_state.pio_sm0;
    uint8_t sm1 = shared_state.pio_sm1;
    pio_sm_unclaim(spdif_rx_pio0, sm0);
    pio_sm_unclaim(spdif_rx_pio1, sm1);
    pio_remove_program(spdif_rx_pio0, &spdif_rx_program, loaded_offset0);
    pio_remove_program(spdif_rx_pio1, &spdif_rx_post_program, loaded_offset1);
    pio_clear_instruction_memory(spdif_rx_pio0);
    pio_clear_instruction_memory(spdif_rx_pio1);
    uint8_t dma_channel0 = shared_state.dma_channel0;
    uint8_t dma_channel1 = shared_state.dma_channel1;
    uint8_t dma_channel2 = shared_state.dma_channel2;
    dma_channel_unclaim(dma_channel0);
    dma_channel_unclaim(dma_channel1);
    dma_channel_unclaim(dma_channel2);
    irq_remove_handler(DMA_IRQ_x, spdif_rx_dma_irq_handler);
    dma_channel_set_irqx_enabled(dma_channel0, 0);
    dma_channel_set_irqx_enabled(dma_channel1, 0);
    dma_channel_set_irqx_enabled(dma_channel2, 0);
}

void spdif_rx_setup(const spdif_rx_config_t *config)
{
    uint8_t sm0 = shared_state.pio_sm0 = config->pio_sm0;
    uint8_t sm1 = shared_state.pio_sm1 = config->pio_sm1;
    pio_sm_claim(spdif_rx_pio0, sm0);
    pio_sm_claim(spdif_rx_pio1, sm1);
    pio_sm_set_clkdiv(spdif_rx_pio0, shared_state.pio_sm0, 1);
    pio_sm_set_clkdiv(spdif_rx_pio1, shared_state.pio_sm1, 17);
    loaded_offset0 = pio_add_program(spdif_rx_pio0, &spdif_rx_program);
    loaded_offset1 = pio_add_program(spdif_rx_pio1, &spdif_rx_post_program);

    __mem_fence_release();
    uint8_t dma_channel0 = config->dma_channel0;
    uint8_t dma_channel1 = config->dma_channel1;
    uint8_t dma_channel2 = config->dma_channel2;
    dma_channel_claim(dma_channel0);
    dma_channel_claim(dma_channel1);
    dma_channel_claim(dma_channel2);
    shared_state.dma_channel0 = dma_channel0;
    shared_state.dma_channel1 = dma_channel1;
    shared_state.dma_channel2 = dma_channel2;

    dma_config0 = dma_channel_get_default_config(dma_channel0);
    channel_config_set_transfer_data_size(&dma_config0, DMA_SIZE_32);
    channel_config_set_read_increment(&dma_config0, false);
    channel_config_set_write_increment(&dma_config0, false);
    channel_config_set_dreq(&dma_config0, DREQ_PIOx_RX0 + sm0);
    channel_config_set_chain_to(&dma_config0, dma_channel1);

    dma_channel_configure(
        dma_channel0,
        &dma_config0,
        &spdif_rx_pio1->txf[sm1],  // dest
        &spdif_rx_pio0->rxf[sm0],  // src
        TRANSFER_COUNTS, // count
        false // trigger
    );

    dma_config1 = dma_channel_get_default_config(dma_channel1);
    channel_config_set_transfer_data_size(&dma_config1, DMA_SIZE_32);
    channel_config_set_read_increment(&dma_config1, false);
    channel_config_set_write_increment(&dma_config1, false);
    channel_config_set_dreq(&dma_config1, DREQ_PIOx_RX0 + sm0);
    channel_config_set_chain_to(&dma_config1, dma_channel0);

    dma_channel_configure(
        dma_channel1,
        &dma_config1,
        &spdif_rx_pio1->txf[sm1],  // dest
        &spdif_rx_pio0->rxf[sm0],  // src
        TRANSFER_COUNTS, // count
        false // trigger
    );

    dma_config2 = dma_channel_get_default_config(dma_channel2);
    channel_config_set_transfer_data_size(&dma_config2, DMA_SIZE_32);
    channel_config_set_read_increment(&dma_config2, false);
    channel_config_set_write_increment(&dma_config2, true);
    channel_config_set_dreq(&dma_config2, DREQ_PIOx_RX1 + sm1);

    dma_channel_configure(
        dma_channel2,
        &dma_config2,
        buff,
        &spdif_rx_pio1->rxf[sm1],  // src
        BUF_SIZE, // count
        false // trigger
    );

    irq_add_shared_handler(DMA_IRQ_x, spdif_rx_dma_irq_handler, PICO_SHARED_IRQ_HANDLER_DEFAULT_ORDER_PRIORITY);
    //irq_add_shared_handler(DMA_IRQ_x, spdif_rx_dma_irq_handler, 0xff); // highest
    dma_channel_set_irqx_enabled(dma_channel0, 1);
    dma_channel_set_irqx_enabled(dma_channel1, 1);
    dma_channel_set_irqx_enabled(dma_channel2, 1);
    irq_set_enabled(DMA_IRQ_x, 1);

    dma_channel_transfer_to_buffer_now(dma_channel2, buff, BUF_SIZE);
    //dma_channel_start(dma_channel2);
    dma_channel_start(dma_channel0);
    spdif_rx_post_program_init(spdif_rx_pio1, sm1, loaded_offset1);
    spdif_rx_program_init(spdif_rx_pio0, sm0, loaded_offset0, config->data_pin);
    printf("2 %d\n", _millis());
}

#define CHK_BUF_SIZE (44100 * 1)
uint32_t data[CHK_BUF_SIZE];
void spdif_rx_check()
{
    /*
    for (int i = 0; i < CHK_BUF_SIZE; i++) {
        data[i] = spdif_rx_program_get32(spdif_rx_pio1, shared_state.pio_sm1);
    }
    printf("done\n");
    */
    /*
    for (int i = 0; i < CHK_BUF_SIZE; i++) {
        printf("L = %04x, R = %04x\n", (data[i] >> 16), data[i] & 0xffff);
    }
    */
    while (true) {}
}
