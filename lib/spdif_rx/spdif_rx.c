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

#define NUM_BUF_SIZE_BITS (10)
#define BUF_SIZE (1 << (NUM_BUF_SIZE_BITS - 2))
//static uint32_t buff[2][BUF_SIZE];
dma_channel_config dma_config0;
dma_channel_config dma_config1;
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

#define dma_intsx __CONCAT(dma_hw->ints, PICO_SPDIF_RX_DMA_IRQ)
#define dma_channel_set_irqx_enabled __CONCAT(__CONCAT(dma_channel_set_irq, PICO_SPDIF_RX_DMA_IRQ),_enabled)
#define DMA_IRQ_x __CONCAT(DMA_IRQ_, PICO_SPDIF_RX_DMA_IRQ)

static uint loaded_offset0 = 0;
static uint loaded_offset1 = 0;
spdif_rx_config_t shared_state;

static int count0 = 0;
static int count1 = 0;

// irq handler for DMA
void __isr __time_critical_func(spdif_rx_dma_irq_handler)() {
    uint dma_channel0 = shared_state.dma_channel0;
    uint dma_channel1 = shared_state.dma_channel1;
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
            BUF_SIZE, // count
            false // trigger
        );
        count0++;
        //printf("0");
        //spdif_rx_check(buff[0]);
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
            BUF_SIZE, // count
            false // trigger
        );
        count1++;
        //printf("1");
        //spdif_rx_check(buff[1]);
        DEBUG_PINS_CLR(spdif_rx_timing, 4);
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
    dma_channel_unclaim(dma_channel0);
    dma_channel_unclaim(dma_channel1);
    irq_remove_handler(DMA_IRQ_x, spdif_rx_dma_irq_handler);
    dma_channel_set_irqx_enabled(dma_channel0, 0);
    dma_channel_set_irqx_enabled(dma_channel1, 0);
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
    dma_channel_claim(dma_channel0);
    dma_channel_claim(dma_channel1);
    shared_state.dma_channel0 = dma_channel0;
    shared_state.dma_channel1 = dma_channel1;

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
        BUF_SIZE, // count
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
        BUF_SIZE, // count
        false // trigger
    );

    irq_add_shared_handler(DMA_IRQ_x, spdif_rx_dma_irq_handler, PICO_SHARED_IRQ_HANDLER_DEFAULT_ORDER_PRIORITY);
    //irq_add_shared_handler(DMA_IRQ_x, spdif_rx_dma_irq_handler, 0xff); // highest
    dma_channel_set_irqx_enabled(dma_channel0, 1);
    dma_channel_set_irqx_enabled(dma_channel1, 1);
    irq_set_enabled(DMA_IRQ_x, 1);

    dma_channel_start(dma_channel0);
    spdif_rx_post_program_init(spdif_rx_pio1, sm1, loaded_offset1);
    spdif_rx_program_init(spdif_rx_pio0, sm0, loaded_offset0, config->data_pin);
}

void spdif_rx_loop()
{
    uint32_t data = spdif_rx_program_get32(spdif_rx_pio1, shared_state.pio_sm1);
    printf("L = %04x, R = %04x\n", (data >> 16), data & 0xffff);
}

void spdif_rx_check(uint32_t buffer[])
{
    int errIndex = 0;
    for (int i = 0; i < BUF_SIZE*2; i++) {
        uint32_t data = buffer[i];
        for (int j = 0; j < 16; j++) {
            uint8_t out = data & 0x3;
            data = data >> 2;
            switch (out) {
                case 0x0:
                    //printf("0");
                    syncSymCount = 0;
                    dataCount++;
                    break;
                case 0x1:
                    //printf("1");
                    syncSymCount = 0;
                    dataCount++;
                    break;
                case 0x3:
                    if (prevOut == 0x3) {
                        //printf("s");
                        syncSymCount++;
                        dataCount = 0;
                    } else {
                        //printf("\ns");
                        if (dataCount != 28 && dataCount != 30) {
                            if (detectedSync) {
                                errIndex = i;
                                errorCount++;
                            }
                        } else if (dataCount == 28) {
                            blockCount++;
                            frameCount++;
                        } else {
                            frameCount++;
                        }
                        detectedSync = true;
                        syncSymCount = 1;
                    }
                    break;
                default:
                    syncSymCount = 0;
                    dataCount = 0;
                    //printf("FATAL ERR\n");
                    break;
            }
            prevOut = out;
            totalCount++;
        }
    }
    printf("errorCount = %d, frameCount = %d, blockCount = %d errIndex = %d\n", errorCount, frameCount, blockCount, errIndex);
}