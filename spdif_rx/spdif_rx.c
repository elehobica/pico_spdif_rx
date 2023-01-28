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

#define BUF_SIZE (384)
uint32_t buff[2][BUF_SIZE];
dma_channel_config dma_config0;
dma_channel_config dma_config1;

// ---- select at most one ---
CU_REGISTER_DEBUG_PINS(spdif_rx_timing)

#define spdif_rx_pio __CONCAT(pio, PICO_SPDIF_RX_PIO)
#define DREQ_PIOx_RX0 __CONCAT(__CONCAT(DREQ_PIO, PICO_SPDIF_RX_PIO), _RX0)

#define dma_intsx __CONCAT(dma_hw->ints, PICO_SPDIF_RX_DMA_IRQ)
#define dma_channel_set_irqx_enabled __CONCAT(__CONCAT(dma_channel_set_irq, PICO_SPDIF_RX_DMA_IRQ),_enabled)
#define DMA_IRQ_x __CONCAT(DMA_IRQ_, PICO_SPDIF_RX_DMA_IRQ)

static uint loaded_offset = 0;
spdif_rx_config_t shared_state;

static int block_count = 0;
static int prev_block_count = 0;
static uint64_t prev_time = 0;
static uint64_t block_interval[10];
static float ave_block_interval;
static int  prev_pos_syncB = 0;
static int sync_ok = 0;
#define SYNC_B 0b1111
#define SYNC_M 0b1011
#define SYNC_W 0b0111

static inline uint64_t _micros(void)
{
	return to_us_since_boot(get_absolute_time());
}

static inline uint32_t _millis(void)
{
	return to_ms_since_boot(get_absolute_time());
}

static void _spdif_rx_check()
{
    printf("done\n");
    uint32_t* data = buff[0];
    for (int i = 0; i < BUF_SIZE; i++) {
        uint32_t left  = (data[i*2+0] >> 12) & 0xffff;
        uint32_t right = (data[i*2+1] >> 12) & 0xffff;
        printf("L = %04x, R = %04x\n", left, right);
    }
    while (true) {}
}

static void _spdif_rx_direct_check()
{
    printf("done\n");
    uint32_t* data = buff[0];
    for (int i = 0; i < BUF_SIZE*2; i++) {
        printf("%08x\n", data[i]);
    }
    while (true) {}
}

static int _syncCheck(uint32_t b[BUF_SIZE])
{
    for (int i = 0; i < BUF_SIZE; i++) {
        uint32_t sync = b[i] & 0xf;
        if (sync == SYNC_B) {
            if (sync_ok) {
                if (prev_pos_syncB != i) {
                    sync_ok = 0;
                    break;
                }
            } else {
                sync_ok = 1;
                prev_pos_syncB = i;
            }
        //} else if (sync_ok && ((i % 2 == prev_pos_syncB % 2 && sync != SYNC_M) || (i % 2 != prev_pos_syncB % 2 && sync != SYNC_W))) {
        } else if (sync_ok) {
            if ((i % 2 == prev_pos_syncB % 2 && sync != SYNC_M) || (i % 2 != prev_pos_syncB % 2 && sync != SYNC_W)) {
                sync_ok = false;
                break;
            }
        }
    }
    return sync_ok;
}

// irq handler for DMA
void __isr __time_critical_func(spdif_rx_dma_irq_handler)() {
    uint dma_channel0 = shared_state.dma_channel0;
    uint dma_channel1 = shared_state.dma_channel1;
    uint8_t sm = shared_state.pio_sm;
    uint64_t now = _micros();
    block_interval[block_count % 10] = now - prev_time;
    uint64_t accum = 0;
    for (int i = 0; i < 10; i++) {
        accum += block_interval[i];
    }
    ave_block_interval = (float) accum / 10;
    block_count++;

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
        _syncCheck(buff[0]);
        //printf("0");
        DEBUG_PINS_CLR(spdif_rx_timing, 4);
    }
    if ((dma_intsx & (1u << dma_channel1))) {
        //_spdif_rx_check();
        //_spdif_rx_direct_check();
        dma_intsx = 1u << dma_channel1;
        DEBUG_PINS_SET(spdif_rx_timing, 4);
        dma_channel_configure(
            dma_channel1,
            &dma_config1,
            buff[1], // dest
            &spdif_rx_pio->rxf[sm],  // src
            BUF_SIZE, // count
            false // trigger
        );
        _syncCheck(buff[1]);
        //printf("1");
        DEBUG_PINS_CLR(spdif_rx_timing, 4);
    }
    prev_time = now;
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
    channel_config_set_chain_to(&dma_config1, dma_channel0);

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
}

int spdif_rx_status()
{
    int flag = (sync_ok && block_count != prev_block_count);
    prev_block_count = block_count;
    return flag;
}

uint32_t spdif_rx_get_samp_freq()
{
    float bitrate = (float) BUF_SIZE * 2 * 8 * 1e6 / ave_block_interval;
    float samp_freq = bitrate / 32.0;
    if (samp_freq >= (float) (SAMP_FREQ_44100 - 100) && samp_freq < (float) (SAMP_FREQ_44100 + 100)) {
        return SAMP_FREQ_44100;
    } else if (samp_freq >= (float) (SAMP_FREQ_48000) - 100 && samp_freq < (float) (SAMP_FREQ_48000 + 100)) {
        return SAMP_FREQ_48000;
    } else {
        return 0;
    }
}
