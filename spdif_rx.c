/*------------------------------------------------------/
/ Copyright (c) 2023, Elehobica
/ Released under the BSD-2-Clause
/ refer to https://opensource.org/licenses/BSD-2-Clause
/------------------------------------------------------*/

#define PICO_SPDIF_RX_PIO 1
#define PICO_SPDIF_RX_DMA_IRQ 1

#include <stdio.h>
#include <string.h>
#include "spdif_rx.h"

#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/irq.h"
#include "hardware/sync.h"
#include "spdif_rx_48000.pio.h"
#include "spdif_rx_96000.pio.h"
#include "spdif_rx_192000.pio.h"

static spin_lock_t* spdif_rx_spin_lock;

uint32_t fifo_buff[SPDIF_RX_FIFO_SIZE];
static uint32_t buff_wr_pre_ptr = 0;
static uint32_t buff_wr_done_ptr = 0;
static uint32_t buff_rd_ptr = 0;
static uint32_t buff_count;

dma_channel_config dma_config0;
dma_channel_config dma_config1;

#define spdif_rx_pio __CONCAT(pio, PICO_SPDIF_RX_PIO)
#define DREQ_PIOx_RX0 __CONCAT(__CONCAT(DREQ_PIO, PICO_SPDIF_RX_PIO), _RX0)

#define DMA_IRQ_x __CONCAT(DMA_IRQ_, PICO_SPDIF_RX_DMA_IRQ)

#define SYNC_B 0b1111
#define SYNC_M 0b1011
#define SYNC_W 0b0111

#define SF_CRITERIA (0.01)
#define NUM_AVE (8)
#define NUM_STABLE_FREQ (16) // 1 ~ 31

static spdif_rx_config_t gcfg;

static audio_format_t audio_format = {
    .sample_freq = 44100, // default
    .pcm_format = AUDIO_PCM_FORMAT_S32,
    .channel_count = AUDIO_CHANNEL_STEREO
};

typedef struct {
    const pio_program_t *program;
    uint offset;
    uint entry_point;
    pio_sm_config (*get_default_config)(uint);
} spdif_rx_pio_program_t;

static spdif_rx_pio_program_t decode_sets[] = {
    {&spdif_rx_48000_program,      0, spdif_rx_48000_offset_entry_point,      spdif_rx_48000_program_get_default_config},
    {&spdif_rx_48000_inv_program,  0, spdif_rx_48000_inv_offset_entry_point,  spdif_rx_48000_inv_program_get_default_config},
    {&spdif_rx_96000_program,      0, spdif_rx_96000_offset_entry_point,      spdif_rx_96000_program_get_default_config},
    {&spdif_rx_96000_inv_program,  0, spdif_rx_96000_inv_offset_entry_point,  spdif_rx_96000_inv_program_get_default_config},
    {&spdif_rx_192000_program,     0, spdif_rx_192000_offset_entry_point,     spdif_rx_192000_program_get_default_config},
    {&spdif_rx_192000_inv_program, 0, spdif_rx_192000_inv_offset_entry_point, spdif_rx_192000_inv_program_get_default_config}
};
static const spdif_rx_samp_freq_t samp_freq_array[] = {
    SAMP_FREQ_44100,
    SAMP_FREQ_48000,
    SAMP_FREQ_88200,
    SAMP_FREQ_96000,
    SAMP_FREQ_176400,
    SAMP_FREQ_192000
};

static int pio_program_id = 0;
static int block_count;
static uint64_t prev_time;
static uint64_t block_interval[NUM_AVE];
static bool block_aligned;
static int block_align_count;
static float samp_freq_actual;
static spdif_rx_samp_freq_t samp_freq;
static uint32_t stable_freq_history;
static bool stable_freq_flg;
static uint trans_count = SPDIF_BLOCK_SIZE;
static uint32_t c_bits;
static uint32_t parity_err_count;

static inline uint64_t _micros(void)
{
	return to_us_since_boot(get_absolute_time());
}

static inline uint32_t _millis(void)
{
	return to_ms_since_boot(get_absolute_time());
}

static inline void spdif_rx_program_init(PIO pio, uint sm, uint offset, uint entry_point, pio_sm_config (*get_default_config)(uint), uint pin) {
    pio_sm_set_consecutive_pindirs(pio, sm, pin, 1, false);
    pio_gpio_init(pio, pin);
    gpio_pull_down(pin);

    pio_sm_config sm_config = get_default_config(offset);

    sm_config_set_jmp_pin(&sm_config, pin);
    sm_config_set_in_pins(&sm_config, pin); // PINCTRL_IN_BASE for wait
    sm_config_set_in_shift(&sm_config, true, false, 32); // shift_right, no autopush, 32bit

    pio_sm_init(pio, sm, offset, &sm_config);
    pio_sm_set_pins(pio, sm, 0); // clear pins

    // set y, OSR (use as config value)
    pio_sm_set_enabled(pio, sm, false);
    pio_sm_put_blocking(pio, sm, 0x0); // y = 0x0
    pio_sm_exec(pio, sm, pio_encode_pull(false, false));
    pio_sm_exec(pio, sm, pio_encode_out(pio_y, 32));
    pio_sm_put_blocking(pio, sm, 0x3); // osr = 0x3
    pio_sm_exec(pio, sm, pio_encode_pull(false, false)); // only pull to store to osr
    pio_sm_set_enabled(pio, sm, true);

    // fifo join needs to be done after pull/out
    sm_config_set_fifo_join(&sm_config, PIO_FIFO_JOIN_RX);

    pio_sm_exec(pio, sm, pio_encode_jmp(offset + entry_point));
}

static inline uint32_t spdif_rx_program_get32(PIO pio, uint sm) {
    // 32-bit read from the FIFO
    while (pio_sm_is_rx_fifo_empty(pio, sm)) {
        tight_loop_contents();
    }
    return (uint32_t) pio->rxf[sm];
}

static inline uint32_t ptr_inc(uint32_t ptr, uint32_t count)
{
    return (ptr + count) % (SPDIF_RX_FIFO_SIZE * 2);
}

static inline uint32_t* to_buff_ptr(uint32_t ptr)
{
    return fifo_buff + ptr % SPDIF_RX_FIFO_SIZE;
}

// default spdif_rx block callback function (you may override at external)
__attribute__((weak))
void spdif_rx_callback_func(uint32_t *buff, uint32_t sub_frame_count, uint32_t c_bits, bool parity_err)
{
    return;
}

static int checkBlock(uint32_t buff[SPDIF_BLOCK_SIZE])
{
    uint pos_syncB = 0;
    uint32_t block_parity_err_count = 0;

    for (int i = 0; i < SPDIF_BLOCK_SIZE; i++) {
        uint32_t sync = buff[i] & 0xf;
        if (sync == SYNC_B) {
            block_aligned = (i == 0);
            pos_syncB = i;
        }
        if (block_aligned) {
            if ((i % 2 == 0 && (sync != SYNC_B && sync != SYNC_M)) || (i % 2 != 0 && sync != SYNC_W)) {
                block_aligned = false;
                break;
            }
            if (!gcfg.full_check) {
                return true;
            }
            // VUCP handling
            // C bits (heading 32bit only)
            if (i % 2 == 0 && i >= 0 && i < 64) { // using even sub frame of heading 32 frames of each block
                uint32_t c_bit = ((buff[i] & (0x1<<30)) != 0x0) ? 0x1 : 0x0;
                c_bits = (c_bits & (~(0x1 << (i / 2)))) | (c_bit << (i / 2));
            }

            // Parity (27 bits of every sub frame)
            uint32_t v = buff[i] & 0x7FFFFFF0; // excluding P and sync
            v ^= v >> 1;
            v ^= v >> 2;
            v = (v & 0x11111111) * 0x11111111; // bithacks trick, faster than __builtin_parity()
            v = (v << (31-28)); // parity is now in bit 28, move to bit 31
            if ((v ^ buff[i]) & (1<<31)) {
                block_parity_err_count++;
            }
        }
    }
    parity_err_count += block_parity_err_count;
    // block align adjustment
    if (block_aligned) {
        trans_count = SPDIF_BLOCK_SIZE;
        if (spdif_rx_get_samp_freq() != SAMP_FREQ_NONE) {
            spdif_rx_callback_func(buff, trans_count, c_bits, block_parity_err_count > 0);
        }
    } else {
        if (pos_syncB != 0 && block_align_count == 0) {
            // dispose pos_syncB to align because fifo_buff[SPDIF_BLOCK_SIZE-1] was (SPDIF_BLOCK_SIZE - pos_syncB)'th sub frame
            // it takes 3 blocks to align because coming 2 transfers already issued
            trans_count = (pos_syncB > SPDIF_BLOCK_SIZE / 2) ? pos_syncB : SPDIF_BLOCK_SIZE / 2; // workaround: small number of trans_count can make DMA overun
            block_align_count = 3;
        } else {
            trans_count = SPDIF_BLOCK_SIZE;
            if (block_align_count > 0) { block_align_count--; }
        }
    }

    return block_aligned;
}

static uint32_t dma_done_and_restart(uint8_t dma_channel, dma_channel_config* dma_config)
{
    uint32_t save = spin_lock_blocking(spdif_rx_spin_lock);
    uint32_t done_ptr = buff_wr_done_ptr;
    if (spdif_rx_get_status()) {
        if (spdif_rx_get_fifo_count() + SPDIF_BLOCK_SIZE > SPDIF_RX_FIFO_SIZE) {
            //printf("spdif_rx fifo overflow\n");
            buff_rd_ptr = ptr_inc(buff_rd_ptr, SPDIF_BLOCK_SIZE); // dispose overflow data
        }
        buff_wr_done_ptr = ptr_inc(done_ptr, SPDIF_BLOCK_SIZE);
    } else { // if status is not ready, fifo should be empty
        buff_wr_done_ptr = ptr_inc(done_ptr, SPDIF_BLOCK_SIZE);
        buff_rd_ptr = buff_wr_done_ptr;
    }
    dma_channel_configure(
        dma_channel,
        dma_config,
        to_buff_ptr(buff_wr_pre_ptr), // dest
        &spdif_rx_pio->rxf[gcfg.pio_sm],  // src
        trans_count, // count
        false // trigger
    );
    buff_wr_pre_ptr = ptr_inc(buff_wr_pre_ptr, SPDIF_BLOCK_SIZE);
    spin_unlock(spdif_rx_spin_lock, save);
    return done_ptr;
}

// irq handler for DMA
void __isr __time_critical_func(spdif_rx_dma_irq_handler)() {
    bool proc_dma0 = false;
    bool proc_dma1 = false;
    uint64_t now = _micros();
    if (dma_irqn_get_channel_status(PICO_SPDIF_RX_DMA_IRQ, gcfg.dma_channel0)) {
        dma_irqn_acknowledge_channel(PICO_SPDIF_RX_DMA_IRQ, gcfg.dma_channel0);
        proc_dma0 = true;
    } else if (dma_irqn_get_channel_status(PICO_SPDIF_RX_DMA_IRQ, gcfg.dma_channel1)) {
        dma_irqn_acknowledge_channel(PICO_SPDIF_RX_DMA_IRQ, gcfg.dma_channel1);
        proc_dma1 = true;
    }
    { // Calculate samp_freq and check if it's stable
        block_interval[block_count % NUM_AVE] = now - prev_time;
        uint64_t accum = 0;
        for (int i = 0; i < NUM_AVE; i++) {
            accum += block_interval[i];
        }
        float ave_block_interval = (float) accum / NUM_AVE;
        float bitrate_16b = (float) SPDIF_BLOCK_SIZE * 2 * 8 * 1e6 / ave_block_interval;
        samp_freq_actual = bitrate_16b / 32.0;
        spdif_rx_samp_freq_t sf = SAMP_FREQ_NONE;
        for (int i = 0; i < sizeof(samp_freq_array) / sizeof(spdif_rx_samp_freq_t); i++) {
            if (samp_freq_actual >= (float) samp_freq_array[i] * (1.0 - SF_CRITERIA) && samp_freq_actual < (float) samp_freq_array[i] * (1.0 + SF_CRITERIA)) {
                sf = samp_freq_array[i];
                break;
            }
        }
        stable_freq_history = (stable_freq_history << 1ul) | (sf != SAMP_FREQ_NONE && sf == samp_freq);
        stable_freq_flg = (stable_freq_history & ~(1ul<<NUM_STABLE_FREQ)) == ~(1ul<<NUM_STABLE_FREQ);
        samp_freq = sf;
    }
    block_count++;

    if (proc_dma0) {
        uint32_t done_ptr = dma_done_and_restart(gcfg.dma_channel0, &dma_config0);
        checkBlock(to_buff_ptr(done_ptr));
    } else if (proc_dma1) {
        uint32_t done_ptr = dma_done_and_restart(gcfg.dma_channel1, &dma_config1);
        checkBlock(to_buff_ptr(done_ptr));
    }
    prev_time = now;
}

void spdif_rx_setup(const spdif_rx_config_t *config)
{
    //pio_program_id = 0;
    buff_wr_pre_ptr = 0;
    buff_wr_done_ptr = 0;
    buff_rd_ptr = 0;
    block_count = 0;
    block_aligned = false;
    block_align_count = 0;
    stable_freq_history = 0;
    stable_freq_flg = false;
    trans_count = SPDIF_BLOCK_SIZE;
    c_bits = 0;
    parity_err_count = 0;

    spdif_rx_spin_lock = spin_lock_init(SPINLOCK_ID_AUDIO_FREE_LIST_LOCK);

    if (config != NULL) {
        memmove(&gcfg, config, sizeof(spdif_rx_config_t)); // copy to gcfg
    }
    // === PIO configuration ===
    pio_sm_claim(spdif_rx_pio, gcfg.pio_sm);
    pio_sm_set_clkdiv(spdif_rx_pio, gcfg.pio_sm, 1);

    // === DMA configuration ===
    __mem_fence_release();
    dma_channel_claim(gcfg.dma_channel0);
    dma_channel_claim(gcfg.dma_channel1);

    // DMA0
    dma_config0 = dma_channel_get_default_config(gcfg.dma_channel0);
    channel_config_set_transfer_data_size(&dma_config0, DMA_SIZE_32);
    channel_config_set_read_increment(&dma_config0, false);
    channel_config_set_write_increment(&dma_config0, true);
    channel_config_set_dreq(&dma_config0, DREQ_PIOx_RX0 + gcfg.pio_sm);
    channel_config_set_chain_to(&dma_config0, gcfg.dma_channel1);

    dma_channel_configure(
        gcfg.dma_channel0,
        &dma_config0,
        to_buff_ptr(buff_wr_pre_ptr), // dest
        &spdif_rx_pio->rxf[gcfg.pio_sm],  // src
        SPDIF_BLOCK_SIZE, // count
        false // trigger
    );
    buff_wr_pre_ptr = ptr_inc(buff_wr_pre_ptr, SPDIF_BLOCK_SIZE);

    // DMA1
    dma_config1 = dma_channel_get_default_config(gcfg.dma_channel1);
    channel_config_set_transfer_data_size(&dma_config1, DMA_SIZE_32);
    channel_config_set_read_increment(&dma_config1, false);
    channel_config_set_write_increment(&dma_config1, true);
    channel_config_set_dreq(&dma_config1, DREQ_PIOx_RX0 + gcfg.pio_sm);
    channel_config_set_chain_to(&dma_config1, gcfg.dma_channel0);

    dma_channel_configure(
        gcfg.dma_channel1,
        &dma_config1,
        to_buff_ptr(buff_wr_pre_ptr), // dest
        &spdif_rx_pio->rxf[gcfg.pio_sm],  // src
        SPDIF_BLOCK_SIZE, // count
        false // trigger
    );
    buff_wr_pre_ptr = ptr_inc(buff_wr_pre_ptr, SPDIF_BLOCK_SIZE);

    // DMA IRQ
    irq_add_shared_handler(DMA_IRQ_x, spdif_rx_dma_irq_handler, PICO_SHARED_IRQ_HANDLER_DEFAULT_ORDER_PRIORITY);
    //irq_add_shared_handler(DMA_IRQ_x, spdif_rx_dma_irq_handler, 0xff); // highest
    dma_irqn_set_channel_enabled(PICO_SPDIF_RX_DMA_IRQ, gcfg.dma_channel0, true);
    dma_irqn_set_channel_enabled(PICO_SPDIF_RX_DMA_IRQ, gcfg.dma_channel1, true);
    irq_set_enabled(DMA_IRQ_x, true);

    // Start DMA
    dma_channel_start(gcfg.dma_channel0);

    // PIO start
    spdif_rx_pio_program_t decode_pg = decode_sets[pio_program_id];
    decode_pg.offset = pio_add_program(spdif_rx_pio, decode_pg.program);
    spdif_rx_program_init(
        spdif_rx_pio,
        gcfg.pio_sm,
        decode_pg.offset,
        decode_pg.entry_point,
        decode_pg.get_default_config,
        gcfg.data_pin
    );
}

void spdif_rx_end()
{
    dma_channel_abort(gcfg.dma_channel0);
    dma_channel_abort(gcfg.dma_channel1);
    pio_sm_drain_tx_fifo(spdif_rx_pio, gcfg.pio_sm);
    spdif_rx_pio_program_t decode_pg = decode_sets[pio_program_id];
    pio_remove_program(spdif_rx_pio, decode_pg.program, decode_pg.offset);
    pio_clear_instruction_memory(spdif_rx_pio);
    pio_sm_unclaim(spdif_rx_pio, gcfg.pio_sm);
    dma_channel_unclaim(gcfg.dma_channel0);
    dma_channel_unclaim(gcfg.dma_channel1);
    irq_remove_handler(DMA_IRQ_x, spdif_rx_dma_irq_handler);
    dma_irqn_set_channel_enabled(PICO_SPDIF_RX_DMA_IRQ, gcfg.dma_channel0, false);
    dma_irqn_set_channel_enabled(PICO_SPDIF_RX_DMA_IRQ, gcfg.dma_channel1, false);
}

void spdif_rx_search_next()
{
    spdif_rx_end();
    pio_program_id = (pio_program_id + 1) % (sizeof(decode_sets) / sizeof(spdif_rx_pio_program_t));
    spdif_rx_setup(NULL);
}

bool spdif_rx_get_status()
{
    uint64_t now = _micros();
    // false if not block_aligned or no IRQ in recent 10 ms
    return block_aligned && stable_freq_flg && (now <= prev_time + 10000);
}

float spdif_rx_get_samp_freq_actual()
{
    return samp_freq_actual;
}

spdif_rx_samp_freq_t spdif_rx_get_samp_freq()
{
    return samp_freq;
}

uint32_t spdif_rx_get_c_bits()
{
    return c_bits;
}

uint32_t spdif_rx_get_parity_err_count()
{
    return parity_err_count;
}

uint32_t spdif_rx_get_fifo_count()
{
    if (buff_wr_done_ptr >= buff_rd_ptr) {
        return buff_wr_done_ptr - buff_rd_ptr;
    } else {
        return buff_wr_done_ptr + SPDIF_RX_FIFO_SIZE*2 - buff_rd_ptr;
    }
}

uint32_t spdif_rx_read_fifo(uint32_t** buff, uint32_t req_count)
{
    uint32_t save = spin_lock_blocking(spdif_rx_spin_lock);
    uint32_t get_count = req_count;
    uint32_t fifo_count = spdif_rx_get_fifo_count();
    if (get_count > fifo_count) {
        get_count = fifo_count;
    }
    if ((buff_rd_ptr % SPDIF_RX_FIFO_SIZE) + get_count <= SPDIF_RX_FIFO_SIZE) { // cannot take due to the end of fifo_buff
        *buff = to_buff_ptr(buff_rd_ptr);
        buff_rd_ptr = ptr_inc(buff_rd_ptr, get_count);
    } else {
        get_count = SPDIF_RX_FIFO_SIZE - (buff_rd_ptr % SPDIF_RX_FIFO_SIZE);
        *buff = to_buff_ptr(buff_rd_ptr);
        buff_rd_ptr = ptr_inc(buff_rd_ptr, get_count);
    }
    spin_unlock(spdif_rx_spin_lock, save);
    return get_count;
}

uint32_t* spdif_rx_read_fifo_single()
{
    uint32_t save = spin_lock_blocking(spdif_rx_spin_lock);
    uint32_t* buff = to_buff_ptr(buff_rd_ptr);
    buff_rd_ptr = ptr_inc(buff_rd_ptr, 1);
    //printf("ptr = %d, buff_ptr = %d\n", buff_rd_ptr, (int) buff);
    spin_unlock(spdif_rx_spin_lock, save);
    return buff;
}