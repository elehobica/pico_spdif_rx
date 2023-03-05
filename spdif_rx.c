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
#include "spdif_rx_detect.pio.h"
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

#define SYSTEM_CLK_FREQUENCY (125000000)
// sample words to detect is equivalent to 64*4+8 symbols (1 frame + sync) at 44.1 KHz @ 125 MHz clock
#define SPDIF_RX_DETECT_SIZE (((SYSTEM_CLK_FREQUENCY / SAMP_FREQ_44100 / 128 + 1) * (64 * 4 + 8) + 31) / 32)

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

typedef enum _spdif_rx_pio_program_id_t  {
    SPDIF_RX_PIO_PROG_48000 = 0,
    SPDIF_RX_PIO_PROG_48000_INV,
    SPDIF_RX_PIO_PROG_96000,
    SPDIF_RX_PIO_PROG_96000_INV,
    SPDIF_RX_PIO_PROG_192000,
    SPDIF_RX_PIO_PROG_192000_INV
} spdif_rx_pio_program_id_t;

typedef struct {
    const spdif_rx_pio_program_id_t id;
    const pio_program_t *program;
    uint offset;
    uint entry_point;
    pio_sm_config (*get_default_config)(uint);
} spdif_rx_pio_program_t;

static spdif_rx_pio_program_t decode_sets[] = {
    {SPDIF_RX_PIO_PROG_48000,      &spdif_rx_48000_program,      0, spdif_rx_48000_offset_entry_point,      spdif_rx_48000_program_get_default_config},
    {SPDIF_RX_PIO_PROG_48000_INV,  &spdif_rx_48000_inv_program,  0, spdif_rx_48000_inv_offset_entry_point,  spdif_rx_48000_inv_program_get_default_config},
    {SPDIF_RX_PIO_PROG_96000,      &spdif_rx_96000_program,      0, spdif_rx_96000_offset_entry_point,      spdif_rx_96000_program_get_default_config},
    {SPDIF_RX_PIO_PROG_96000_INV,  &spdif_rx_96000_inv_program,  0, spdif_rx_96000_inv_offset_entry_point,  spdif_rx_96000_inv_program_get_default_config},
    {SPDIF_RX_PIO_PROG_192000,     &spdif_rx_192000_program,     0, spdif_rx_192000_offset_entry_point,     spdif_rx_192000_program_get_default_config},
    {SPDIF_RX_PIO_PROG_192000_INV, &spdif_rx_192000_inv_program, 0, spdif_rx_192000_inv_offset_entry_point, spdif_rx_192000_inv_program_get_default_config}
};

static const spdif_rx_samp_freq_t samp_freq_array[] = {
    SAMP_FREQ_44100,
    SAMP_FREQ_48000,
    SAMP_FREQ_88200,
    SAMP_FREQ_96000,
    SAMP_FREQ_176400,
    SAMP_FREQ_192000
};

static bool setup_done = false;
static bool stable_done = false;
static bool stable_lost = false;
static int pio_program_id = 0;
static int block_count;
static uint64_t prev_time = 0;
static uint64_t prev_time32 = 0;
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
    sm_config_set_fifo_join(&sm_config, PIO_FIFO_JOIN_RX);
    sm_config_set_clkdiv(&sm_config, 1);

    pio_sm_init(pio, sm, offset, &sm_config);
    pio_sm_set_pins(pio, sm, 0); // clear pins

    // set y, OSR (use as fixed value)
    pio_sm_set_enabled(pio, sm, false);
    pio_sm_exec(pio, sm, pio_encode_set(pio_y, 0x3));
    pio_sm_exec(pio, sm, pio_encode_mov(pio_osr, pio_y)); // osr = 0x3
    pio_sm_exec(pio, sm, pio_encode_set(pio_y, 0x0)); // y = 0x0
    pio_sm_set_enabled(pio, sm, true);

    pio_sm_exec(pio, sm, pio_encode_jmp(offset + entry_point));
}

static inline uint32_t ptr_inc(uint32_t ptr, uint32_t count)
{
    return (ptr + count) % (SPDIF_RX_FIFO_SIZE * 2);
}

static inline uint32_t* to_buff_ptr(uint32_t ptr)
{
    return &fifo_buff[ptr % SPDIF_RX_FIFO_SIZE];
}

// default spdif_rx block callback function (you may override at external)
__attribute__((weak))
void spdif_rx_callback_func(uint32_t *buff, uint32_t sub_frame_count, uint32_t c_bits, bool parity_err)
{
    return;
}

static int check_block(uint32_t buff[SPDIF_BLOCK_SIZE])
{
    uint pos_syncB = 0;
    uint32_t block_parity_err_count = 0;
    uint32_t block_cbits = 0;
    uint32_t cbit_mask = 0x1; // start with LSB

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
            if (i % 2 == 0 && cbit_mask) { // using even sub frame of heading 32 frames of each block
                if (buff[i] & (0x1<<30)) {
                    block_cbits |= cbit_mask;
                }
                cbit_mask <<= 1; // will become 0 after 32 bits are collected, and stop the if()
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
        c_bits = block_cbits; // copy what's been collected
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

static int spdif_rx_get_pio_promgam_id(spdif_rx_samp_freq_t samp_freq, bool inverted)
{
    int pio_program_id = 0;
    switch (samp_freq) {
        case SAMP_FREQ_44100: // fallthrough
        case SAMP_FREQ_48000:
            pio_program_id = SPDIF_RX_PIO_PROG_48000;
            break;
        case SAMP_FREQ_88200: // fallthrough
        case SAMP_FREQ_96000:
            pio_program_id = SPDIF_RX_PIO_PROG_96000;
            break;
        case SAMP_FREQ_176400: // fallthrough
        case SAMP_FREQ_192000:
            pio_program_id = SPDIF_RX_PIO_PROG_192000;
            break;
        default:
            pio_program_id = SPDIF_RX_PIO_PROG_48000;
            break;
    }
    if (inverted) {
        pio_program_id++;
    }
    return pio_program_id;
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
        if (setup_done && stable_freq_flg && block_aligned) {
            stable_done = true;
            stable_lost = false;
        } else if (setup_done && stable_done && (!stable_freq_flg || !block_aligned)) {
            stable_lost = true;
        }
        samp_freq = sf;
    }
    block_count++;

    if (proc_dma0) {
        uint32_t done_ptr = dma_done_and_restart(gcfg.dma_channel0, &dma_config0);
        check_block(to_buff_ptr(done_ptr));
    } else if (proc_dma1) {
        uint32_t done_ptr = dma_done_and_restart(gcfg.dma_channel1, &dma_config1);
        check_block(to_buff_ptr(done_ptr));
    }
    prev_time = now;
    prev_time32 = _millis();
}

void spdif_rx_set_config(const spdif_rx_config_t *config)
{
    memmove(&gcfg, config, sizeof(spdif_rx_config_t)); // copy to gcfg
}

int spdif_rx_search()
{
    spdif_rx_samp_freq_t samp_freq;
    bool inverted;
    if (setup_done) {
        spdif_rx_end();
        setup_done = false;
    }
    if (spdif_rx_detect(&samp_freq, &inverted)) {
        spdif_rx_setup(samp_freq, inverted);
        setup_done = true;
        return 1;
    }
    return 0;
}

int spdif_rx_detect(spdif_rx_samp_freq_t *samp_freq, bool *inverted)
{
    // DMA0
    dma_channel_claim(gcfg.dma_channel0);
    dma_config0 = dma_channel_get_default_config(gcfg.dma_channel0);
    channel_config_set_transfer_data_size(&dma_config0, DMA_SIZE_32);
    channel_config_set_read_increment(&dma_config0, false);
    channel_config_set_write_increment(&dma_config0, true);
    channel_config_set_dreq(&dma_config0, DREQ_PIOx_RX0 + gcfg.pio_sm);
    channel_config_set_chain_to(&dma_config0, gcfg.dma_channel0);

    dma_channel_configure(
        gcfg.dma_channel0,
        &dma_config0,
        fifo_buff,
        &spdif_rx_pio->rxf[gcfg.pio_sm],  // src
        SPDIF_RX_DETECT_SIZE,
        false // trigger
    );
    dma_channel_start(gcfg.dma_channel0);

    // === PIO configuration ===
    pio_sm_claim(spdif_rx_pio, gcfg.pio_sm);
    uint offset = pio_add_program(spdif_rx_pio, &spdif_rx_detect_program);
    spdif_rx_detect_program_init(
        spdif_rx_pio,
        gcfg.pio_sm,
        offset,
        gcfg.data_pin
    );

    sleep_us(100); // give enough time to detect toggles and sample SPDIF_RX_DETECT_SIZE times

    bool timeout = false;
    if (dma_channel_is_busy(gcfg.dma_channel0)) {
        dma_channel_abort(gcfg.dma_channel0);
        timeout = true;
    }
    pio_sm_drain_tx_fifo(spdif_rx_pio, gcfg.pio_sm);
    pio_remove_program(spdif_rx_pio, &spdif_rx_detect_program, offset);
    pio_clear_instruction_memory(spdif_rx_pio);
    pio_sm_unclaim(spdif_rx_pio, gcfg.pio_sm);
    dma_channel_unclaim(gcfg.dma_channel0);

    if (timeout) {
        return 0;
    }

    // sampled data analysis to calculate min_width, max_width, max_edge_interval for both 0 and 1
    //for (int i = 0; i < SPDIF_RX_DETECT_SIZE; i++) printf("data[%3d] = %032b\n", i, fifo_buff[i]);
    int edge_interval[2] = {0, 0};
    int min_width[2] = {256, 256};
    //int max_width[2] = {0, 0}; // not needed for frequency detection
    int max_edge_interval[2] = {0, 0};
    int i = 0;
    int succ = 0;
    int cur = 1;
    bool end_flag = false;
    // bithacks for trailing zeros by de Bruijn sequences
    const int MultiplyDeBruijnBitPosition[32] = {
        0, 1, 28, 2, 29, 14, 24, 3, 30, 22, 20, 15, 25, 17, 4, 8,
        31, 27, 13, 23, 21, 19, 16, 7, 26, 12, 18, 6, 11, 5, 10, 9
    };
    while (i < 32 * SPDIF_RX_DETECT_SIZE) {
        int word_idx = i / 32;
        int bit_pos = i % 32;
        uint32_t v = (cur) ? ~fifo_buff[word_idx] : fifo_buff[word_idx];
        int r = (v) ? MultiplyDeBruijnBitPosition[((uint32_t) ((v & -v) * 0x077CB531U)) >> 27] : 32;
        if (r + bit_pos <= 31) { // within 32bit
            succ += r;
            int next = 1 - cur;
            edge_interval[cur] += succ;
            edge_interval[next] += succ;
            if (min_width[cur] > succ) min_width[cur] = succ;
            //if (max_width[cur] < succ) max_width[cur] = succ;
            if (max_edge_interval[next] < edge_interval[next]) max_edge_interval[next] = edge_interval[next];
            edge_interval[next] = 0; // this edge
            fifo_buff[word_idx] >>= r;
            i += r;
            succ = 0;
            cur = next;
        } else { // otherwise go first bit in next 32bit
            i += 32 - bit_pos;
            succ += 32 - bit_pos;
        }
    }
    //printf("min0 = %d, max_edge0 = %d, min1 = %d, max_edge1 = %d\n", min_width[0], max_edge_interval[0], min_width[1], max_edge_interval[1]);

    // evaluate analysis result to confirm if it meets criteria of sampling frequencies
    const int SC = 1; // sample criteria
    for (int i = 0; i < sizeof(samp_freq_array) / sizeof(spdif_rx_samp_freq_t); i++) {
        int min_exp = SYSTEM_CLK_FREQUENCY / samp_freq_array[i] / 128; // symbol cycle
        // Judge polarity: thanks to great idea by IDC-Dragon
        // focusing on Sync Code M which appears in L sub-frame (except for head frame in the block)
        //                                                                    |<--------->|
        // if max interval of rising edge = 6,  then polarity is normal   (0 -> 1 1 1 0 0 0 1 0 -> 1)
        // if max interval of falling edge = 6, then polarity is inverted (1 -> 0 0 0 1 1 1 0 1 -> 0)
        int max_edge_interval_exp = SYSTEM_CLK_FREQUENCY / samp_freq_array[i] * 6 / 128; // symbol cycle
        if ((min_width[0] >= min_exp - SC && min_width[0] <= min_exp + SC) && (min_width[1] >= min_exp - SC && min_width[1] <= min_exp + SC)) {
            if (max_edge_interval[1] >= max_edge_interval_exp - SC && max_edge_interval[1] <= max_edge_interval_exp + SC) { // normal bit stream
                *samp_freq = samp_freq_array[i];
                *inverted = false;
                return 1;
            } else if (max_edge_interval[0] >= max_edge_interval_exp - SC && max_edge_interval[0] <= max_edge_interval_exp + SC) { // inverted bit stream
                *samp_freq = samp_freq_array[i];
                *inverted = true;
                return 1;
            }
        }
    }

    return 0;
}

void spdif_rx_setup(spdif_rx_samp_freq_t samp_freq, bool inverted)
{
    pio_program_id = spdif_rx_get_pio_promgam_id(samp_freq, inverted);
    stable_done = false;
    stable_lost = false;
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

    // === PIO configuration ===
    pio_sm_claim(spdif_rx_pio, gcfg.pio_sm);
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

spdif_rx_status_t spdif_rx_get_status()
{
    uint64_t now = _millis();
    if (setup_done && !stable_done) {
        return SPDIF_RX_STATUS_WAITING_STABLE;
    } else if (setup_done && stable_done && !stable_lost && now <= prev_time32 + 10) { // exclude stable when no IRQ in recent 10 ms
        return SPDIF_RX_STATUS_STABLE;
    } else {
        return SPDIF_RX_STATUS_NO_SIGNAL;
    }
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