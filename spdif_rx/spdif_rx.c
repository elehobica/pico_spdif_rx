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
#include "spdif_rx.pio.h"

static spin_lock_t* spdif_rx_spin_lock;

#define BLOCK_SIZE (384) // sub frames per block
#define NUM_BLOCKS (8) // must be >= 2
#define FIFO_SIZE (NUM_BLOCKS * BLOCK_SIZE)
uint32_t fifo_buff[FIFO_SIZE];
static uint32_t buff_wr_pre_ptr = 0;
static uint32_t buff_wr_done_ptr = 0;
static uint32_t buff_rd_ptr = 0;
static uint32_t buff_count;

dma_channel_config dma_config0;
dma_channel_config dma_config1;

#define spdif_rx_pio __CONCAT(pio, PICO_SPDIF_RX_PIO)
#define DREQ_PIOx_RX0 __CONCAT(__CONCAT(DREQ_PIO, PICO_SPDIF_RX_PIO), _RX0)

#define dma_intsx __CONCAT(dma_hw->ints, PICO_SPDIF_RX_DMA_IRQ)
#define dma_channel_set_irqx_enabled __CONCAT(__CONCAT(dma_channel_set_irq, PICO_SPDIF_RX_DMA_IRQ),_enabled)
#define DMA_IRQ_x __CONCAT(DMA_IRQ_, PICO_SPDIF_RX_DMA_IRQ)

#define SYNC_B 0b1111
#define SYNC_M 0b1011
#define SYNC_W 0b0111

static spdif_rx_config_t gcfg;

static audio_format_t audio_format = {
    .sample_freq = 44100, // default
    .pcm_format = AUDIO_PCM_FORMAT_S32,
    .channel_count = AUDIO_CHANNEL_STEREO
};

static audio_buffer_format_t producer_format = {
    .format = &audio_format,
    .sample_stride = 8
};
static audio_buffer_pool_t *producer_pool = NULL;
bool use_audio_buffer = false;

typedef struct {
    const pio_program_t *program;
    uint offset;
    uint entry_point;
    pio_sm_config (*get_default_config)(uint);
} spdif_rx_pio_program_t;

static spdif_rx_pio_program_t decode_sets[] = {
    {&spdif_rx_program,     0, spdif_rx_offset_entry_point,     spdif_rx_program_get_default_config},
    {&spdif_rx_inv_program, 0, spdif_rx_inv_offset_entry_point, spdif_rx_inv_program_get_default_config}
};

static int program_id = 0;

static int block_count = 0;
static int prev_block_count = 0;
static uint64_t prev_time = 0;
static uint64_t block_interval[10];
static float ave_block_interval;
static bool block_aligned = false;
static int block_align_count = 0;
static uint trans_count = BLOCK_SIZE;
static uint32_t c_bits;
static uint32_t parity_err_count = 0;

const char count_ones8[256] =
	"\x00\x01\x01\x02\x01\x02\x02\x03\x01\x02\x02\x03\x02\x03\x03\x04"
	"\x01\x02\x02\x03\x02\x03\x03\x04\x02\x03\x03\x04\x03\x04\x04\x05"
	"\x01\x02\x02\x03\x02\x03\x03\x04\x02\x03\x03\x04\x03\x04\x04\x05"
	"\x02\x03\x03\x04\x03\x04\x04\x05\x03\x04\x04\x05\x04\x05\x05\x06"
	"\x01\x02\x02\x03\x02\x03\x03\x04\x02\x03\x03\x04\x03\x04\x04\x05"
	"\x02\x03\x03\x04\x03\x04\x04\x05\x03\x04\x04\x05\x04\x05\x05\x06"
	"\x02\x03\x03\x04\x03\x04\x04\x05\x03\x04\x04\x05\x04\x05\x05\x06"
	"\x03\x04\x04\x05\x04\x05\x05\x06\x04\x05\x05\x06\x05\x06\x06\x07"
	"\x01\x02\x02\x03\x02\x03\x03\x04\x02\x03\x03\x04\x03\x04\x04\x05"
	"\x02\x03\x03\x04\x03\x04\x04\x05\x03\x04\x04\x05\x04\x05\x05\x06"
	"\x02\x03\x03\x04\x03\x04\x04\x05\x03\x04\x04\x05\x04\x05\x05\x06"
	"\x03\x04\x04\x05\x04\x05\x05\x06\x04\x05\x05\x06\x05\x06\x06\x07"
	"\x02\x03\x03\x04\x03\x04\x04\x05\x03\x04\x04\x05\x04\x05\x05\x06"
	"\x03\x04\x04\x05\x04\x05\x05\x06\x04\x05\x05\x06\x05\x06\x06\x07"
	"\x03\x04\x04\x05\x04\x05\x05\x06\x04\x05\x05\x06\x05\x06\x06\x07"
	"\x04\x05\x05\x06\x05\x06\x06\x07\x05\x06\x06\x07\x06\x07\x07\x08";

static inline uint64_t _micros(void)
{
	return to_us_since_boot(get_absolute_time());
}

static inline uint32_t _millis(void)
{
	return to_ms_since_boot(get_absolute_time());
}

static uint32_t ptr_inc(uint32_t ptr, uint32_t count)
{
    return (ptr + count) % (FIFO_SIZE * 2);
}

static uint32_t* to_buff_ptr(uint32_t ptr)
{
    return fifo_buff + ptr % FIFO_SIZE;
}

static void _spdif_rx_check()
{
    printf("done\n");
    uint32_t* data = &fifo_buff[0];
    for (int i = 0; i < BLOCK_SIZE; i++) {
        uint32_t left  = (data[i*2+0] >> 12) & 0xffff;
        uint32_t right = (data[i*2+1] >> 12) & 0xffff;
        printf("L = %04x, R = %04x\n", left, right);
    }
    while (true) {}
}

static void _spdif_rx_direct_check()
{
    printf("done\n");
    uint32_t* data = &fifo_buff[0];
    for (int i = 0; i < BLOCK_SIZE*2; i++) {
        printf("%08x\n", data[i]);
    }
    while (true) {}
}

// default spdif_rx block callback function (you may override at external)
__attribute__((weak))
void spdif_rx_callback_func(uint32_t *buff, uint32_t sub_frame_count, uint32_t c_bits, bool parity_err)
{
    return;
    static int count = 0;
    if (producer_pool == NULL) { return; }
    //audio_format.sample_freq = (uint32_t) (spdif_rx_get_samp_freq_actual() + 0.5);
    //audio_format.sample_freq = 44105;
    //pio_sm_set_clkdiv_int_frac(pio0, 0, 22, 36 + (count % 10)); // This scheme includes clock Jitter
    audio_buffer_t *buffer;
    if ((buffer = take_audio_buffer(producer_pool, false)) == NULL) { return; }

    #ifdef DEBUG_PLAYAUDIO
    {
        uint32_t time = to_ms_since_boot(get_absolute_time());
        printf("AUDIO::decode start at %d ms\n", time);
    }
    #endif // DEBUG_PLAYAUDIO

    int32_t *samples = (int32_t *) buffer->buffer->bytes;
    //buffer->sample_count = buffer->max_sample_count;
    //printf("sample_count = %d, max = %d\n", buffer->sample_count, buffer->max_sample_count);
    //printf("prepared_list = %d, prepared_list_tail = %d\n", (int) producer_pool->prepared_list, (int) producer_pool->prepared_list_tail);
    // sample freq difference adjustment
    if (buffer->sample_count > sub_frame_count / 2) {
        buffer->sample_count = sub_frame_count / 2;
    }
    for (int i = 0; i < buffer->sample_count; i++) {
        samples[i*2+0] = (int32_t) (((buff[i*2+0] >> 12) & 0xffff) << 16) / 64; // temporary volume
        samples[i*2+1] = (int32_t) (((buff[i*2+1] >> 12) & 0xffff) << 16) / 64; // temporary volume
    }
    for (int i = buffer->sample_count - 1; i < buffer->sample_count; i++) {
        samples[i*2+0] = (int32_t) (((buff[(i-1)*2+0] >> 12) & 0xffff) << 16) / 64; // temporary volume
        samples[i*2+1] = (int32_t) (((buff[(i-1)*2+1] >> 12) & 0xffff) << 16) / 64; // temporary volume
    }
    give_audio_buffer(producer_pool, buffer);

    /*
	uint32_t time = _millis();
    printf("spdif_rx_callback_func %d\n", time);
    */
    count++;
    return;
}

static int _checkBlock(uint32_t buff[BLOCK_SIZE])
{
    uint pos_syncB = 0;
    uint32_t block_parity_err_count = 0;

    for (int i = 0; i < BLOCK_SIZE; i++) {
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
                c_bits = (c_bits & (~(0x1 << (i/2)))) | (c_bit << (i/2));
            }
            // Parity (27 bits of every sub frame)
            uint32_t count_ones32 = count_ones8[(buff[i]>>24) & 0x7f] + // exclueding P
                                    count_ones8[(buff[i]>>16) & 0xff] +
                                    count_ones8[(buff[i]>> 8) & 0xff] +
                                    count_ones8[(buff[i]>> 0) & 0xf0];  // excluding sync
            if ((count_ones32 & 0x1) != (buff[i] >> 31)) {
                block_parity_err_count++;
            }
        }
    }
    parity_err_count += block_parity_err_count;
    // block align adjustment
    if (block_aligned) {
        trans_count = BLOCK_SIZE;
        if (spdif_rx_get_samp_freq() != SAMP_FREQ_NONE) {
            spdif_rx_callback_func(buff, trans_count, c_bits, block_parity_err_count > 0);
        }
    } else {
        if (pos_syncB != 0 && block_align_count == 0) {
            // dispose pos_syncB to align because fifo_buff[BLOCK_SIZE-1] was (BLOCK_SIZE - pos_syncB)'th sub frame
            // it takes 3 blocks to align because coming 2 transfers already issued
            trans_count = pos_syncB;
            block_align_count = 3;
        } else {
            trans_count = BLOCK_SIZE;
            if (block_align_count > 0) { block_align_count--; }
        }
    }

    return block_aligned;
}

// irq handler for DMA
void __isr __time_critical_func(spdif_rx_dma_irq_handler)() {
    uint64_t now = _micros();
    block_interval[block_count % 10] = now - prev_time;
    uint64_t accum = 0;
    for (int i = 0; i < 10; i++) {
        accum += block_interval[i];
    }
    ave_block_interval = (float) accum / 10;
    block_count++;

    if ((dma_intsx & (1u << gcfg.dma_channel0))) {
        dma_intsx = 1u << gcfg.dma_channel0;
        uint32_t save = spin_lock_blocking(spdif_rx_spin_lock);
        uint32_t done_ptr = buff_wr_done_ptr;
        if (block_aligned) {
            if (spdif_rx_get_fifo_count() + BLOCK_SIZE > FIFO_SIZE) {
                printf("spdif_rx fifo overflow\n");
            }
            buff_wr_done_ptr = ptr_inc(done_ptr, BLOCK_SIZE);
        } else { // if status is not ready, fifo should be empty
            buff_wr_done_ptr = ptr_inc(done_ptr, BLOCK_SIZE);
            buff_rd_ptr = buff_wr_done_ptr;
        }
        dma_channel_configure(
            gcfg.dma_channel0,
            &dma_config0,
            to_buff_ptr(buff_wr_pre_ptr), // dest
            &spdif_rx_pio->rxf[gcfg.pio_sm],  // src
            trans_count, // count
            false // trigger
        );
        buff_wr_pre_ptr = ptr_inc(buff_wr_pre_ptr, BLOCK_SIZE);
        spin_unlock(spdif_rx_spin_lock, save);
        _checkBlock(to_buff_ptr(done_ptr));
        //printf("0");
    }
    if ((dma_intsx & (1u << gcfg.dma_channel1))) {
        dma_intsx = 1u << gcfg.dma_channel1;
        uint32_t save = spin_lock_blocking(spdif_rx_spin_lock);
        uint32_t done_ptr = buff_wr_done_ptr;
        if (block_aligned) {
            if (spdif_rx_get_fifo_count() + BLOCK_SIZE > FIFO_SIZE) {
                printf("spdif_rx fifo overflow\n");
            }
            buff_wr_done_ptr = ptr_inc(done_ptr, BLOCK_SIZE);
        } else { // if status is not ready, fifo should be empty
            buff_wr_done_ptr = ptr_inc(done_ptr, BLOCK_SIZE);
            buff_rd_ptr = buff_wr_done_ptr;
        }
        buff_wr_done_ptr = ptr_inc(done_ptr, BLOCK_SIZE);
        //_spdif_rx_check();
        //_spdif_rx_direct_check();
        dma_channel_configure(
            gcfg.dma_channel1,
            &dma_config1,
            to_buff_ptr(buff_wr_pre_ptr), // dest
            &spdif_rx_pio->rxf[gcfg.pio_sm],  // src
            trans_count, // count
            false // trigger
        );
        buff_wr_pre_ptr = ptr_inc(buff_wr_pre_ptr, BLOCK_SIZE);
        spin_unlock(spdif_rx_spin_lock, save);
        _checkBlock(to_buff_ptr(done_ptr));
        //printf("1");
    }
    prev_time = now;
}

void spdif_rx_setup(const spdif_rx_config_t *config)
{
    spdif_rx_spin_lock = spin_lock_init(SPINLOCK_ID_AUDIO_FREE_LIST_LOCK);

    memmove(&gcfg, config, sizeof(spdif_rx_config_t)); // copy to gcfg
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
        BLOCK_SIZE, // count
        false // trigger
    );
    buff_wr_pre_ptr = ptr_inc(buff_wr_pre_ptr, BLOCK_SIZE);

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
        BLOCK_SIZE, // count
        false // trigger
    );
    buff_wr_pre_ptr = ptr_inc(buff_wr_pre_ptr, BLOCK_SIZE);

    // DMA IRQ
    irq_add_shared_handler(DMA_IRQ_x, spdif_rx_dma_irq_handler, PICO_SHARED_IRQ_HANDLER_DEFAULT_ORDER_PRIORITY);
    //irq_add_shared_handler(DMA_IRQ_x, spdif_rx_dma_irq_handler, 0xff); // highest
    dma_channel_set_irqx_enabled(gcfg.dma_channel0, 1);
    dma_channel_set_irqx_enabled(gcfg.dma_channel1, 1);
    irq_set_enabled(DMA_IRQ_x, 1);

    // Start DMA
    dma_channel_start(gcfg.dma_channel0);

    // PIO start
    spdif_rx_pio_program_t decode_pg = decode_sets[program_id];
    decode_pg.offset = pio_add_program(spdif_rx_pio, decode_pg.program);
    spdif_rx_program_init(
        spdif_rx_pio,
        gcfg.pio_sm,
        decode_pg.offset,
        decode_pg.entry_point,
        decode_pg.get_default_config,
        gcfg.data_pin
    );
    block_aligned = false;
    parity_err_count = 0;
}

void spdif_rx_end()
{
    pio_sm_unclaim(spdif_rx_pio, gcfg.pio_sm);
    spdif_rx_pio_program_t decode_pg = decode_sets[program_id];
    pio_remove_program(spdif_rx_pio, decode_pg.program, decode_pg.offset);
    pio_clear_instruction_memory(spdif_rx_pio);
    dma_channel_unclaim(gcfg.dma_channel0);
    dma_channel_unclaim(gcfg.dma_channel1);
    irq_remove_handler(DMA_IRQ_x, spdif_rx_dma_irq_handler);
    dma_channel_set_irqx_enabled(gcfg.dma_channel0, 0);
    dma_channel_set_irqx_enabled(gcfg.dma_channel1, 0);
}

void spdif_rx_search_next()
{
    // Stop PIO
    pio_sm_unclaim(spdif_rx_pio, gcfg.pio_sm);
    pio_sm_clear_fifos(spdif_rx_pio, gcfg.pio_sm);
    pio_clear_instruction_memory(spdif_rx_pio);
    // Load another program and start
    program_id = (program_id + 1) % (sizeof(decode_sets) / sizeof(spdif_rx_pio_program_t));
    spdif_rx_pio_program_t decode_pg = decode_sets[program_id];
    pio_sm_claim(spdif_rx_pio, gcfg.pio_sm);
    decode_pg.offset = pio_add_program(spdif_rx_pio, decode_pg.program);
    spdif_rx_program_init(
        spdif_rx_pio,
        gcfg.pio_sm,
        decode_pg.offset,
        decode_pg.entry_point,
        decode_pg.get_default_config,
        gcfg.data_pin
    );
    block_aligned = false;
    parity_err_count = 0;
}

bool spdif_rx_status()
{
    bool flag = (block_aligned && block_count != prev_block_count);
    prev_block_count = block_count;
    return flag;
}

float spdif_rx_get_samp_freq_actual()
{
    float bitrate16 = (float) BLOCK_SIZE * 2 * 8 * 1e6 / ave_block_interval;
    return bitrate16 / 32.0;
}

spdif_rx_samp_freq_t spdif_rx_get_samp_freq()
{
    float samp_freq = spdif_rx_get_samp_freq_actual();
    if (samp_freq >= (float) (SAMP_FREQ_44100 - 100) && samp_freq < (float) (SAMP_FREQ_44100 + 100)) {
        return SAMP_FREQ_44100;
    } else if (samp_freq >= (float) (SAMP_FREQ_48000) - 100 && samp_freq < (float) (SAMP_FREQ_48000 + 100)) {
        return SAMP_FREQ_48000;
    } else {
        return SAMP_FREQ_NONE;
    }
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
        return buff_wr_done_ptr + FIFO_SIZE*2 - buff_rd_ptr;
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
    if ((buff_rd_ptr % FIFO_SIZE) + get_count <= FIFO_SIZE) { // cannot take due to the end of fifo_buff
        *buff = to_buff_ptr(buff_rd_ptr);
        buff_rd_ptr = ptr_inc(buff_rd_ptr, get_count);
    } else {
        get_count = FIFO_SIZE - (buff_rd_ptr % FIFO_SIZE);
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