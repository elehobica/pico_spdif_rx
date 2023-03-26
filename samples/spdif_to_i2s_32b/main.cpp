/*------------------------------------------------------/
/ Copyright (c) 2023, Elehobica
/ Released under the BSD-2-Clause
/ refer to https://opensource.org/licenses/BSD-2-Clause
/------------------------------------------------------*/

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/clocks.h"
#include "hardware/pio.h"
#include "pico/audio_i2s.h"
#include "spdif_rx.h"

static constexpr uint8_t PIN_DCDC_PSM_CTRL = 23;
static constexpr uint8_t PIN_PICO_SPDIF_RX_DATA = 15;

static constexpr int SAMPLES_PER_BUFFER = PICO_AUDIO_I2S_BUFFER_SAMPLE_LENGTH; // Samples / channel
static constexpr int32_t DAC_ZERO = 1;
static int16_t buf_s16[SAMPLES_PER_BUFFER*2]; // 16bit 2ch data before applying volume
static audio_buffer_pool_t* ap = nullptr;
static bool decode_flg = false;
volatile static bool i2s_setup_flg = false;
volatile static bool i2s_cancel_flg = false;

#define audio_pio __CONCAT(pio, PICO_AUDIO_I2S_PIO)

static audio_format_t audio_format = {
    .sample_freq = 44100,
    .pcm_format = AUDIO_PCM_FORMAT_S32,
    .channel_count = AUDIO_CHANNEL_STEREO
};

static audio_buffer_format_t producer_format = {
    .format = &audio_format,
    .sample_stride = 8
};

static audio_i2s_config_t i2s_config = {
    .data_pin = PICO_AUDIO_I2S_DATA_PIN,
    .clock_pin_base = PICO_AUDIO_I2S_CLOCK_PIN_BASE,
    .dma_channel0 = 0,
    .dma_channel1 = 1,
    .pio_sm = 0
};

static io_rw_32* reg_clkdiv;
static io_rw_32 clkdiv_tbl[3];
typedef enum _clkdiv_speed_t {
    CLKDIV_FAST = 0,
    CLKDIV_NORM = 1,
    CLKDIV_SLOW = 2
} clkdiv_speed_t;

static int32_t volume = 10;

void save_center_clkdiv(PIO pio, uint sm)
{
    reg_clkdiv = &(pio->sm[sm].clkdiv);
    clkdiv_tbl[CLKDIV_NORM] = *reg_clkdiv;
    clkdiv_tbl[CLKDIV_FAST] = clkdiv_tbl[CLKDIV_NORM] - (1 << PIO_SM0_CLKDIV_FRAC_LSB);
    clkdiv_tbl[CLKDIV_SLOW] = clkdiv_tbl[CLKDIV_NORM] + (1 << PIO_SM0_CLKDIV_FRAC_LSB);
}

void set_offset_clkdiv(clkdiv_speed_t speed)
{
    *reg_clkdiv = clkdiv_tbl[speed];
}

void i2s_audio_deinit()
{
    decode_flg = false;

    audio_i2s_set_enabled(false);
    audio_i2s_end();

    audio_buffer_t* ab;
    ab = take_audio_buffer(ap, false);
    while (ab != nullptr) {
        free(ab->buffer->bytes);
        free(ab->buffer);
        ab = take_audio_buffer(ap, false);
    }
    ab = get_free_audio_buffer(ap, false);
    while (ab != nullptr) {
        free(ab->buffer->bytes);
        free(ab->buffer);
        ab = get_free_audio_buffer(ap, false);
    }
    ab = get_full_audio_buffer(ap, false);
    while (ab != nullptr) {
        free(ab->buffer->bytes);
        free(ab->buffer);
        ab = get_full_audio_buffer(ap, false);
    }
    free(ap);
    ap = nullptr;
}

audio_buffer_pool_t *i2s_audio_init(uint32_t sample_freq)
{
    audio_format.sample_freq = sample_freq;

    audio_buffer_pool_t *producer_pool = audio_new_producer_pool(&producer_format, 3, SAMPLES_PER_BUFFER);
    ap = producer_pool;

    bool __unused ok;
    const audio_format_t *output_format;

    output_format = audio_i2s_setup(&audio_format, &audio_format, &i2s_config);
    if (!output_format) {
        panic("PicoAudio: Unable to open audio device.\n");
    }

    ok = audio_i2s_connect(producer_pool);
    assert(ok);
    { // initial buffer data
        audio_buffer_t *ab = take_audio_buffer(producer_pool, true);
        int32_t *samples = (int32_t *) ab->buffer->bytes;
        for (uint i = 0; i < ab->max_sample_count; i++) {
            samples[i*2+0] = DAC_ZERO;
            samples[i*2+1] = DAC_ZERO;
        }
        ab->sample_count = ab->max_sample_count;
        give_audio_buffer(producer_pool, ab);
    }
    save_center_clkdiv(audio_pio, i2s_config.pio_sm);
    audio_i2s_set_enabled(true);

    decode_flg = true;
    return producer_pool;
}

void decode()
{
    static bool mute_flag = true;

    if (ap == nullptr) { return; }
    audio_buffer_t *ab;
    if ((ab = take_audio_buffer(ap, false)) == nullptr) { return; }

    #ifdef DEBUG_PLAYAUDIO
    {
        uint32_t time = to_ms_since_boot(get_absolute_time());
        printf("AUDIO::decode start at %d ms\n", time);
    }
    #endif // DEBUG_PLAYAUDIO

    ab->sample_count = ab->max_sample_count;
    int32_t *samples = (int32_t *) ab->buffer->bytes;

    uint32_t fifo_count = spdif_rx_get_fifo_count();
    if (spdif_rx_get_state() == SPDIF_RX_STATE_STABLE) {
        if (mute_flag && fifo_count >= SPDIF_RX_FIFO_SIZE / 2) {
            mute_flag = false;
        }
    } else {
        mute_flag = true;
    }

    if (mute_flag) {
        for (int i = 0; i < ab->sample_count; i++) {
            samples[i*2+0] = DAC_ZERO;
            samples[i*2+1] = DAC_ZERO;
        }
    } else {
        // I2S frequency adjustment (feedback from SPDIF_RX fffo_count)
        // note that this scheme could increase I2S clock jitter
        // (using pio_sm_set_clkdiv should have already include jitter unless dividing by integer)
        if (fifo_count <= SPDIF_RX_FIFO_SIZE / 2 - SPDIF_BLOCK_SIZE) {
            set_offset_clkdiv(CLKDIV_SLOW);
            //printf("<");
        } else if (fifo_count <= SPDIF_RX_FIFO_SIZE / 2 + SPDIF_BLOCK_SIZE) {
            set_offset_clkdiv(CLKDIV_NORM);
            //printf("-");
        } else {
            set_offset_clkdiv(CLKDIV_FAST);
            //printf(">");
        }
        //printf("%d,", fifo_count);
        if (ab->sample_count > fifo_count / 2) {
            ab->sample_count = fifo_count / 2;
        }
        uint32_t total_count = ab->sample_count * 2;
        int i = 0;
        uint32_t read_count = 0;
        uint32_t* buff;
        while (read_count < total_count) {
            uint32_t get_count = spdif_rx_read_fifo(&buff, total_count - read_count);
            for (int j = 0; j < get_count / 2; j++) {
                samples[i*2+0] = (int32_t) ((buff[j*2+0] & 0x0ffffff0) << 4) / 256 * volume + DAC_ZERO;
                samples[i*2+1] = (int32_t) ((buff[j*2+1] & 0x0ffffff0) << 4) / 256 * volume + DAC_ZERO;
                i++;
            }
            read_count += get_count;
        }
    }
    give_audio_buffer(ap, ab);

    #ifdef DEBUG_PLAYAUDIO
    {
        uint32_t time = to_ms_since_boot(get_absolute_time());
        printf("AUDIO::decode end   at %d ms\n", time);
    }
    #endif // DEBUG_PLAYAUDIO
}

void measure_freqs(void) {
    uint f_pll_sys = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_PLL_SYS_CLKSRC_PRIMARY);
    uint f_pll_usb = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_PLL_USB_CLKSRC_PRIMARY);
    uint f_rosc = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_ROSC_CLKSRC);
    uint f_clk_sys = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_CLK_SYS);
    uint f_clk_peri = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_CLK_PERI);
    uint f_clk_usb = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_CLK_USB);
    uint f_clk_adc = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_CLK_ADC);
    uint f_clk_rtc = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_CLK_RTC);

    printf("pll_sys  = %dkHz\n", f_pll_sys);
    printf("pll_usb  = %dkHz\n", f_pll_usb);
    printf("rosc     = %dkHz\n", f_rosc);
    printf("clk_sys  = %dkHz\n", f_clk_sys);
    printf("clk_peri = %dkHz\n", f_clk_peri);
    printf("clk_usb  = %dkHz\n", f_clk_usb);
    printf("clk_adc  = %dkHz\n", f_clk_adc);
    printf("clk_rtc  = %dkHz\n", f_clk_rtc);

    // Can't measure clk_ref / xosc as it is the ref
}

extern "C" {
void i2s_callback_func();
}
// callback from:
//   void __isr __time_critical_func(audio_i2s_dma_irq_handler)()
//   defined at my_pico_audio_i2s/audio_i2s.c
//   where i2s_callback_func() is declared with __attribute__((weak))
void i2s_callback_func()
{
    if (decode_flg) {
        decode();
    }
}

void i2s_setup(spdif_rx_samp_freq_t samp_freq)
{
    float samp_freq_actual = spdif_rx_get_samp_freq_actual();
    uint32_t c_bits;
    spdif_rx_get_c_bits(&c_bits, sizeof(c_bits), 0);
    printf("Samp Freq = %d Hz (%6.4f KHz)\n", (int) samp_freq, samp_freq_actual / 1e3);
    printf("SPDIF C bits = %08x\n", c_bits);
    if (ap != nullptr) {
        i2s_audio_deinit(); // less gap noise if deinit() is done when input is stable
    }
    // need to care the case when lost during setup to avoid noise
    if (i2s_cancel_flg) { return; }

    i2s_audio_init(samp_freq);

    // need to care the case when lost during setup to avoid noise
    if (i2s_cancel_flg) {
        if (ap != nullptr) {
            i2s_audio_deinit();
        }
    }
}

void on_stable_func(spdif_rx_samp_freq_t samp_freq)
{
    // callback function should be returned as quick as possible
    i2s_setup_flg = true;
    i2s_cancel_flg = false;
}

void on_lost_stable_func()
{
    // callback function should be returned as quick as possible
    i2s_cancel_flg = true;
}

int main()
{
    stdio_init_all();

    //measure_freqs();

    // DCDC PSM control
    // 0: PFM mode (best efficiency)
    // 1: PWM mode (improved ripple)
    gpio_init(PIN_DCDC_PSM_CTRL);
    gpio_set_dir(PIN_DCDC_PSM_CTRL, GPIO_OUT);
    gpio_put(PIN_DCDC_PSM_CTRL, 1); // PWM mode for less Audio noise

    spdif_rx_config_t config = {
        .data_pin = PIN_PICO_SPDIF_RX_DATA,
        .pio_sm = 0,
        .dma_channel0 = 2,
        .dma_channel1 = 3,
        .alarm = 0,
        .flags = SPDIF_RX_FLAGS_ALL
    };

    spdif_rx_start(&config);
    spdif_rx_set_callback_on_stable(on_stable_func);
    spdif_rx_set_callback_on_lost_stable(on_lost_stable_func);

    while (true) {
        if (i2s_setup_flg) {
            i2s_setup_flg = false;
            i2s_setup(spdif_rx_get_samp_freq());
        }
        int c = getchar_timeout_us(0);
        if (c) {
            if (c == '-' && volume > 0) {
                volume--;
            } else if ((c == '=' || c == '+') && volume < 256) {
                volume++;
            }
        }
        tight_loop_contents();
        sleep_ms(10);
    }

    return 0;
}
