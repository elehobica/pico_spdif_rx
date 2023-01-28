/*------------------------------------------------------/
/ Copyright (c) 2023, Elehobica
/ Released under the BSD-2-Clause
/ refer to https://opensource.org/licenses/BSD-2-Clause
/------------------------------------------------------*/

#include <stdio.h>
#include <math.h>
#include <cinttypes>

#include "pico/stdlib.h"
#include "pico/stdio_usb.h"

#include "hardware/pll.h"
#include "hardware/clocks.h"

#include "spdif_rx/spdif_rx.h"

#define SAMPLES_PER_BUFFER 1156 // Samples / channel

static constexpr uint8_t PIN_DCDC_PSM_CTRL = 23;
static constexpr uint8_t PIN_PICO_SPDIF_RX_DATA = 15;

static inline uint32_t _millis(void)
{
	return to_ms_since_boot(get_absolute_time());
}

void set_pll_usb_96MHz()
{
    // Set PLL_USB 96.0 MHz
    pll_init(pll_usb, 1, 1536 * MHZ, 4, 4);
    clock_configure(clk_usb,
        0,
        CLOCKS_CLK_USB_CTRL_AUXSRC_VALUE_CLKSRC_PLL_USB,
        96 * MHZ,
        48 * MHZ);
    // Change clk_sys to be 96.0 MHz.
    clock_configure(clk_sys,
        CLOCKS_CLK_SYS_CTRL_SRC_VALUE_CLKSRC_CLK_SYS_AUX,
        CLOCKS_CLK_SYS_CTRL_AUXSRC_VALUE_CLKSRC_PLL_USB,
        96 * MHZ,
        96 * MHZ);
    // CLK peri is clocked from clk_sys so need to change clk_peri's freq
    clock_configure(clk_peri,
        0,
        CLOCKS_CLK_PERI_CTRL_AUXSRC_VALUE_CLK_SYS,
        96 * MHZ,
        96 * MHZ);
    // Reinit uart/usb_cdc now that clk_peri has changed
    stdio_uart_init();
    stdio_usb_init(); // don't call multiple times without stdio_usb_deinit because of duplicated IRQ calls
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

int main()
{
    // Call stdio_usb_init() in set_pll_usb_96MHz() for modified code rather than stdio_init_all()
    stdio_init_all();

    // Set PLL_USB 96MHz and use it for PIO clock for I2S
    //set_pll_usb_96MHz();

    measure_freqs();

    // DCDC PSM control
    // 0: PFM mode (best efficiency)
    // 1: PWM mode (improved ripple)
    gpio_init(PIN_DCDC_PSM_CTRL);
    gpio_set_dir(PIN_DCDC_PSM_CTRL, GPIO_OUT);
    gpio_put(PIN_DCDC_PSM_CTRL, 1); // PWM mode for less Audio noise

    spdif_rx_config_t config = {
        .data_pin = PIN_PICO_SPDIF_RX_DATA,
        .pio_sm = 0,
        .dma_channel0 = 0,
        .dma_channel1 = 1
    };
    spdif_rx_setup(&config);
    printf("setup done\n");

    while (true) {
        if (spdif_rx_status()) {
            uint32_t samp_freq = spdif_rx_get_samp_freq();
            float samp_freq_actual = spdif_rx_get_get_samp_freq_actual();
            printf("Samp Freq = %d Hz (%7.4f KHz)\n", samp_freq, samp_freq_actual / 1e3);
            printf("c_bits = 0x%08x\n", spdif_rx_get_c_bits());
            printf("parity errors = %d\n", spdif_rx_get_parity_err_count());
        } else {
            printf("stable sync not detected\n");
            spdif_rx_search_next();
        }
        tight_loop_contents();
        sleep_ms(1000);
    }

    return 0;
}
