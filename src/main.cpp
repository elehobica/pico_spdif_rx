/*------------------------------------------------------/
/ Copyright (c) 2023, Elehobica
/ Released under the BSD-2-Clause
/ refer to https://opensource.org/licenses/BSD-2-Clause
/------------------------------------------------------*/

#include <stdio.h>
#include <math.h>

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

void pw_set_pll_usb_96MHz()
{
    // Set PLL_USB 96MHz
    pll_init(pll_usb, 1, 1536 * MHZ, 4, 4);
    clock_configure(clk_usb,
        0,
        CLOCKS_CLK_USB_CTRL_AUXSRC_VALUE_CLKSRC_PLL_USB,
        96 * MHZ,
        48 * MHZ);
    // Change clk_sys to be 96MHz.
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

int main()
{
    // Call stdio_usb_init() in pw_set_pll_usb_96MHz() for modified code rather than stdio_init_all()
    //stdio_init_all();

    // Set PLL_USB 96MHz and use it for PIO clock for I2S
    pw_set_pll_usb_96MHz();

    // DCDC PSM control
    // 0: PFM mode (best efficiency)
    // 1: PWM mode (improved ripple)
    gpio_init(PIN_DCDC_PSM_CTRL);
    gpio_set_dir(PIN_DCDC_PSM_CTRL, GPIO_OUT);
    gpio_put(PIN_DCDC_PSM_CTRL, 1); // PWM mode for less Audio noise

    spdif_rx_config_t config = {
        .data_pin = PIN_PICO_SPDIF_RX_DATA,
        .pio_sm0 = 0,
        .pio_sm1 = 1,
        .dma_channel0 = 0,
        .dma_channel1 = 1
    };
    spdif_rx_setup(&config);
    printf("setup done\n");

    while (true) {
        // Echo characters received from PIO to the console
        spdif_rx_loop();
        tight_loop_contents();
        //sleep_ms(1000);
    }

    return 0;
}
