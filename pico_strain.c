/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "pico/stdlib.h"
#include <stdio.h>
#include "ADS131/ads131m0x.h"
#ifndef LED_DELAY_MS
#define LED_DELAY_MS 250
#endif

// Initialize the GPIO for the LED
void pico_led_init(void) {
gpio_init(25);
gpio_init(15);
gpio_set_dir(25, GPIO_OUT);
gpio_set_dir(15, GPIO_OUT);
}

// Turn the LED on or off
void pico_set_led(bool led_on) {
    gpio_put(25, led_on);
}

int main() {
    stdio_init_all();
    pico_led_init();
    adcStartup();
    while (true) {
        printf("Revision ID in HEX: 0x%02X\n", REVISION_ID);
        gpio_put(15, true);
        pico_set_led(true);
        sleep_ms(LED_DELAY_MS);
        pico_set_led(false);
        sleep_ms(LED_DELAY_MS);
    }
}
