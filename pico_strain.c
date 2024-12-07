/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "pico/stdlib.h"
#include <stdio.h>
#include "ads131m0x.h"
#include "hal.h"
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
adc_channel_data data;
int main() {
    stdio_init_all();
    pico_led_init();
    adcStartup();
    while (true) {
        if(flag_nDRDY_INTERRUPT){
            flag_nDRDY_INTERRUPT=false;
            readData(&data);
            printf("%d,%d,%d,%d,%d,%d,%d,%d\n",data.channel0,data.channel1,data.channel2,data.channel3,
                data.channel4,data.channel5,data.channel6,data.channel7);
        }
    }
}
