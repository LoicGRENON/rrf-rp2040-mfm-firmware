/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdio.h>
#include <stdlib.h>

// #include "pico/stdlib.h"
// #include "hardware/pio.h"
// #include "ws2812.pio.h"
#include "ws2812.h"
#include "hardware/clocks.h"

#define IS_RGBW true

#define WS2812_PIN 16

void ws2812_init() {
    PIO pio = pio0;
    uint offset = pio_add_program(pio, &ws2812_program);
    int sm = 0;
    ws2812_program_init(pio, sm, offset, WS2812_PIN, 800000, IS_RGBW);
}
// int main() {
    //set_sys_clock_48();
    // stdio_init_all();
    // printf("WS2812 Smoke Test, using pin %d", WS2812_PIN);

    // todo get free sm
    // PIO pio = pio0;
    // int sm = 0;
    // uint offset = pio_add_program(pio, &ws2812_program);

    // ws2812_program_init(pio, sm, offset, WS2812_PIN, 800000, IS_RGBW);

    // int t = 0;
    // while (1) {
        // int pat = rand() % count_of(pattern_table);
        // int dir = (rand() >> 30) & 1 ? 1 : -1;
        // puts(pattern_table[pat].name);
        // puts(dir == 1 ? "(forward)" : "(backward)");
        // for (int i = 0; i < 1000; ++i) {
            // pattern_table[pat].pat(NUM_PIXELS, t);
            // sleep_ms(10);
            // t += dir;
        // }
    // }
// }
