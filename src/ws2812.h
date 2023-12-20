#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "ws2812.pio.h"
static inline void put_pixel(uint32_t pixel_grb) {
    pio_sm_put_blocking(pio0, 0, pixel_grb << 8u);
}

static inline void put_pixel_rgbw(uint32_t pixel_grbw) {
    pio_sm_put_blocking(pio0, 0, pixel_grbw);
}

// TODO figure out how to handle white, shifting w << 24 doesn't seem to work
static inline uint32_t urgb_u32(uint8_t r, uint8_t g, uint8_t b) {
    return
            ((uint32_t) (r) << 8) |
            ((uint32_t) (g) << 16) |
            (uint32_t) (b);
}

void ws2812_init();