#include "pico/stdlib.h"

bool as5600_init();

#define AS5600_MAGNET_DETECTED 0b00100000
#define AS5600_MAGNET_TOO_WEAK 0b00010000
#define AS5600_MAGNET_TOO_STRONG 0b00001000
#define AS500_ABN_2048_15K6HZ 0x08


uint8_t as5600_get_status();
bool as5600_set_abn(uint8_t);
bool as5600_set_current_zero_position();
uint16_t as5600_get_angle();
uint8_t as5600_get_agc();
uint16_t as5600_get_magnitude();

uint16_t as5600_get_zero_position();
uint16_t as5600_get_max_position();
uint16_t as5600_get_max_angle();