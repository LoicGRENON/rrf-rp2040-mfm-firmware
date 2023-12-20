#include "as5600.h"
#include "hardware/i2c.h"

#include "debug.h"

#define AS5600_ADDR 0x36
#define AS5600_I2C i2c1
#define AS5600_DIR 9
#define AS5600_SDA 10
#define AS5600_SCL 11

#define RegZMCO 0x00
#define RegZPOSA 0x01
#define RegZPOSB 0x02
#define RegMPOSA 0x03
#define RegMPOSB 0x04
#define RegMANGA 0x05
#define RegMANGB 0x06
#define RegCONFA 0x07
#define RegCONFB 0x08
#define RegPUSHTHR 0x0A

#define RegRAWANGLEA 0x0C
#define RegRAWANGLEB 0x0D
#define RegANGLEA 0x0E
#define RegANGLEB 0x0F

#define RegSTATUS 0x0B
#define RegAGC 0x1A
#define RegMAGNITUDEA 0x1B
#define RegMAGNITUDEB 0x1C

#define ConfFTH_Shift 10
#define ConfFTH_18LSB (0x04 << ConfFTH_Shift)
#define ConfSF_Shift 8
#define ConfSF_2X (0x03 << ConfSF_Shift)
#define ConfHYST_Shift 2
#define ConfHYST_LSB3 (0x03 << ConfHYST_Shift)
#define ConfPM_Shift 0
#define ConfPM_NOM (0x00 << ConfPM_Shift)

// bitcount: 3 FTH, 2 SF, 2 HYST, 2 PM => 9 bits to zero
// not altering the first 2 bits (reserved?), not altering watchdog, pwmf or outs
#define CONFIG_MASK 0b1110000011110000
#define AS5601Config (ConfFTH_18LSB | ConfSF_2X | ConfHYST_LSB3 | ConfPM_NOM)
#define TIMEO_US 10000

#define SWAP_BYTES(x) { x = (x >> 8) & 0xff | (x & 0xff) << 8; }

static bool as5600_write(uint8_t reg_addr, uint8_t *buf, uint8_t len) {
    i2c_write_timeout_us(AS5600_I2C, AS5600_ADDR, &reg_addr, 1, true, TIMEO_US);
    return i2c_write_timeout_us(AS5600_I2C, AS5600_ADDR, buf, len, false, TIMEO_US) == len;
}
static bool as5600_write16(uint8_t reg_addr, uint16_t value) {
    SWAP_BYTES(value);
    return as5600_write(reg_addr, (uint8_t*)&value, 2);
}
static bool as5600_write8(uint8_t reg_addr, uint8_t value) {
    return as5600_write(reg_addr, &value, 1);
}

static bool as5600_read(uint8_t reg_addr, uint8_t *buf, uint8_t len) {
    i2c_write_timeout_us(AS5600_I2C, AS5600_ADDR, &reg_addr, 1, true, TIMEO_US);
    return i2c_read_timeout_us(AS5600_I2C, AS5600_ADDR, buf, len, false, TIMEO_US) == len;
}
static bool as5600_read8(uint8_t reg_addr, uint8_t *val) {
    return as5600_read(reg_addr, val, 1);
}

static bool as5600_read16(uint8_t reg_addr, uint16_t *val) {
    if (as5600_read(reg_addr, (uint8_t*)val, 2)) {
        SWAP_BYTES(*val);
        return true;
    }
    return false;
}

bool as5600_init() {
    static bool prereq_init = false;
    if (!prereq_init) {
        log_debug("i2c init");
        i2c_init(AS5600_I2C, 1000000);
        log_debug("gpio init");
        gpio_set_function(AS5600_SDA, GPIO_FUNC_I2C);
        gpio_set_function(AS5600_SCL, GPIO_FUNC_I2C);
        gpio_pull_up(AS5600_SDA);
        gpio_pull_up(AS5600_SCL);

        gpio_init(AS5600_DIR);
        gpio_set_dir(AS5600_DIR, GPIO_OUT);
        gpio_put(AS5600_DIR, true);
        prereq_init = true;
    }
    log_debug("write conf");
    as5600_write16(RegCONFA, AS5601Config);
}

static void as5600_set_config() {
    uint16_t config;
    as5600_read16(RegCONFA, &config);
    // mask to update only desired bit fields
    config = (config & CONFIG_MASK) | AS5601Config;
    as5600_write16(RegCONFA, config);
}

uint8_t as5600_get_status() {
    uint8_t r = 0;
    as5600_read8(RegSTATUS, &r);
    return r & (AS5600_MAGNET_TOO_WEAK | AS5600_MAGNET_TOO_STRONG | AS5600_MAGNET_DETECTED);
}

bool as5600_set_current_zero_position() {
    uint16_t raw_angle;
    uint16_t zero_pos;
    if (as5600_read16(RegRAWANGLEA, &raw_angle) && as5600_read16(RegZPOSA, &zero_pos)) {
        return as5600_write16(RegZPOSA, zero_pos & 0xf000 | raw_angle & 0x0fff);
    }
    return false;
}

uint8_t as5600_get_agc() {
    uint8_t agc;
    as5600_read8(RegAGC, &agc);
    return agc;
}

#define MAKE_READ_REG16(name, reg) uint16_t as5600_##name() { \
    uint16_t u16; \
    as5600_read16(reg, &u16); \
    return u16; \
}

MAKE_READ_REG16(get_angle, RegANGLEA)
MAKE_READ_REG16(get_magnitude, RegMAGNITUDEA)
MAKE_READ_REG16(get_zero_position, RegZPOSA)
MAKE_READ_REG16(get_max_position, RegMPOSA)
MAKE_READ_REG16(get_max_angle, RegMANGA)