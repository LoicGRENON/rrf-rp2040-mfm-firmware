#include <stdio.h>
#include <stdlib.h>

#include "pico/time.h"
#include "pico/stdlib.h"
#include "hardware/watchdog.h"
#include "hardware/gpio.h"

#include "FreeRTOS.h"
#include <task.h>
#include <queue.h>

#include "ws2812.h"
#include "debug.h"
#include "as5600.h"

#define INPUT_PIN 29
#define OUTPUT_PIN 28

#define FIRMWARE_VERSION 4

TaskHandle_t input_task_handle = NULL;
TaskHandle_t output_task_handle = NULL;
TaskHandle_t as5600_task_handle = NULL;
TaskHandle_t neopixel_task_handle = NULL;
TaskHandle_t wdt_handle = NULL;

QueueHandle_t output_queue = NULL;

const uint16_t ParityBit = 0x8000u;
const uint16_t ErrorBits = 0x2000u;
const uint16_t PositionBits = 0x0800u;
const uint16_t VersionBits = 0x6000u;
const uint16_t MagnitudeBits = 0x6200u;
const uint16_t AgcBits = 0x6300u;

#define STATUS_OK 3
#define STATUS_ERR_I2C 4
#define STATUS_ERR_NOMAG 6
#define STATUS_ERR_TOO_WEAK 7
#define STATUS_ERR_TOO_STRONG 8
#define STATUS_FORWARD 9
#define STATUS_BACKWARD 10
#define STATUS_PULSE_ON 11
#define STATUS_PULSE_OFF 12

// uint8_t mode format PMMMIKKK:
//   1 bit parity, 3 bits to select mode, I invert bit, 3 bits pulse mask
//     0-7=(10 bits total, selectable 2-9 bit mask for values of 0-7)
//     I bit inverts pulse direction output on pulse mode: 1 = invert, 0 = as-is
#define MODE_MFM 0x10 // default is mfm
#define MODE_PULSE 0x20 // will be pulse until rebooted, or set back to mfm
// 24mm rotation,
// 9 bit mask = on for half rotation, off for half rotation, 1 pulse per rotation
// 8 = on for 1/4 rotation
// 7 = on for 1/8 rotation
// 6 = on for 1/16
// 5 = on for 1/32
// 4 = on for 1/64
// 3 = on for 1/128
// 2 = on for 1/256
uint8_t pulse_mask = 0;
bool pulse_dir_invert = 0;
uint8_t sensor_mode = MODE_MFM;
bool suppress_output = false;

#define AS5600_BITS_TO_DISCARD 2

void input_pin_isr(uint, uint32_t);
void enable_irq(bool enable) {
    gpio_set_irq_enabled_with_callback(
        INPUT_PIN,
        GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE,
        enable,
        &input_pin_isr);
}

void set_sensor_mode(uint8_t mode) {
    uint8_t new_mode;
    suppress_output = true;
    if ((mode & MODE_PULSE) == MODE_PULSE) {
        pulse_mask = 2 + (mode & 0x07);
        pulse_dir_invert = mode & 0x04;
        sensor_mode = MODE_PULSE;
        log_debug("Setting sensor to pulse mode with %d bit mask", pulse_mask);
    } else {
        log_debug("Setting sensor to mfm mode");
        // delay setting mode to mfm so the position task does not interfere
        sensor_mode = MODE_MFM;
    }
    // allow the printer firmware to confirm the mode switch is successful, query the pin
    // states over a period of time to ascertain success
    enable_irq(false);
    gpio_pull_down(INPUT_PIN);
    gpio_put(OUTPUT_PIN, true);
    vTaskDelay(pdMS_TO_TICKS(50));
    gpio_pull_up(INPUT_PIN);
    gpio_put(OUTPUT_PIN, false);
    vTaskDelay(pdMS_TO_TICKS(50));
    gpio_pull_down(INPUT_PIN);
    gpio_put(OUTPUT_PIN, true);
    vTaskDelay(pdMS_TO_TICKS(50));
    gpio_put(OUTPUT_PIN, false);

    // reset any pullup/down from pulse counting
    gpio_pull_down(INPUT_PIN);
    enable_irq(true);
    suppress_output = false;
}

uint32_t status_to_color(uint32_t status) {
    switch (status) {
        case STATUS_OK:
            return urgb_u32(64, 64, 0);
        case STATUS_ERR_I2C:
            return urgb_u32(255, 0, 0);
        case STATUS_ERR_NOMAG:
            return urgb_u32(255, 255, 0);
        case STATUS_ERR_TOO_STRONG:
            return urgb_u32(0, 255, 255);
        case STATUS_ERR_TOO_WEAK:
            return urgb_u32(255, 0, 255);
        case STATUS_FORWARD:
            return urgb_u32(0, 128, 0);
        case STATUS_BACKWARD:
            return urgb_u32(96, 0, 96);
        default:
            return urgb_u32(0, 255, 0);
    }
}

void neopixel_task(void* unused_arg) {
    bool is_on = false;
    ws2812_init();

    uint8_t counter = 0;

    uint32_t led_status;
    uint32_t last_status = STATUS_OK;
    bool is_error;
    for (;;) {
        if (xTaskNotifyWait(0, 0xffffffffUL, &led_status, pdMS_TO_TICKS(750))) {
            if (led_status == STATUS_PULSE_ON) {
                put_pixel(urgb_u32(64, 64, 64));
            } else if (led_status == STATUS_PULSE_OFF) {
                put_pixel(0);
            }
            //log_debug("got led data: %d", led_status);
            if (last_status != led_status) {
                is_on = 0;
                counter = 0;
            }
            last_status = led_status;
        }
        if (led_status != STATUS_PULSE_ON && led_status != STATUS_PULSE_OFF) {
            put_pixel(is_on ? 0 : status_to_color(last_status));

            counter = (counter + 1) % 3;
            if ((last_status != STATUS_FORWARD && last_status != STATUS_BACKWARD) || counter == 0) {
                is_on = !is_on;
            }
        }
    }
}

void wdt_task(void* unused_arg) {
    uint8_t count = 0;
    for (;;) {
        watchdog_update();
        vTaskDelay(pdMS_TO_TICKS(50));
        // tick every 5 seconds just so I can make sure my console isnt dead
        if (++count % 100 == 0) {
            count = 0;
            log_debug("watchdog tick");
        }
    }
}

void input_pin_isr(uint gpio, uint32_t events) {
    if (gpio == INPUT_PIN) {
        BaseType_t task_woken = pdFALSE;
        vTaskNotifyGiveFromISR(input_task_handle, &task_woken);
        portYIELD_FROM_ISR(task_woken);
    }
}

void _send_word(TickType_t *xNow, uint16_t data) {
    // no need to swap bytes here as bits are being sent one by one starting from MSB
	// Calculate the parity bit
	uint8_t data8 = (uint8_t)((data >> 8) ^ data);
	data8 ^= (data8 >> 4);
	data8 ^= (data8 >> 2);
	data8 ^= (data8 >> 1);
	if (data8 & 1)
	{
		data ^= ParityBit;
	}

    // start sequence
    gpio_put(OUTPUT_PIN, false);
    vTaskDelayUntil(xNow, pdMS_TO_TICKS(1));
    gpio_put(OUTPUT_PIN, true);
    vTaskDelayUntil(xNow, pdMS_TO_TICKS(1));
    gpio_put(OUTPUT_PIN, false);

	// Send 4 nibbles + stuffing bits
	for (uint8_t nibble = 0; nibble < 4; ++nibble)
	{
		bool b;
		for (uint8_t i = 0; i < 4; ++i)
		{
			b = ((data & 0x8000u) != 0);
            vTaskDelayUntil(xNow, pdMS_TO_TICKS(1));
            gpio_put(OUTPUT_PIN, b);
			data <<= 1;
		}

		// Send the stuffing bit, which is the opposite of the last bit
        vTaskDelayUntil(xNow, pdMS_TO_TICKS(1));
        gpio_put(OUTPUT_PIN, !b);

	}

    vTaskDelayUntil(xNow, pdMS_TO_TICKS(1));
    gpio_put(OUTPUT_PIN, false);
    vTaskDelayUntil(xNow, pdMS_TO_TICKS(2));
}

void send_word(uint16_t data) {
    if (!xQueueSendToBack(output_queue, &data, 0)) {
        log_debug("Failed to queue send_word");
    }
}

void send_led(uint8_t status) {
    xTaskNotify(neopixel_task_handle, status, eSetValueWithOverwrite);
}

void report_error(uint8_t errnum) {
    log_debug("reporting error %d", errnum);
    send_led(errnum);
    send_word(ErrorBits | errnum);
}

void output_pin_task(void* unused_arg) {
    uint16_t output_word;
    while (true) {
        if (xQueueReceive(output_queue, &output_word, pdMS_TO_TICKS(500)) && !suppress_output) {
            if (sensor_mode == MODE_MFM) {
                //log_debug("sending output 0x%04x", output_word);
                TickType_t xNow = xTaskGetTickCount();
                _send_word(&xNow, output_word);
                // give the receiver some time to recover after sending a word
                vTaskDelay(pdMS_TO_TICKS(10));
            } else {
                // pulse mode just set on or off
                gpio_put(OUTPUT_PIN, output_word);
            }
        }
    }
}

uint8_t status_to_err(uint8_t status) {
	return  ((status & AS5600_MAGNET_TOO_STRONG) != 0)
        ? STATUS_ERR_TOO_STRONG
        : ((status & AS5600_MAGNET_DETECTED) == 0)
            ? STATUS_ERR_NOMAG
            : ((status & AS5600_MAGNET_TOO_WEAK) != 0)
                ? STATUS_ERR_TOO_WEAK
                : STATUS_OK;
}

uint8_t pulse_direction(uint16_t last_angle, uint16_t angle) {
    // TODO simplify
    if (angle > last_angle) {
        // detect wraparound
        return abs(angle - last_angle) > 500 ? STATUS_BACKWARD : STATUS_FORWARD;
    } else {
        // detect wraparound
        return abs(angle - last_angle) > 500 ? STATUS_FORWARD : STATUS_BACKWARD;
    }

}

void as5600_position_task(void* unused_arg) {
    uint16_t last_angle = 0;
    uint8_t direction = STATUS_FORWARD;

    for (;;) {
        send_word(VersionBits | FIRMWARE_VERSION);
        log_debug("Initializing AS5600");

        if (!as5600_init()) {
            report_error(STATUS_ERR_I2C);
            log_debug("Failed to initialize AS5600");
        } else {
            vTaskDelay(pdMS_TO_TICKS(1)); // init needs some time to settle

            uint8_t status = as5600_get_status();
            log_debug("Magnet status: 0x%02x", status);

            if (status == AS5600_MAGNET_DETECTED) {
                break;
            }
            report_error(status_to_err(status));
        }
    }

    log_debug("zpos = %d, max pos = %d, max angle = %d",
        as5600_get_zero_position(),
        as5600_get_max_position(),
        as5600_get_max_angle());
    log_debug("Entering magnet monitor loop");

    TickType_t xLastWakeTime = xTaskGetTickCount();
    TickType_t xLastPositionTime;
    TickType_t xLastStatusTime;
    bool status_good = false;
    bool send_status = false;
    for (;;) {
        xTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(40));
        uint8_t status = as5600_get_status();
        if (status == AS5600_MAGNET_DETECTED) {
            uint16_t angle = (as5600_get_angle() >> AS5600_BITS_TO_DISCARD) & 0x3ff;
            if (sensor_mode == MODE_MFM) {
                // drop 2 bits and take the lower 10bits for a range of 0-1024
                if (last_angle != angle) {
                    direction = pulse_direction(last_angle, angle);
                    log_debug("Sending position %c %d/1024",
                        direction == STATUS_FORWARD ? '>' : '<', angle);
                    send_led(direction);
                    send_word(PositionBits | angle);
                    xLastPositionTime = xLastWakeTime;
                } else if (!status_good || (pdMS_TO_TICKS(500) <= (xLastWakeTime - xLastPositionTime))) {
                    if (!status_good) {
                        log_debug("status has fixed, send immediate update");
                    }
                    log_debug("No position update sending %d/1024", last_angle);
                    send_led(status_to_err(status));
                    send_word(PositionBits | last_angle);
                    xLastPositionTime = xLastWakeTime;
                }
                status_good = true;
            } else if (!suppress_output) {
                bool last_pulse = (last_angle >> pulse_mask) & 1;
                bool pulse_on = (angle >> pulse_mask) & 1;
                if (last_pulse != pulse_on) {
                    direction = pulse_direction(last_angle, angle);
                    enable_irq(false);
                    bool up_or_down = direction == STATUS_FORWARD;
                    if (pulse_dir_invert) {
                        up_or_down = !up_or_down;
                    }

                    if (up_or_down) {
                        gpio_pull_up(INPUT_PIN);
                    } else {
                        gpio_pull_down(INPUT_PIN);
                    }
                    enable_irq(true);
                    log_debug("Sending pulse state %s", pulse_on ? "on" : "off");
                }
                send_led(pulse_on ? STATUS_PULSE_ON : STATUS_PULSE_OFF);
                gpio_put(OUTPUT_PIN, pulse_on);
            }
            last_angle = angle;
        } else {
            if (status_good) {
                // send status immediately when the as5600 state goes bad
                send_status = true;
            }
            status_good = false;
        }

        if (sensor_mode == MODE_MFM && (send_status || pdMS_TO_TICKS(500) <= xLastWakeTime - xLastStatusTime)) {
            if (!status_good) {
                log_debug("sensor in a bad status, send status %d", status);
                report_error(status_to_err(status));
            }
            uint8_t agc = as5600_get_agc();
            uint16_t mag = as5600_get_magnitude() >> 4;
            log_debug("Sending status mag %d agc %d", mag, agc);
            send_word(VersionBits | FIRMWARE_VERSION);
            send_word(AgcBits | agc);
            send_word(MagnitudeBits | (mag & 0x00ff));

            send_status = false;
            xLastStatusTime = xLastWakeTime;
        }
    }
}

typedef enum {
    input_start,
    input_bits,
    input_done,
    input_recovery,
} input_state;

void validate_process_input(bool parity, uint8_t value) {
    uint8_t data8 = value & 0x7f;
    data8 ^= (data8 >> 4);
    data8 ^= (data8 >> 2);
    data8 ^= (data8 >> 1);
    if ((data8 & 1) != parity)
    {
        log_debug("parity mismatch 0x%02x %d != %d", value, data8 & 1, parity);
    } else {
        log_debug("received input command: 0x%02x data8=0x%02x parity=%d", value & 0x7f, data8, parity);
        set_sensor_mode(value & 0x7f);
    }
}

void input_pin_task(void* unused_arg) {
    input_state state = input_done;
    log_debug("input pin task");
    absolute_time_t last_time = get_absolute_time();
    uint8_t bit_count = 0;
    bool parity_value;
    bool need_stuffing_check;
    bool stuffing_bit_passed;
    uint8_t value = 0;

    for (;;) {
        if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(100)) && !suppress_output) {
            absolute_time_t now = get_absolute_time();
            uint64_t diff_us = absolute_time_diff_us(last_time, now);
            bool is_high = !gpio_get(INPUT_PIN);
            // actually, we're counting *was* high
            // how many bits does this interval represent?
            uint64_t bits = diff_us / 10000 + (diff_us % 10000 > 5000 ? 1 : 0);
            // something is wrong with this log_debug... some imaginary 0 value is pulled in for 3rd
            log_debug("state %d, count %d, ignore %d, bit %d", state, bits, is_high ? 1 : 0, diff_us);
            switch (state) {
                case input_start:
                  if (!is_high && bits > 0) {
                    log_debug("start bit low");
                    state = input_bits;
                    value = 0;
                    bits--;
                  } else {
                    log_debug("wrong start state %d %d %dus", is_high, bits, diff_us);
                    bits = 0;
                    state = input_recovery;
                  }
                  if (bits == 0) {
                    break;
                  }
                case input_bits: if (bits > 0) {
                  if (bits < 1 || bits > 5) {
                    log_debug("Bit interval wrong %dus", diff_us);
                    state = input_recovery;
                    break;
                  }
                  if (bit_count == 0) {
                    parity_value = is_high;
                    log_debug("parity bit %d", parity_value);
                    value = value << 1 | parity_value;
                    bit_count++;
                    bits--;
                  }
                  for (;bits > 0 && bit_count <= 8; bits--) {
                    if (bit_count == 1 || bit_count == 5) {
                        need_stuffing_check = true;
                    }
                    if (bit_count > 0 && bit_count % 4 == 0 && need_stuffing_check) {
                        need_stuffing_check = false;
                        if (value & 1 == is_high) {
                            log_debug("stuffing bit mismatch %d == %d", value & 1, is_high);
                            state = input_recovery;
                        } else if (bit_count == 8) {
                            stuffing_bit_passed = true;
                        }
                    } else {
                        value = value << 1 | is_high;
                        bit_count++;
                        log_debug("saving regular value: %d 0x%x", bit_count, value);
                    }
                  }
                  if (bit_count > 8) {
                    log_debug("Too many bits found: %d", bit_count);
                    state = input_recovery;
                  }
                  if (bit_count == 8 && stuffing_bit_passed) {
                    validate_process_input(parity_value, value);
                    bit_count = 0;
                    state = input_done;
                  }
                }
                  break;
                case input_recovery:
                  // do nothing until we time out
                  log_debug("Waiting for recovery from bad bits: %d", is_high);
                  break;
                case input_done:
                  if (is_high) {
                    if (diff_us > 7000 && diff_us < 13000) {
                        log_debug("start bit high");
                        state = input_start;
                    } else {
                        log_debug("start high pulse wrong duration %dus", diff_us);
                        state = input_recovery;
                    }
                  }
                  break;
            }
            last_time = now;
        } else {
            if (bit_count == 8 && stuffing_bit_passed) {
                validate_process_input(parity_value, value);
            } else if (bit_count == 8 && need_stuffing_check) {

                // logic slightly inverted because we're checking after the edge transition
                bool is_high = !gpio_get(INPUT_PIN);
                if (value & 1 == gpio_get(INPUT_PIN)) {
                    log_debug("stuffing bit mismatch %d == %d", value & 1, is_high);
                    state = input_recovery;
                } else {
                    validate_process_input(parity_value, value);
                }
            } else if (bit_count > 0) {
                log_debug("discarding incomplete value 0x%02x bits %d", value, bit_count);
            }
            bit_count = 0;
            stuffing_bit_passed = false;
            state = input_done;
        }
    }
}

void log_device_info(void) {
    printf("App: %s %s (%i)\n", APP_NAME, APP_VERSION, BUILD_NUM);
}

int main() {

#ifdef DEBUG
    stdio_usb_init();
    // stdio_init_all();
    // Pause to allow the USB path to initialize
    sleep_ms(2000);

    // Log app info
    log_device_info();
#endif

    watchdog_enable(100, 1); // 100ms watchdog update interval, pause on debug
    watchdog_update();

    gpio_init(OUTPUT_PIN);
    gpio_set_dir(OUTPUT_PIN, GPIO_OUT);
    gpio_put(OUTPUT_PIN, false);
    gpio_init(INPUT_PIN);
    gpio_pull_down(INPUT_PIN);
    gpio_set_dir(INPUT_PIN, GPIO_IN);
    enable_irq(true);

    BaseType_t pico_status = xTaskCreateAffinitySet(output_pin_task,
                                         "OutputPin", 
                                         1024, 
                                         NULL, 
                                         1, 
                                         2,
                                         &output_task_handle);
    BaseType_t gpio_status = xTaskCreate(input_pin_task, 
                                         "InputPin", 
                                         1024, 
                                         NULL, 
                                         1, 
                                         &input_task_handle);
    BaseType_t as5600_status = xTaskCreateAffinitySet(as5600_position_task, 
                                           "AS5600Position", 
                                           1024, 
                                           NULL, 
                                           1, 
                                           1,
                                           &as5600_task_handle);
    BaseType_t neopixel_status = xTaskCreateAffinitySet(neopixel_task,
                                             "NeopixelTask", 
                                             1024, 
                                             NULL, 
                                             1,
                                             2,
                                             &neopixel_task_handle);
    BaseType_t wdt_status = xTaskCreate(wdt_task, 
                                        "Watchdog", 
                                        128, 
                                        NULL, 
                                        1, 
                                        &wdt_handle);
    output_queue = xQueueCreate(10, sizeof(uint16_t));

    log_debug("Starting rtos scheduler");
    if (pico_status == pdPASS && gpio_status == pdPASS && as5600_status == pdPASS) {
        vTaskStartScheduler();
    }

    log_debug("Why am I here?");
    return 0;
}