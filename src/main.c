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

#define INPUT_PIN 28
#define OUTPUT_PIN 29

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

#define AS5600_BITS_TO_DISCARD 2

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
            //log_debug("got led data: %d", led_status);
            if (last_status != led_status) {
                is_on = 0;
                counter = 0;
            }
            last_status = led_status;
        }
        put_pixel(is_on ? 0 : status_to_color(last_status));

        counter = (counter + 1) % 3;
        if ((last_status != STATUS_FORWARD && last_status != STATUS_BACKWARD) || counter == 0) {
            is_on = !is_on;
        }
    }
}

void wdt_task(void* unused_arg) {
    for (;;) {
        watchdog_update();
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

void input_pin_isr(uint gpio, uint32_t events) {
    gpio_acknowledge_irq(gpio, events);
    BaseType_t task_woken = pdFALSE;
    xTaskNotifyFromISR(input_task_handle, 1, eSetBits, &task_woken);
    portYIELD_FROM_ISR(task_woken);
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
        if (xQueueReceive(output_queue, &output_word, pdMS_TO_TICKS(500))) {
            //log_debug("sending output 0x%04x", output_word);
            TickType_t xNow = xTaskGetTickCount();
            _send_word(&xNow, output_word);
            // give the receiver some time to recover after sending a word
            vTaskDelay(pdMS_TO_TICKS(10));
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

void as5600_position_task(void* unused_arg) {
    uint16_t last_angle = 0;
    uint8_t direction = STATUS_FORWARD;

    for (;;) {
        // vTaskDelay(pdMS_TO_TICKS(2000));
        send_word(VersionBits | FIRMWARE_VERSION);
        // vTaskDelay(pdMS_TO_TICKS(10));
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

	//if (!as5600_set_current_zero_position()) {
    //    log_debug("could not set zero position");
    //}
    
    TickType_t xLastWakeTime = xTaskGetTickCount();
    TickType_t xLastPositionTime;
    TickType_t xLastStatusTime;
    bool status_good = false;
    bool send_status = false;
    for (;;) {
        xTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(40));
        uint8_t status = as5600_get_status();
        if (status == AS5600_MAGNET_DETECTED) {
            // drop 2 bits and take the lower 10bits for a range of 0-1024
            uint16_t angle = (as5600_get_angle() >> AS5600_BITS_TO_DISCARD) & 0x3ff;
            if (last_angle != angle) {
                // TODO simplify
                if (angle > last_angle) {
                    // detect wraparound
                    direction = abs(angle - last_angle) > 500 ? STATUS_BACKWARD : STATUS_FORWARD;
                } else {
                    // detect wraparound
                    direction = abs(angle - last_angle) > 500 ? STATUS_FORWARD : STATUS_BACKWARD;
                }
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
            last_angle = angle;
            status_good = true;
        } else {
            if (status_good) {
                // send status immediately when the as5600 state goes bad
                send_status = true;
            }
            status_good = false;
        }

        if (send_status || pdMS_TO_TICKS(500) <= xLastWakeTime - xLastStatusTime) {
            if (!status_good) {
                log_debug("sensor in a bad status, send status %d", status);
                report_error(status_to_err(status));
            }
            log_debug("Sending status");
            send_word(VersionBits | FIRMWARE_VERSION);
            send_word(AgcBits | as5600_get_agc());
            send_word(MagnitudeBits | ((as5600_get_magnitude() >> 4) & 0x00ff));

            send_status = false;
            xLastStatusTime = xLastWakeTime;
        }
    }
}

typedef enum {
    input_unknown,
    input_start,
    input_bits,
    input_done,
    input_none
} input_state;

void input_pin_task(void* unused_arg) {
    TickType_t xPrevWakeTime = 0;
    uint16_t command_word = 0;
    input_state state = input_unknown;
    log_debug("input pin task");

    while (true) {
        uint32_t value = 0;
        if (pdTRUE == xTaskNotifyWait(0, 0, &value, pdMS_TO_TICKS(10))) {
            uint64_t us_time = to_us_since_boot(get_absolute_time());
            TickType_t xWakeTime = xTaskGetTickCount();
            bool is_high = gpio_get(INPUT_PIN);
            command_word = (command_word << 1) | is_high;
            xPrevWakeTime = xWakeTime;
        } else {
            xPrevWakeTime = xTaskGetTickCount();
            state = input_none;
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
    gpio_set_irq_enabled_with_callback(
        INPUT_PIN,
        GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE,
        true,
        &input_pin_isr);

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