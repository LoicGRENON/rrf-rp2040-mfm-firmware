#include <stdlib.h>
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include "debug.h"
#include "pico/platform.h"
#include "pico/time.h"
#include "FreeRTOS.h"
#include <task.h>

void _log_debug(const char* fmt, const char* file, uint32_t line, ...) {
#ifdef DEBUG
    static char core_buffer1[1024];
    static char core_buffer2[1024];
    char *sprintf_buffer = get_core_num() == 0 ? core_buffer1 : core_buffer2;
    va_list argp;
    va_start(argp, line);
    TickType_t now = xTaskGetTickCount();
    snprintf(sprintf_buffer, 1024, "[DEBUG] %d %d.%03d %s:%d %s\n",
        get_core_num(),
        now / 1000, now % 1000,
        file, line, fmt);
    vprintf(sprintf_buffer, argp);
    va_end(argp);
#endif
}