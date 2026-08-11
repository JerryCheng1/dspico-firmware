#pragma once

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// Initializes the GPIO0/1 debug UART and the non-blocking log queue.
void uartLogInit(void);

// Formats one message into the SRAM queue. This never waits for UART TX.
void uartLogPrintf(const char* format, ...)
    __attribute__((format(printf, 1, 2)));

// Moves as many queued bytes as fit into the UART FIFO without waiting.
// Returns true while queued bytes remain, so the main loop can keep pumping.
bool uartLogDrain(void);

#ifdef __cplusplus
}
#endif
