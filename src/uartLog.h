#pragma once

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// Initializes UART1 on RP2354A USB_DP/USB_DM and the non-blocking log queue.
void uartLogInit(void);

// Formats one message into the SRAM queue. This never waits for UART TX.
void uartLogPrintf(const char* format, ...)
    __attribute__((format(printf, 1, 2)));

// Moves as many queued bytes as fit into the UART FIFO without waiting.
// Returns true while queued bytes remain, so the main loop can keep pumping.
bool uartLogDrain(void);

// Deadlock-safe drain for IRQ context (bounded try-lock; see uartLog.c).
// Diagnostic heartbeats only - normal runtime draining stays on uartLogDrain().
bool uartLogDrainFromIrq(void);

// Drains the complete queue and waits for the UART shifter to finish. This is
// only for boot-time milestones immediately before a potentially blocking
// hardware operation; runtime logging must continue to use uartLogDrain().
void uartLogFlush(void);

// Writes a fixed boot diagnostic directly to UART1 and waits for completion.
// This bypasses the ring and is only safe before timing-critical interfaces
// start (or in a terminal fault path).
void uartLogPutsBlocking(const char* text);

// Formats a boot diagnostic and writes it directly to UART1. Like the fixed
// string variant, this bypasses the shared queue and is intentionally
// blocking; do not use it once timing-critical runtime service has started.
void uartLogPrintfBlocking(const char* format, ...)
    __attribute__((format(printf, 1, 2)));

#ifdef __cplusplus
}
#endif
