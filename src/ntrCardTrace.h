#pragma once

// Non-intrusive cartridge trace (see docs/uart-log-non-intrusive-cart-trace-design.md).
//
// The trace is a pure side channel: a dedicated PIO0 state machine observes the
// physical CEB/WREB/D0..D7 pins, an IRQ-less DMA channel stores the raw command
// words into an SRAM ring, and core1 decodes them and emits bounded UART text.
// No function in this interface may be called from PIO0_IRQ_0, the GPIO/DMA/
// timer IRQs or the E3/E4/E5 handlers.

#include <stdbool.h>
#include <stdint.h>

#if defined(ENABLE_UART_LOG) && defined(ENABLE_CART_TRACE)
#define NTRC_TRACE_ENABLED 1
#else
#define NTRC_TRACE_ENABLED 0
#endif

#ifdef __cplusplus
extern "C" {
#endif

#if NTRC_TRACE_ENABLED

// Fixed-size SD/cache snapshot. Published by core0 (never from an IRQ) using a
// seqlock commit sequence; core1 is the only reader. `requestGeneration`
// advances whenever core0 observes a new E3 sector request.
typedef struct
{
    uint32_t sdState;
    uint32_t sdSequentialState;
    uint32_t sdSectorAddress;
    uint32_t sdSectorsCompleted;
    uint32_t sdSectorCount;
    uint32_t cacheSector0;
    uint32_t cacheSector1;
    uint32_t cacheIndex;
    uint32_t requestGeneration;
    uint32_t errorCode;
} ntrCardTraceSnapshot;

// Starts the PIO listener and the trace DMA channel. Call once, after the
// service SM (PIO0 SM0) and SDIO are initialized. Never called from an IRQ.
void ntrCardTraceInit(void);

// Core1 side: drains the trace ring, decodes records and pumps the bounded
// non-blocking UART output. Call from core1's idle loop.
void ntrCardTraceCore1Poll(void);

// Core0 side: publishes a consistent snapshot for core1 to correlate with the
// decoded command stream. Called from the main loop only.
void ntrCardTracePublishSnapshot(const ntrCardTraceSnapshot* snapshot);

// Core0 side: fills out[0..1] with the two cached sector numbers, out[2] with
// the active buffer index and out[3] with a busy bitmask. Read-only accessor
// for the snapshot publisher; the cartridge IRQ remains its only writer.
void ntrCardTraceGetCacheInfo(uint32_t out[4]);

// Cache-slot state owned by ntrCardRomGameSd.cpp. The non-intrusive snapshot
// publisher reads them from core0; the E3/E4/E5 handlers stay their only
// writer, so this export adds no work to the real-time path.
extern uint32_t sSdSectorBuffersSectors[2];
extern uint32_t sBufferIndex;
extern bool sReadBusy;
extern bool sWriteBusy;

#endif // NTRC_TRACE_ENABLED

#ifdef __cplusplus
}
#endif
