#pragma once
#include "common.h"
#include "cacheInitLogRing.h"

// Design stage S1 / section 9.2: bounded, lock-free boot/init diagnostics built
// on the hardware-free ring in cacheInitLogRing.h.
//
//   * single producer  : core0 (init and main loop) only
//   * single consumer  : cacheInitLogPoll() from the core0 main loop only
//   * push             : fixed 32 B record, no lock, no allocation, no I/O
//   * poll             : at most one record formatted and at most
//                        CACHE_LOG_UART_BYTES_PER_POLL bytes pushed into the
//                        UART FIFO per call, only while the cartridge bus is
//                        idle; a full ring or a stalled UART drops bytes and
//                        never blocks the caller.
//
// Enablement requires CACHE_INIT_LOG (which itself requires ENABLE_UART_LOG);
// with either off every function is an empty no-op so the non-log build's
// handlers stay instruction-identical.

#define CACHE_LOG_RING_BYTES (CACHE_INIT_LOG_EVENTS * 32u + 32u)
#define CACHE_LOG_UART_BYTES_PER_POLL 16u

#if CACHE_STAGE >= 1 && defined(CACHE_INIT_LOG) && defined(ENABLE_UART_LOG)
#define CACHE_INIT_LOG_ENABLED 1
#else
#define CACHE_INIT_LOG_ENABLED 0
#endif

#ifdef __cplusplus
extern "C" {
#endif

void cacheInitLogInit(void);

// Enqueue one record. Returns false when the ring is full (the record is
// dropped and counted). Safe to call from core0 init and main loop only.
bool cacheInitLogEmit(u8 eventId, u8 step, u8 chip, u8 result, u32 elapsedUs,
                      u32 a, u32 b, u32 c, u32 d, u32 e, u32 f);

// Format/drain at most one record into the UART. No-op in stage-0 builds.
void cacheInitLogPoll(void);

// Emit a low-rate stats record when CACHE_STATS_LOG is set (default OFF).
void cacheInitLogStatsStep(void);

u32 cacheInitLogDropped(void);
u32 cacheInitLogPushed(void);

#ifdef __cplusplus
}
#endif
