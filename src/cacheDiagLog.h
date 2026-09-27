#pragma once
#include <stddef.h>
#include "cacheSd.h"
#ifdef __cplusplus
extern "C" {
#endif

// Fixed-work streaming formatter: each call emits at most one character and
// converts at most one uint32 (10 digits). No printf, allocation or device I/O.
// The caller owns the snapshot and must not replace it until the stream ends.
typedef struct {
    const char* format;
    uint32_t values[96];
    unsigned count, index, digits_left, zeroes;
    char digits[10];
    bool negative;
} cacheTextStream;
void cacheTextStart(cacheTextStream* s, const char* format, unsigned count);
int cacheTextNext(cacheTextStream* s); // byte or -1 at end
void cacheDiagStartLine(cacheTextStream* s, unsigned line, uint32_t sample,
                        const cacheSdCounters* c);
// One-line user view. Capacity is usable payload, occupancy counts currently
// valid 512-byte sectors, and the hit denominator counts committed demands.
void cacheSummaryStartLine(cacheTextStream* s, uint32_t capacity_bytes,
                           const cacheSdCounters* c);

// Core1-only admission policy. Activity is sampled from existing counters;
// no writer is added to the cartridge IRQ. This is not a reserved bus window.
#define CACHE_WATCH_BOOT_QUIET_US 5000000u
#define CACHE_WATCH_IDLE_US 200000u
#define CACHE_WATCH_STALLED_US 5000000u
#define CACHE_WATCH_POLL_US 1000u
typedef struct {
    uint32_t boot_us, changed_us, progress_us, activity[7];
    bool started, warm, observed;
} cacheWatchGate;
bool cacheWatchGateWarm(cacheWatchGate* g, uint32_t now);
bool cacheWatchGateIdle(cacheWatchGate* g, uint32_t now, bool available,
                        const uint32_t activity[7]);
// Call after GateIdle. Unlike normal admission, held-low CS must not hide
// a stopped host forever. This is a diagnostic timeout, not a bus lease.
bool cacheWatchGateStalled(const cacheWatchGate* g, uint32_t now);
#ifdef __cplusplus
}
#endif
