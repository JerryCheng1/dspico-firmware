#pragma once
// Standalone, hardware-free event ring for the design stage S1 init log
// (section 9.2). Kept free of any pico/stdlib include so it can be built and
// fuzzed by the host test in tests/host/cacheInitLogRingTest.c.
//
// Properties the design requires:
//   * fixed 32 B records, 64 slots,
//   * usable capacity is N-1 so head==tail unambiguously means empty,
//   * a full ring never overwrites an unread record,
//   * each rejected push increments dropped exactly once and remembers the
//     first dropped record for the fault summary,
//   * FIFO order.

#include <stdint.h>
#include <stdbool.h>

#define CACHE_INIT_LOG_EVENTS 64u

enum
{
    CACHE_LOG_EV_FIFO = 32,
    CACHE_LOG_EV_BOOT = 1,
    CACHE_LOG_EV_RESOURCE,
    CACHE_LOG_EV_INIT_SUMMARY,
    CACHE_LOG_EV_PSRAM,
    CACHE_LOG_EV_FAULT,
    CACHE_LOG_EV_STATS,
    CACHE_LOG_EV_SD_STATS,
    CACHE_LOG_EV_L2_STATE,
    CACHE_LOG_EV_SD_STATE,
    // Categorized attribution (design section 9.1). No single total is used to
    // judge success; these records keep data, timing and benefit separate.
    CACHE_LOG_EV_SD_COUNTS,
    CACHE_LOG_EV_FILL_COUNTS,
    CACHE_LOG_EV_HANDOVER,
    CACHE_LOG_EV_FAULT_FIRST,
    CACHE_LOG_EV_CACHE_HEALTH,
};

typedef struct
{
    uint8_t event;
    uint8_t step;
    uint8_t chip;
    uint8_t result;
    uint32_t elapsedUs;
    uint32_t arg[6];
} cache_init_log_event_t;

_Static_assert(sizeof(cache_init_log_event_t) == 32, "init-log record must stay 32 B");

typedef struct
{
    uint32_t head;
    uint32_t tail;
    uint32_t totalPushed;
    uint32_t dropped;
    uint32_t haveFirstDropped;
    cache_init_log_event_t firstDropped;
    cache_init_log_event_t items[CACHE_INIT_LOG_EVENTS];
} cache_init_log_ring_t;

static inline bool cacheInitLogRingPush(cache_init_log_ring_t* ring,
                                        const cache_init_log_event_t* ev)
{
    uint32_t next = (ring->head + 1u) % CACHE_INIT_LOG_EVENTS;
    if (next == ring->tail)
    {
        if (!ring->haveFirstDropped)
        {
            ring->firstDropped = *ev;
            ring->haveFirstDropped = 1u;
        }
        ring->dropped++;
        return false;
    }
    ring->items[ring->head] = *ev;
    ring->head = next;
    ring->totalPushed++;
    return true;
}

static inline bool cacheInitLogRingPop(cache_init_log_ring_t* ring,
                                       cache_init_log_event_t* out)
{
    if (ring->tail == ring->head)
        return false;
    *out = ring->items[ring->tail];
    ring->tail = (ring->tail + 1u) % CACHE_INIT_LOG_EVENTS;
    return true;
}
