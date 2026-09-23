#pragma once
// PSRAM sector cache backend for the extended SD (E3/E4/E5) path.
// Implements the request/job/completion + slot lifecycle of
// docs/cache-fix-design.md on top of the hardware-independent state machine in
// cacheSlots.h. IRQ entry points only touch fixed-size metadata; the core0
// main-loop service (cacheSdStep) owns the SD and PSRAM I/O.
//
// This header and cacheSd.c are deliberately free of any pico/SDK include: the
// hardware is reached through the cacheSdHw adapter below, so the host tests
// (tests/host) drive the exact production logic with mocked SD/PSRAM/DMA/IRQ.
// The target adapter is installed by cacheSdInit() when none was set.

#include <stdint.h>
#include <stdbool.h>
#include "cacheSlots.h"

#if CACHE_STAGE >= 3
#define CACHE_SD_ENABLED 1
#else
#define CACHE_SD_ENABLED 0
#endif

// 4096 sectors x 512 B = 2 MiB of PSRAM data spread over the four devices.
#define CACHE_SD_SECTORS 4096u
#define PSRAM_CACHE_FRAG_BYTES 32u

// Core0 main-loop service result (design 7.2). DEMAND_PUBLISHED explicitly means
// this round published a demand response and started no background transport.
typedef enum
{
    CACHE_SD_STEP_IDLE = 0,
    CACHE_SD_STEP_WORKED = 1,
    CACHE_SD_STEP_DEMAND_PUBLISHED = 2,
} cacheSdStepResult;

// Hardware adapter. Every field is mandatory; the host tests replace the whole
// table to inject SD/PSRAM/DMA events and failures.
typedef struct
{
    // Start an asynchronous SD read of one sector into dst. False when the
    // engine is not free.
    bool (*sdBeginRead)(uint8_t* dst, uint32_t sector);
    // SD engine idle (independent of whether *our* transfer succeeded).
    bool (*sdReady)(void);
    // Monotonic token of the most recently started SD transfer.
    uint32_t (*sdTransferId)(void);
    // PSRAM quad access, already fragmented to <= PSRAM_CACHE_FRAG_BYTES.
    bool (*psramRead)(uint32_t chip, uint32_t addr, void* buf, uint32_t len);
    bool (*psramWrite)(uint32_t chip, uint32_t addr, const void* buf, uint32_t len);
    // DMA0 (cartridge response feed) busy.
    bool (*dma0Busy)(void);
    // The cartridge bus is idle (no command being clocked). Used for the
    // T_IDLE_ADMIT gate before starting a new background fragment (design 7.4).
    bool (*cartIdle)(void);
    // The most recently completed SD transfer reported an error (design 11).
    bool (*sdError)(void);
    // Interrupt state save/restore for the short metadata critical sections.
    uint32_t (*irqSave)(void);
    void (*irqRestore)(uint32_t);
    uint64_t (*nowUs)(void);
} cacheSdHw;

#ifdef __cplusplus
extern "C" {
#endif

// Categorized counters (design section 9.1). Exposed instead of one legacy
// error total so data correctness, timing and benefit are judged separately.
typedef struct
{
    uint32_t sd_token_obsolete;
    uint32_t sd_token_live_conflict;
    uint32_t sd_transfer_error;
    uint32_t l2_hit_attempt;
    uint32_t l2_hit_verified;
    uint32_t l2_crc_fail;
    uint32_t psram_read_fail;
    uint32_t psram_fill_fail;
    uint32_t fill_admit;
    uint32_t fill_drop;
    uint32_t fill_defer;
    uint32_t fill_commit;
    uint32_t fill_quarantine;
    uint32_t fill_drop_reason;   // last drop reason (CS_FILL_DROP_*)
    // Runtime diagnostics: approximate cross-core samples, not a transaction.
    uint32_t mode, enabled, degraded, epoch, sd_miss;
    uint32_t demand_hit_try, demand_hit_ok, probe_try, probe_ok;
    uint32_t read_bytes, write_bytes; // successful transport fragments, cumulative
    uint32_t fill_state, fill_sector, fill_offset, snapshot;
    uint32_t gate_checks, gate_blocked, gate_mask; // WATCH/HOST; 1/1024 steps sampled
    uint32_t gate_measured; // 0: only a software-state observation; counts unavailable
} cacheSdCounters;

enum {
    CACHE_GATE_INTENT = 1u, CACHE_GATE_COMPLETION = 2u,
    CACHE_GATE_BINDING = 4u, CACHE_GATE_WRITE = 8u,
    CACHE_GATE_BUS = 16u, // DMA0/bus busy when software foreground did not short-circuit
    CACHE_GATE_JOB = 32u, CACHE_GATE_PUBLISH = 64u, CACHE_GATE_BUDGET = 128u
};

// A read-only summary of the last buffer submitted to DMA0 (not wire proof).
typedef struct { uint32_t sends, sector, head, tail, offer; } cacheSdTxSnapshot;

#if CACHE_SD_ENABLED

void cacheSdSetHw(const cacheSdHw* hw);
void cacheSdInit(void);
void cacheSdSetEnabled(bool enabled);

// Diagnostic mode (design section 10). M0/M1 isolate bring-up, M2 the snapshot
// cost, M3 the backfill coexistence, M4 the transfer/generation check and M5
// the real hit path. Values default to CACHE_L2_MODE from the build.
enum
{
    CACHE_L2_MODE_M0 = 0,  // OFF: no init, SD only
    CACHE_L2_MODE_M1 = 1,  // INIT_ONLY: init, no snapshot/fill/hit
    CACHE_L2_MODE_M2 = 2,  // SNAPSHOT_ONLY: snapshot copy, no transport
    CACHE_L2_MODE_M3 = 3,  // FILL_ONLY: snapshot + PSRAM write, no hit
    CACHE_L2_MODE_M4 = 4,  // READ_PROBE: fill + verified read probe
    CACHE_L2_MODE_M5 = 5,  // FULL
};
void cacheSdSetMode(uint8_t mode);
uint8_t cacheSdMode(void);

// Time budgets in microseconds (design 7.4). These are source-parameterized and
// MUST be replaced with measured bounds before final acceptance. fragment_us /
// cleanup_us are recorded here but their qualification is D4 (finite hardware
// transport); passing 0 disables that particular gate.
void cacheSdSetTimeBudgets(uint32_t step_us, uint32_t fragment_us,
                           uint32_t cleanup_us, uint32_t idle_admit_us);

// Read-only IRQ fast status. Published only after cs_commit accepts a complete
// sector; cleared on replacement, write, reset, media change and consumption.
// E4 must call cacheSdPollReady AFTER queuing ready=1 to establish its binding.
// Both run in the same highest-priority cartridge IRQ: no main-loop writer or
// other cartridge command can invalidate this observation between them.
extern volatile uint32_t gCacheSdReadReady;

// Slot buffer handed to E5/DMA0. Caller must hold the binding returned by
// cacheSdConsume().
uint8_t* cacheSdSlotBuffer(uint8_t slot);

// Cartridge IRQ entry points (bounded; no I/O, no CRC, no directory scan).
uint32_t cacheSdRequest(uint32_t sector);                 // E3
// E4 entry point (bounded, no I/O). \p queued_ready is the ready value that
// was ALREADY pushed into the PIO TX FIFO before this call. Returns true when a
// valid binding for the current offer was established. A queued ready that
// cannot bind is recorded as the first E4_ACK_MISMATCH (design section 6.1).
bool cacheSdPollReadySampled(bool queued_ready);
// Compatibility wrapper: samples the current IRQ-visible fast status.
bool cacheSdPollReady(void);
// Offer identity of the live binding/ack (0 when none).
uint32_t cacheSdOfferId(void);
uint32_t cacheSdAckedOfferId(void);
// Frozen first abnormal handover record (design section 9.2).
const cs_fault_record* cacheSdFirstFault(void);

void cacheSdGetCounters(cacheSdCounters* out);
// Cartridge IRQ only: E4 grants a validated lease; E5 takes it once, pins
// the slot, and revokes ready before the caller starts DMA. No main-loop use.
uint8_t* cacheSdTakeAckedFromIrq(uint32_t* sector);
void cacheSdRejectSendFromIrq(void);
void cacheSdRecordSendFromIrq(uint32_t sector, const uint8_t* data);
const volatile cacheSdTxSnapshot* cacheSdLastTx(void);
bool cacheSdConsume(uint32_t* sector, uint8_t* slot, uint32_t* slot_version); // E5
void cacheSdDmaDone(void);                                // explicit DMA0 completion

// Write lifecycle (design section 8.1). First call of a write sequence returns
// a token and blocks reads; continuation blocks reuse it. cacheSdWriteEnd()
// with the same token releases the barrier once the whole sequence is durable.
uint32_t cacheSdWriteBegin(void);
void cacheSdWriteEnd(uint32_t token);

// Session/medium generation changes (design section 9.1).
void cacheSdResetCart(void);
void cacheSdSetMediaEpoch(uint32_t media_epoch);

// Core0 main-loop service. Returns a bounded work result (design 7.2): the
// DEMAND_PUBLISHED result explicitly tells the caller that this round published
// a demand response and therefore started NO background transport.
cacheSdStepResult cacheSdStep(void);

// Derived fast-ready (design 6.2). True only while the response descriptor is
// PREPARED/ACKED with a matching identity and no pending write barrier. E4 must
// use this value, never an independent flag.
bool cacheSdFastReady(void);

// E5 SENDING support. The precheck is available to diagnostics/tests; the
// IRQ queues its length first and uses cacheSdConsume() before feeding data. cacheSdMarkE5Recovery() records that the E5
// entered the bounded transaction recovery (no data was fed; the CEB-rise
// handler resynchronises).
bool cacheSdE5Precheck(void);
void cacheSdMarkE5Recovery(void);

// L2 degrade / restore (design section 11).
void cacheSdDegrade(uint32_t reason);
bool cacheSdDegraded(void);

// Backfill lifecycle / drop reason names for diagnostics (design 7.1).
const char* cacheSdFillStateName(void);
uint32_t cacheSdFillDropReason(void);

// Diagnostics.
uint32_t cacheSdHits(void);
uint32_t cacheSdMisses(void);
uint32_t cacheSdFills(void);
uint32_t cacheSdErrors(void);
uint32_t cacheSdSectors(void);                 // live entries in current epoch
const cs_state* cacheSdState(void);

#else
static inline uint8_t* cacheSdTakeAckedFromIrq(uint32_t* s) { (void)s; return 0; }
static inline void cacheSdRejectSendFromIrq(void) {}
static inline void cacheSdRecordSendFromIrq(uint32_t s, const uint8_t* d) { (void)s; (void)d; }
static inline const volatile cacheSdTxSnapshot* cacheSdLastTx(void) { return 0; }
static inline void cacheSdSetHw(const cacheSdHw* hw) { (void)hw; }
static inline void cacheSdInit(void) {}
static inline void cacheSdSetEnabled(bool e) { (void)e; }
static inline void cacheSdSetMode(uint8_t m) { (void)m; }
static inline uint8_t cacheSdMode(void) { return 0; }
static inline void cacheSdSetTimeBudgets(uint32_t a, uint32_t b, uint32_t c, uint32_t d) { (void)a; (void)b; (void)c; (void)d; }
static inline bool cacheSdFastReady(void) { return false; }
static inline bool cacheSdE5Precheck(void) { return false; }
static inline void cacheSdMarkE5Recovery(void) {}
static inline void cacheSdDegrade(uint32_t r) { (void)r; }
static inline bool cacheSdDegraded(void) { return false; }
static inline const char* cacheSdFillStateName(void) { return "NONE"; }
static inline uint32_t cacheSdFillDropReason(void) { return 0; }
static inline uint8_t* cacheSdSlotBuffer(uint8_t slot) { (void)slot; return 0; }
static inline uint32_t cacheSdRequest(uint32_t s) { (void)s; return 3; }
static inline bool cacheSdPollReady(void) { return false; }
static inline bool cacheSdPollReadySampled(bool r) { (void)r; return false; }
static inline uint32_t cacheSdOfferId(void) { return 0; }
static inline uint32_t cacheSdAckedOfferId(void) { return 0; }
static inline const cs_fault_record* cacheSdFirstFault(void) { return 0; }
static inline void cacheSdGetCounters(cacheSdCounters* o) { if (o) { uint32_t* p = (uint32_t*)o; for (unsigned i = 0; i < sizeof(*o)/4u; i++) p[i] = 0; } }
static inline bool cacheSdConsume(uint32_t* s, uint8_t* sl, uint32_t* v) { (void)s; (void)sl; (void)v; return false; }
static inline void cacheSdDmaDone(void) {}
static inline uint32_t cacheSdWriteBegin(void) { return 0; }
static inline void cacheSdWriteEnd(uint32_t t) { (void)t; }
static inline void cacheSdResetCart(void) {}
static inline void cacheSdSetMediaEpoch(uint32_t e) { (void)e; }
static inline cacheSdStepResult cacheSdStep(void) { return CACHE_SD_STEP_IDLE; }
static inline uint32_t cacheSdHits(void) { return 0; }
static inline uint32_t cacheSdMisses(void) { return 0; }
static inline uint32_t cacheSdFills(void) { return 0; }
static inline uint32_t cacheSdErrors(void) { return 0; }
static inline uint32_t cacheSdSectors(void) { return 0; }
static inline const cs_state* cacheSdState(void) { return 0; }
#endif

#ifdef __cplusplus
}
#endif
