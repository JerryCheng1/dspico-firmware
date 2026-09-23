#pragma once
// Hardware-independent read-request / completion / slot state machine for the
// E3/E4/E5 path (docs/cache-fix-design.md sections 5 and 6).
//
// It is deliberately free of any pico/SDK include so the host tests build and
// drive the exact production logic. All functions are bounded: fixed-size
// field copies and small loops over CS_SLOT_COUNT, no I/O, no logs, no CRC.
//
// Concurrency model on target: the cartridge IRQs (E3/E4/E5, write barrier)
// run at the highest priority and call the IRQ-entry functions directly; the
// core0 main loop calls the backend-entry functions inside a short
// save/disable_interrupts section. There is only ever one writer per entry at a
// time, so no spinlock/queue is needed.

#include <stdint.h>
#include <stdbool.h>

// IRQ entry points and metadata critical sections must not fetch code from
// XIP while the cartridge response is on a deadline. Host tests stay ordinary
// C; target functions are copied into main SRAM by the SDK linker script.
#ifdef PICO_ON_DEVICE
#define CACHE_RAM_CODE __attribute__((section(".time_critical.cache")))
#else
#define CACHE_RAM_CODE
#endif

#define CS_SLOT_COUNT 4u
#define CS_SECTOR_BYTES 512u

// Bounded retry budget for a demand job whose SD transfer was superseded by
// another engine owner (design section 13.5 / T24). After this many attempts
// the intent is latched as a diagnosable terminal fault instead of spinning.
#define CS_MAX_RETRY 8u

typedef struct
{
    uint32_t request_id;   // one logical host request; E4 polls do not bump it
    uint32_t cart_epoch;   // cartridge reset / protocol session
    uint32_t media_epoch;  // medium identity / re-init
    uint32_t write_epoch;  // write barrier generation
    uint32_t sector;
} cs_identity;

typedef struct
{
    cs_identity id;        // immutable snapshot taken at accept time
    uint32_t job_id;
    uint32_t slot_version;
    uint32_t source_epoch; // namespace generation the source was valid under
    uint8_t slot;
} cs_job;

enum { CS_RES_OK = 0, CS_RES_CANCELLED = 1, CS_RES_FAULT = 2 };

typedef struct
{
    cs_identity id;
    uint32_t job_id;
    uint32_t slot_version;
    uint8_t slot;
    uint8_t result;
    // Immutable publication identity (design section 6.2). Assigned by
    // cs_commit() on the stored copy; never zero for a live offer. E4 records
    // the offer it acknowledged and E5 must match the same value.
    uint32_t offer_id;
} cs_completion;

// First-fault classification (design section 9.2). A fixed-size SRAM record of
// the FIRST abnormal handover decision is frozen so a later completion cannot
// overwrite the reason. Kinds are stable numbers, not a bitmask.
enum
{
    CS_FAULT_NONE = 0,
    CS_FAULT_E4_ACK_MISMATCH = 1,
    CS_FAULT_E5_NO_ACK = 2,
    CS_FAULT_E5_IDENTITY = 3,
    CS_FAULT_E5_SLOT = 4,
    CS_FAULT_SD_TOKEN = 5,
    CS_FAULT_L2_CRC = 6,
    CS_FAULT_PSRAM = 7,
    CS_FAULT_FILL = 8,
    CS_FAULT_E5_RECOVERY = 9,
    CS_FAULT_DEGRADE = 10,
};

// Which context produced a record. The design (9.2) forbids treating the main
// loop and the cartridge IRQ as co-writers of one lock-free ring.
enum
{
    CS_PRODUCER_MAIN = 0,
    CS_PRODUCER_IRQ_E4 = 1,
    CS_PRODUCER_IRQ_E5 = 2,
    CS_PRODUCER_IRQ_OTHER = 3,
};

// Explicit response lifecycle (design section 6.2).
//   EMPTY -> FILLING -> PREPARED -> ACKED -> SENDING -> RETIRED.
enum
{
    CS_RESP_EMPTY = 0,
    CS_RESP_FILLING,   // backend owns a reserved slot for the live intent
    CS_RESP_PREPARED,  // completion committed, non-zero offer published
    CS_RESP_ACKED,     // E4 queued ready and recorded acked_offer_id
    CS_RESP_SENDING,   // E5 handed the slot to DMA0
    CS_RESP_RETIRED,   // DMA0 finished; slot released
};

// Why a published/partial response was revoked (design 6.1 "最后一次失效原因").
enum
{
    CS_INV_NONE = 0,
    CS_INV_INTENT_REPLACED = 1,
    CS_INV_WRITE_BARRIER = 2,
    CS_INV_RESET = 3,
    CS_INV_MEDIA = 4,
    CS_INV_CONSUMED = 5,
    CS_INV_SLOT_MISMATCH = 6,
    CS_INV_COMMIT_OBSOLETE = 7,
    CS_INV_MAILBOX_STALE = 8,
    CS_INV_DEGRADE = 9,
};

typedef struct
{
    uint8_t valid;
    uint8_t kind;
    uint8_t producer;
    uint8_t resp_state;
    uint8_t last_invalidate;
    uint8_t write_pending;
    uint8_t slot;
    uint8_t intent_consumed;
    uint32_t seq;                // fault counter value at capture (1-based)
    cs_identity id;              // request/cart/media/write epoch + sector
    uint32_t offer_id;           // offer involved (0 when none)
    uint32_t acked_offer_id;     // what E4 acknowledged (0 when none)
    uint32_t sampled_ready;      // E4 value already queued into the FIFO
    uint32_t completion_valid;   // mailbox state at capture
    uint32_t binding_valid;
    uint32_t slot_version;
    uint32_t detail;             // kind-specific (sector/slot/epoch/token)
    // Caller-supplied context, e.g. SD old/new token or fill state/offset.
    uint32_t ctx[2];
} cs_fault_record;

// E3 result.
enum
{
    CS_INTENT_MERGED = 0,
    CS_INTENT_REPLACED = 1,
    CS_INTENT_QUEUED = 2,
    CS_INTENT_FAULT = 3,
};

// Slot states (design section 7).
enum
{
    CS_SLOT_FREE = 0,
    CS_SLOT_RESERVED,   // backend owns; filling from SD/L2
    CS_SLOT_READY,      // completion committed; visible to E4
    CS_SLOT_DMA,        // pinned by E5 / DMA0 in flight
};

typedef struct
{
    uint8_t state;
    uint32_t version;      // bumped on every allocation
    uint32_t request_id;   // current owner's request id
} cs_slot;

typedef struct
{
    // Frontend mailbox: the latest host read intent.
    cs_identity intent;
    uint8_t intent_valid;
    uint8_t intent_consumed; // backend already snapshotted it into a job

    // Binding: completion turned into a pinnable response for the host.
    cs_completion binding;
    uint8_t binding_valid;

    // Single completion mailbox (capacity 1, not overwritten until consumed).
    cs_completion completion;
    uint8_t completion_valid;

    uint32_t next_request_id;
    uint32_t next_job_id;
    uint32_t next_slot_version;
    uint32_t next_offer_id;

    // Lightweight E4 acknowledgement (design section 6.2). Non-zero only after
    // E4 queued a ready=1 for the offer carried by the current binding.
    uint32_t acked_offer_id;

    cs_slot slots[CS_SLOT_COUNT];

    uint32_t cart_epoch;
    uint32_t media_epoch;
    uint32_t write_epoch;

    // Write lifecycle (design section 8.1). A write intent is accepted at
    // cs_write_begin() and stays pending until the matching cs_write_end(),
    // blocking every new read publication and backfill in between.
    uint32_t write_token;
    uint32_t next_write_token;
    uint8_t write_pending;

    // Terminal request fault (bounded SD retries exhausted).
    uint8_t intent_faulted;
    uint32_t retry_count;

    // Diagnostics (design section 12.1).
    uint32_t c_intent_merged;
    uint32_t c_intent_replaced;
    uint32_t c_completion_obsolete;
    uint32_t c_slot_wait;
    uint32_t c_protocol_fault;
    uint32_t c_barrier_reject;
    uint32_t c_slot_reuse;       // allocations of a previously used slot
    uint32_t c_write_blocked;    // accept/commit refused while a write is pending
    uint32_t c_sd_retry;         // job requeued after an SD-owner token change
    uint32_t c_sd_fault;         // intent latched as a terminal fault
    uint32_t c_dma_reclaim;      // DMA0-owned slots reclaimed at a safe boundary

    // Categorized counters (design section 9.1). These replace reasoning from
    // the single legacy error total. They are separate so data correctness,
    // protocol timing and cache benefit can be judged independently.
    uint32_t c_e4_busy;              // E4 sampled busy
    uint32_t c_e4_ready_queued;      // E4 queued ready and the ack bound
    uint32_t c_e4_ack_failed;        // ready was queued but the binding failed
    uint32_t c_e5_ok;                // E5 consumed a fully valid ack
    uint32_t c_e5_no_ack;            // E5 without a same-offer E4 ack
    uint32_t c_e5_identity_mismatch; // binding no longer matches the intent
    uint32_t c_e5_slot_mismatch;     // slot state/version no longer pinnable
    uint32_t c_fifo_recovery;        // pre-armed length dropped without clear
    uint32_t c_offer_assign;         // offers assigned by cs_commit()
    uint32_t c_offer_retire;         // offers retired/consumed

    cs_fault_record first_fault;
    uint32_t fault_seq;              // total faults captured (first is frozen)

    // Explicit response lifecycle and last revocation reason (design 6.1/6.2).
    uint8_t resp_state;
    uint8_t last_invalidate_reason;
    // L2 data path degraded: no new hit/fill admission, published responses are
    // kept and SD service continues (design section 11).
    uint8_t degraded;
    uint32_t c_e5_recovery;          // E5 rejects that entered bounded recovery

} cs_state;

void cs_init(cs_state* s);

// ---- Frontend (cartridge IRQ context) -------------------------------------
uint32_t cs_read_intent(cs_state* s, uint32_t sector);          // E3
bool cs_poll_read(cs_state* s);                                  // E4
bool cs_consume_read(cs_state* s, uint32_t* sector, uint8_t* slot,
                     uint32_t* slot_version);                    // E5
void cs_dma_done(cs_state* s, uint8_t slot, uint32_t slot_version);

// ---- Backend (core0 main loop) --------------------------------------------
// Reserve a slot for the current intent and snapshot it. False when the intent
// was replaced/consumed or no slot is free.
bool cs_accept_intent(cs_state* s, cs_job* job);
// Publish a completed+verified job. False when it is obsolete (replaced intent,
// newer write barrier, wrong slot) or the completion mailbox is occupied.
bool cs_commit(cs_state* s, const cs_completion* c);
// Retire a job that will never produce a usable result (cancel/fault).
void cs_job_abort(cs_state* s, const cs_job* job);

// Abandon a job but, when it still maps to the live intent, let the intent be
// retried under the current generations. Returns true when the intent was
// re-armed for another attempt, false when the job was simply retired (a newer
// intent superseded it, or the bounded retry budget was exhausted and the
// intent is now latched as a terminal fault).
bool cs_job_requeue(cs_state* s, const cs_job* job);

// ---- Write lifecycle (design section 8.1) ---------------------------------
// Accept a write intent: bump the write barrier generation, drop any pending
// read binding/completion and reserve the generation for the whole write
// sequence. A second call while a write is pending reuses the same token and
// does not advance the generation again. Returns a non-zero token.
uint32_t cs_write_begin(cs_state* s);
// Confirm completion of the write sequence. Only the matching token clears the
// pending state; a stale token is ignored.
void cs_write_end(cs_state* s, uint32_t token);
bool cs_write_is_pending(const cs_state* s);
bool cs_is_faulted(const cs_state* s);

// Record the FIRST diagnostic fault (design section 9.2). The frozen record is
// never overwritten; later faults only advance the sequence counter. Callable
// from IRQ and main-loop context.
void cs_note_fault(cs_state* s, uint8_t kind, uint32_t detail);
// Same but with an explicit producer and up to two caller context words.
void cs_note_fault_ex(cs_state* s, uint8_t kind, uint32_t detail,
                      uint8_t producer, uint32_t ctx0, uint32_t ctx1);
// E4-specific variant: sampled_ready is the value already queued into the PIO
// FIFO before the binding attempt.
void cs_note_e4_ack_failed(cs_state* s, uint32_t sampled_ready);
const cs_fault_record* cs_first_fault(const cs_state* s);

// Fast, non-mutating predicate for the E5 SENDING state (design 6.2): true only
// when the exact ack, identity, write barrier and slot/version all hold. E5
// checks this BEFORE arming a response length, so a rejected E5 cannot emit
// 512 B of garbage; cs_consume_read() then performs the authoritative
// transition and classification.
bool cs_e5_precheck(const cs_state* s);

// L2 degrade / restore (design section 11). Degrade refuses new hits and new
// backfill admission without touching already-published SRAM responses.
void cs_set_degraded(cs_state* s, bool degraded);
bool cs_is_degraded(const cs_state* s);
const char* cs_response_state_name(const cs_state* s);

// ---- Epochs ---------------------------------------------------------------
void cs_reset_cart(cs_state* s);
void cs_set_media_epoch(cs_state* s, uint32_t media_epoch);

// ---- Helpers --------------------------------------------------------------
static inline __attribute__((always_inline)) bool cs_identity_equal(const cs_identity* a, const cs_identity* b)
{
    return a->request_id == b->request_id &&
           a->cart_epoch == b->cart_epoch &&
           a->media_epoch == b->media_epoch &&
           a->write_epoch == b->write_epoch &&
           a->sector == b->sector;
}
