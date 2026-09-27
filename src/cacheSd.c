#include "cacheSd.h"

#if CACHE_SD_ENABLED
#include <string.h>
#ifdef CACHE_FULL_PAGE_4CHIP
#include "cachePageStore.h"
#endif

#ifndef CACHE_SD_HOST
#include "common.h"
#include "hardware/sync.h"
#include "hardware/dma.h"
#include "hardware/gpio.h"
#include "psram.h"
CACHE_RAM_CODE const cacheSdHw* cacheSdDefaultHw(void);
#endif

// ---------------------------------------------------------------------------
// Durability / namespace model
//
//   sCacheEpoch   L2 namespace generation. Bumped once per accepted write
//                 sequence (in lockstep with the cs write generation).
//   source_epoch  captured at job accept time; a fill inherits it and never
//                 reads "the current epoch" as a substitute (design 13.3/R2).
// ---------------------------------------------------------------------------
#ifdef CACHE_FULL_PAGE_4CHIP
static cachePageStore sPageStore;
static cachePageTask sDemandPageTask, sFillPageTask;
static volatile uint32_t sPageFill[4], sPageProbe[4], sPageHitOk[4], sPageMaxSet[4];
#else
static uint32_t sTag[CACHE_SD_SECTORS];
static uint32_t sTagEpoch[CACHE_SD_SECTORS];
static uint32_t sCrc[CACHE_SD_SECTORS];
static uint8_t sState[CACHE_SD_SECTORS];   // 0 empty, 1 filling, 2 ready
#endif
#ifdef CACHE_SHADOW_STRIPES_4CHIP
// Diagnostic address extension: each chip owns 16 x 512 KiB stripes. A
// directory index keeps its 512 B position within a stripe; each newly
// admitted fill rotates to the next stripe of that chip. The game still uses
// SD for every demand. This is coverage instrumentation, not the final 4 KiB
// allocator from design section 4.3.
static uint8_t sStripe[CACHE_SD_SECTORS];
static uint8_t sNextStripe[4];
static uint8_t sFillStripe;
static volatile uint32_t sShadowBanks[4], sShadowVerify[4];
#endif
static uint32_t sCrcTab[256];
static volatile uint32_t sCacheEpoch = 1;

// L0 response slots handed to E5/DMA0.
static uint8_t sSlotBuf[CS_SLOT_COUNT][CS_SECTOR_BYTES] __attribute__((aligned(4)));

// Demand I/O staging: the single buffer every SD/L2 demand read lands in. It
// is copied into the response slot (and the backfill snapshot) only after
// verification, so the SD DMA destination is never the published response.
static uint8_t sDemandStage[CS_SECTOR_BYTES] __attribute__((aligned(4)));
#ifdef CACHE_BOOT_SECTOR0_SEED
// The legacy prewarm buffer is outside stage 3's demand/response lifecycle.
// Keep a separate copy: another sector may be requested before sector 0.
static uint8_t sBootSector0[CS_SECTOR_BYTES] __attribute__((aligned(4)));
static uint8_t sBootSector0Valid;
static volatile uint32_t sBootSeedUse;
#endif

// Independent SD fallback buffer (design section 11). If a PSRAM read failed,
// the demand may only continue from a buffer the failed transport was never
// allowed to touch; the staging buffer is not reused for SD in that case.
static uint8_t sFallbackStage[CS_SECTOR_BYTES] __attribute__((aligned(4)));

// Backfill snapshot, independent of the response slot and of sDemandStage once
// it has been taken (design section 7).
static uint8_t sFillStage[CS_SECTOR_BYTES] __attribute__((aligned(4)));

static cs_state sCs;
static const cacheSdHw* sHw;
static bool sEnabled;
#ifndef CACHE_L2_MODE
#define CACHE_L2_MODE 5
#endif
static uint8_t sL2Mode = (uint8_t)CACHE_L2_MODE;
#ifdef CACHE_M4_PROMOTE_M5_AFTER_PROBES
static uint8_t sM4Promoted;
#endif

// L2 data path degraded (design section 11): no new hit/fill admission, the
// already-published SRAM responses are kept and SD service continues.
static uint8_t sDegraded;

// Demand job (single, main-loop owned).
static cs_job sJob;
static uint8_t sJobActive;
static uint8_t sJobSrc;       // 0 = SD, 1 = L2, 2 = boot sector-0 seed
static uint8_t sJobSdStarted;
static uint8_t sJobUseFallback;
static uint32_t sJobOff;
#ifndef CACHE_FULL_PAGE_4CHIP
static uint32_t sJobCrc;
static uint32_t sJobL2Idx;
#endif
static uint32_t sJobL2Epoch;
static uint32_t sJobSdToken;

// Backfill lifecycle (design section 7.1). A snapshot is taken only after the
// demand response has been published, and its transport is admitted only once
// the response it was derived from has finished its E5/DMA0 transfer. This is
// the H1 isolation: no background PSRAM work is started in the handover
// window or in the same cacheSdStep() that published the demand.
enum
{
    FILL_NONE = 0,
    FILL_SNAPSHOT_PENDING,   // reserved (copy is formed immediately on target)
    FILL_SNAPSHOT_READY,     // independent snapshot taken; waiting for DMA0 end
    FILL_ADMITTED,           // demand transfer ended; transport may start
    FILL_TRANSFERRING,       // fragments in flight
    FILL_VERIFIED,           // whole snapshot written and CRC computed
    FILL_COMMITTED,          // directory committed
    FILL_DROP,               // discarded before commit
    FILL_CANCEL_REQUESTED,   // drop requested mid-transfer
    FILL_CLEANUP,            // bounded wrap-up
    FILL_RETIRED,            // finished, buffer reusable
    FILL_QUARANTINED,        // not proven quiescent; not reused
};
static const char* const sFillStateNames[] = {
    "NONE", "SNAPSHOT_PENDING", "SNAPSHOT_READY", "ADMITTED", "TRANSFERRING",
    "VERIFIED", "COMMITTED", "DROP", "CANCEL_REQUESTED", "CLEANUP", "RETIRED",
    "QUARANTINED",
};

// Why a new backfill was refused / a pending one was dropped (design 7.1).
enum
{
    FILL_DROP_NONE = 0,
    FILL_DROP_NO_SLOT,
    FILL_DROP_EXISTING,
    FILL_DROP_DEMAND,
    FILL_DROP_WINDOW,
    FILL_DROP_SUPERSEDED,
    FILL_DROP_EPOCH,
    FILL_DROP_MODE,
    FILL_DROP_DEGRADE,
    FILL_DROP_BUDGET,
};

static uint8_t sFillState;
static uint8_t sFillIsProbe; // M4 READ_PROBE: verify the entry instead of writing it
#ifndef CACHE_FULL_PAGE_4CHIP
static uint8_t sProbeChunk[PSRAM_CACHE_FRAG_BYTES]; // independent probe buffer
#endif
static uint8_t sFillDropReason;
static uint8_t sFillWaitSlot;
static uint32_t sFillWaitVersion;
static uint8_t sDidPublish;    // a demand was published during this step
// Immutable snapshot identity (design 7.1): sector, source media/write/cache
// epoch, source request/job ID, snapshot id and fragment position.
static uint32_t sFillSector, sFillIdx, sFillOff, sFillCrc, sFillEpoch;
static uint32_t sFillMediaEpoch, sFillWriteEpoch, sFillReqId, sFillJobId, sFillSnapshotId;

// Defined in the main-loop section below; declared here because the DMA
// completion path (IRQ context) decides backfill admission.
static CACHE_RAM_CODE void fillAbortReason(uint8_t reason);
static CACHE_RAM_CODE void fillAdmitLocked(void);

// DMA0-owned response slot tracking (written by E5/DMA-done and the main-loop
// reclaim, always inside a critical section).
static uint8_t sDmaValid, sDmaSlot;
static uint32_t sDmaVersion;

static volatile uint32_t sHits, sMisses, sFills, sErrors;

// Categorized counters (design section 9.1). Monotonic; read without masking
// interrupts because they are diagnostic only.
static volatile uint32_t sSdTokenObsolete, sSdTokenLiveConflict, sSdTransferError;
static volatile uint32_t sL2HitAttempt, sL2HitVerified, sL2CrcFail;
static volatile uint32_t sPsramReadFail, sPsramFillFail;
static volatile uint32_t sFillAdmit, sFillDrop, sFillDefer, sFillCommit, sFillQuarantine;
static volatile uint32_t sDemandHitOk, sDemandNonPsramOk, sProbeTry, sProbeOk, sReadBytes, sWriteBytes;
#ifdef CACHE_PROBE_MISMATCH_DIAG
static volatile uint32_t sProbeFirstValid, sProbeFirstSector, sProbeFirstOffset;
static volatile uint32_t sProbeFirstExpected, sProbeFirstActual;
static volatile uint32_t sProbeFirstRefCrc, sProbeFirstTagCrc;
#endif
#ifdef CACHE_FILL_QUIET_EXPERIMENT
#define CACHE_FILL_QUIET_US 1000u
static volatile uint32_t sLastHostCommandUs;
static volatile uint32_t sHostCommandSeq;
static uint32_t sLastQuietFragmentUs;
static volatile uint32_t sQuietFillFragments, sQuietFillMaxUs;
static volatile uint32_t sQuietFragmentActive, sQuietIrqOverlap, sQuietE5Overlap;
static inline CACHE_RAM_CODE uint32_t hostCommandNow32(void)
{
#ifdef CACHE_SD_HOST
    return (uint32_t)sHw->nowUs();
#else
    // One timer register read in the command IRQ. The 64-bit SDK helper is
    // unnecessary for a one-millisecond guard and costs more IRQ cycles.
    return time_us_32();
#endif
}
#endif
#if defined(CACHE_WATCH_ACTIVE_MARKERS) || defined(CACHE_SD_HOST)
static volatile uint32_t sGateChecks, sGateBlocked, sGateMask;
static uint32_t sGatePhase;
#endif
volatile uint32_t gCacheSdReadReady;
// Published only by E4 after full identity/slot validation. All revocations
// run on core0, serialized by the cartridge IRQ priority / main-loop mask.
static uint8_t* sAckedData;
static volatile cacheSdTxSnapshot sLastTx;


// Time budgets (design 7.4). Units are microseconds and the values are
// placeholders that MUST be replaced by measured bounds before the final
// acceptance; they are runtime-settable so the host tests can drive them.
// Loop counts and nominal clock rates are explicitly NOT accepted as worst-case
// time bounds. A fragment whose bound is not qualified (D4) stays out.
#ifndef CACHE_T_STEP_MAX_US
#define CACHE_T_STEP_MAX_US 250u
#endif
#ifndef CACHE_T_FRAGMENT_MAX_US
#define CACHE_T_FRAGMENT_MAX_US 80u
#endif
#ifndef CACHE_T_CLEANUP_MAX_US
#define CACHE_T_CLEANUP_MAX_US 100u
#endif
#ifndef CACHE_T_IDLE_ADMIT_US
#define CACHE_T_IDLE_ADMIT_US 0u
#endif
static uint32_t sTStepMaxUs = CACHE_T_STEP_MAX_US;
static uint32_t sTFragmentMaxUs = CACHE_T_FRAGMENT_MAX_US;
static uint32_t sTCleanupMaxUs = CACHE_T_CLEANUP_MAX_US;
static uint32_t sTIdleAdmitUs = CACHE_T_IDLE_ADMIT_US;

// M3 contrast build (design section 10): reproduce the old "publish then fill
// in the same round" behaviour for one-variable comparison. MUST stay off and
// MUST NOT become the default after acceptance.
#if defined(CACHE_L2_LEGACY_PUBLISH_FILL)
#define FILL_LEGACY_PUBLISH 1
#else
#define FILL_LEGACY_PUBLISH 0
#endif

// ---------------------------------------------------------------------------

static inline CACHE_RAM_CODE uint32_t cacheIdx(uint32_t sector) { return sector & (CACHE_SD_SECTORS - 1u); }

#ifndef CACHE_FULL_PAGE_4CHIP
static inline CACHE_RAM_CODE uint32_t cacheAddr(uint32_t idx, uint32_t off)
{
    uint32_t base = (idx >> 2) * CS_SECTOR_BYTES + off;
#ifdef CACHE_SHADOW_STRIPES_4CHIP
    base += (uint32_t)sStripe[idx] * 0x80000u;
#endif
    return base;
}

static inline CACHE_RAM_CODE uint32_t fillAddr(uint32_t off)
{
    uint32_t base = (sFillIdx >> 2) * CS_SECTOR_BYTES + off;
#ifdef CACHE_SHADOW_STRIPES_4CHIP
    base += (uint32_t)sFillStripe * 0x80000u;
#endif
    return base;
}
#endif

// Buffer currently holding the demand payload (fallback after a PSRAM fault).
static inline CACHE_RAM_CODE uint8_t* jobBuf(void)
{
    return sJobUseFallback ? sFallbackStage : sDemandStage;
}

static CACHE_RAM_CODE uint32_t crc32Update(uint32_t crc, const uint8_t* p, uint32_t n)
{
    while (n--)
        crc = sCrcTab[(crc ^ *p++) & 0xFFu] ^ (crc >> 8);
    return crc;
}

// Derived fast ready (design 6.2). The value is computed from the response
// descriptor, never an independent truth.
CACHE_RAM_CODE bool cacheSdFastReady(void)
{
    if (!sCs.intent_valid || sCs.write_pending)
        return false;
    if (sCs.completion_valid &&
        cs_identity_equal(&sCs.completion.id, &sCs.intent) &&
        sCs.completion.id.write_epoch == sCs.write_epoch &&
        sCs.completion.result == CS_RES_OK)
        return true;
    if (sCs.binding_valid &&
        cs_identity_equal(&sCs.binding.id, &sCs.intent) &&
        sCs.binding.id.write_epoch == sCs.write_epoch)
        return true;
    return false;
}

static CACHE_RAM_CODE void refreshFastReady(void)
{
    gCacheSdReadReady = cacheSdFastReady() ? 1u : 0u;
    if (!gCacheSdReadReady)
        sAckedData = NULL;
}

CACHE_RAM_CODE uint8_t* cacheSdSlotBuffer(uint8_t slot);

CACHE_RAM_CODE void cacheSdSetHw(const cacheSdHw* hw)
{
    sHw = hw;
}

CACHE_RAM_CODE void cacheSdSetTimeBudgets(uint32_t step_us, uint32_t fragment_us,
                                          uint32_t cleanup_us, uint32_t idle_admit_us)
{
    sTStepMaxUs = step_us;
    sTFragmentMaxUs = fragment_us;
    sTCleanupMaxUs = cleanup_us;
    sTIdleAdmitUs = idle_admit_us;
}

CACHE_RAM_CODE void cacheSdInit(void)
{
#ifdef CACHE_SHADOW_STRIPES_4CHIP
    memset(sStripe, 0, sizeof(sStripe));
    memset(sNextStripe, 0, sizeof(sNextStripe));
    memset((void*)sShadowBanks, 0, sizeof(sShadowBanks));
    memset((void*)sShadowVerify, 0, sizeof(sShadowVerify));
    sFillStripe = 0;
#endif
#ifdef CACHE_M4_PROMOTE_M5_AFTER_PROBES
    sM4Promoted = 0;
#endif
    for (uint32_t i = 0; i < 256; i++)
    {
        uint32_t c = i;
        for (int k = 0; k < 8; k++)
            c = (c & 1u) ? (0xEDB88320u ^ (c >> 1)) : (c >> 1);
        sCrcTab[i] = c;
    }
#ifndef CACHE_SD_HOST
    if (!sHw)
        sHw = cacheSdDefaultHw();
#endif
#ifdef CACHE_FULL_PAGE_4CHIP
    cachePageStoreInit(&sPageStore, 0x0fu);
    memset(&sDemandPageTask, 0, sizeof(sDemandPageTask));
    memset(&sFillPageTask, 0, sizeof(sFillPageTask));
    memset((void*)sPageFill, 0, sizeof(sPageFill));
    memset((void*)sPageProbe, 0, sizeof(sPageProbe));
    memset((void*)sPageHitOk, 0, sizeof(sPageHitOk));
    memset((void*)sPageMaxSet, 0, sizeof(sPageMaxSet));
#else
    memset(sState, 0, sizeof(sState));
    memset(sTagEpoch, 0, sizeof(sTagEpoch));
#endif
    sCacheEpoch = 1;
    cs_init(&sCs);
    sJobActive = 0;
#ifdef CACHE_BOOT_SECTOR0_SEED
    sBootSector0Valid = 0;
    sBootSeedUse = 0;
#endif
    sJobUseFallback = 0;
    sFillState = FILL_NONE;
    sFillIsProbe = 0;
    sFillDropReason = FILL_DROP_NONE;
    sFillSector = sFillOff = sFillSnapshotId = 0;
    sDidPublish = 0;
    sDmaValid = 0;
    sDegraded = 0;
    gCacheSdReadReady = 0;
    sAckedData = NULL;
    memset((void*)&sLastTx, 0, sizeof(sLastTx));
    sHits = sMisses = sFills = sErrors = 0;
    sSdTokenObsolete = sSdTokenLiveConflict = sSdTransferError = 0;
    sL2HitAttempt = sL2HitVerified = sL2CrcFail = 0;
    sPsramReadFail = sPsramFillFail = 0;
    sFillAdmit = sFillDrop = sFillDefer = sFillCommit = sFillQuarantine = 0;
    sDemandHitOk = sDemandNonPsramOk = sProbeTry = sProbeOk = sReadBytes = sWriteBytes = 0;
#ifdef CACHE_PROBE_MISMATCH_DIAG
    sProbeFirstValid = sProbeFirstSector = sProbeFirstOffset = 0;
    sProbeFirstExpected = sProbeFirstActual = 0;
    sProbeFirstRefCrc = sProbeFirstTagCrc = 0;
#endif
#ifdef CACHE_FILL_QUIET_EXPERIMENT
    sLastHostCommandUs = hostCommandNow32();
    sHostCommandSeq = 0;
    sLastQuietFragmentUs = sLastHostCommandUs;
    sQuietFillFragments = sQuietFillMaxUs = 0;
    sQuietFragmentActive = sQuietIrqOverlap = sQuietE5Overlap = 0;
#endif
#if defined(CACHE_WATCH_ACTIVE_MARKERS) || defined(CACHE_SD_HOST)
    sGateChecks = sGateBlocked = sGateMask = 0;
    sGatePhase = 0;
#endif
    sEnabled = false;
}

CACHE_RAM_CODE void cacheSdSeedBootSector0(const uint8_t* data)
{
#ifdef CACHE_BOOT_SECTOR0_SEED
    if (!data)
        return;
    // Called before the cartridge can issue E3. Do not publish a response:
    // the normal intent/job/slot identity checks remain authoritative.
    memcpy(sBootSector0, data, CS_SECTOR_BYTES);
    sBootSector0Valid = 1;
#else
    (void)data;
#endif
}

CACHE_RAM_CODE void cacheSdSetEnabled(bool enabled)
{
    sEnabled = enabled;
    refreshFastReady();
}

CACHE_RAM_CODE void cacheSdNoteHostCommandFromIrq(uint8_t kind)
{
#ifdef CACHE_FILL_QUIET_EXPERIMENT
    // A single 32-bit store in the command IRQ. The timer wraps safely under
    // unsigned subtraction; this is a guard, not a future idle guarantee.
    sLastHostCommandUs = hostCommandNow32();
    sHostCommandSeq++;
    if (sQuietFragmentActive)
    {
        sQuietIrqOverlap++;
        if (kind == CACHE_HOST_CMD_E5) sQuietE5Overlap++;
    }
#else
    (void)kind;
#endif
}

CACHE_RAM_CODE void cacheSdSetMode(uint8_t mode)
{
#ifdef CACHE_SHADOW_STRIPES_4CHIP
    // A shadow build can never be switched to PSRAM demand service.
    if (mode > CACHE_L2_MODE_M4) mode = CACHE_L2_MODE_M4;
#endif
#if defined(CACHE_FULL_PAGE_4CHIP) && !defined(CACHE_PAGE_M5_EXPERIMENT)
    if (mode > CACHE_L2_MODE_M4) mode = CACHE_L2_MODE_M4;
#endif
    sL2Mode = mode;
#ifdef CACHE_M4_PROMOTE_M5_AFTER_PROBES
    sM4Promoted = 0;
#endif
}
CACHE_RAM_CODE uint8_t cacheSdMode(void) { return sL2Mode; }

CACHE_RAM_CODE void cacheSdDegrade(uint32_t reason)
{
    uint32_t save = sHw->irqSave();
    sDegraded = 1;
    cs_set_degraded(&sCs, true);
    // Close new fill admission but keep already-published SRAM responses.
    if (sFillState == FILL_SNAPSHOT_READY || sFillState == FILL_ADMITTED ||
        sFillState == FILL_TRANSFERRING)
        fillAbortReason(FILL_DROP_DEGRADE);
    cs_note_fault_ex(&sCs, CS_FAULT_DEGRADE, reason, CS_PRODUCER_MAIN, reason, 0);
    refreshFastReady();
    sHw->irqRestore(save);
}

CACHE_RAM_CODE bool cacheSdDegraded(void) { return sDegraded != 0; }

CACHE_RAM_CODE const char* cacheSdFillStateName(void)
{
    if (sFillState >= sizeof(sFillStateNames) / sizeof(sFillStateNames[0]))
        return "?";
    return sFillStateNames[sFillState];
}

CACHE_RAM_CODE uint32_t cacheSdFillDropReason(void) { return sFillDropReason; }

// ---------------------------------------------------------------------------
// IRQ entry points (bounded, no I/O)
// ---------------------------------------------------------------------------

// Release a finished DMA0 slot. The tracking identity, the DMA state check and
// the matching release all happen in one critical section so a new E5 cannot
// slip between the check and the release (design 13.2/R1).
static CACHE_RAM_CODE void dmaReclaimLocked(void)
{
    if (!sDmaValid)
        return;
    // Snapshot the transmission identity before the (possibly reentrant) DMA
    // state query, so a send bound during the query can never be released by
    // mistake (design 13.2/R1, T20).
    uint8_t slot = sDmaSlot;
    uint32_t ver = sDmaVersion;
    if (sHw->dma0Busy())
        return;
    // The demand transfer for this slot is complete: the response's handover
    // window is closed and its backfill may now be admitted (design 7.3).
    if (sFillState == FILL_SNAPSHOT_READY && slot == sFillWaitSlot && ver == sFillWaitVersion)
        fillAdmitLocked();
    cs_dma_done(&sCs, slot, ver);
    if (sDmaValid && sDmaSlot == slot && sDmaVersion == ver)
        sDmaValid = 0;
    sCs.c_dma_reclaim++;
}

CACHE_RAM_CODE void cacheSdDmaDone(void)
{
    uint32_t save = sHw->irqSave();
    if (sDmaValid)
    {
        if (sFillState == FILL_SNAPSHOT_READY && sDmaSlot == sFillWaitSlot &&
            sDmaVersion == sFillWaitVersion)
            fillAdmitLocked();
        cs_dma_done(&sCs, sDmaSlot, sDmaVersion);
        sDmaValid = 0;
    }
    sHw->irqRestore(save);
}

CACHE_RAM_CODE uint32_t cacheSdRequest(uint32_t sector)
{
    uint32_t result = cs_read_intent(&sCs, sector);
    refreshFastReady();
    return result;
}

CACHE_RAM_CODE bool cacheSdPollReadySampled(bool queued_ready)
{
    if (!queued_ready)
    {
        sCs.c_e4_busy++;
        return false; // a busy word cannot acknowledge an offer
    }
    bool ready = cs_poll_read(&sCs);
    if (queued_ready && !ready)
    {
        // A ready=1 was already in the FIFO but no binding could be made. This
        // is the H2/H3 signature; freeze it and never rewrite the frame.
        sCs.c_e4_ack_failed++;
        cs_note_e4_ack_failed(&sCs, 1);
    }
    else if (ready)
    {
        sCs.c_e4_ready_queued++;
    }
    else
    {
        sCs.c_e4_busy++;
    }
    refreshFastReady();
    // The old DMA may be finished but not reclaimed yet. Do that after the
    // E4 status store, not on the next E5's first-data deadline.
    dmaReclaimLocked();
    sAckedData = ready && cs_e5_precheck(&sCs) && !sDmaValid
                     ? sSlotBuf[sCs.binding.slot] : NULL;
    return ready;
}

CACHE_RAM_CODE bool cacheSdPollReady(void)
{
    return cacheSdPollReadySampled(cacheSdFastReady());
}

CACHE_RAM_CODE bool cacheSdE5Precheck(void)
{
    return cs_e5_precheck(&sCs);
}

CACHE_RAM_CODE void cacheSdMarkE5Recovery(void)
{
    // The precise reject reason is frozen by cs_consume_read(); this only counts
    // that the E5 entered the bounded recovery (no length armed, no data fed).
    sCs.c_e5_recovery++;
}

// Hot E5 path. The lease implies the SAME immutable binding fully checked
// at E4. E3 replacement, write/reset/media and normal consume revoke it via
// refreshFastReady(). Main cannot replace an accepted response. The caller
// must be the highest-priority cartridge IRQ; no callback or mask is needed.
CACHE_RAM_CODE uint8_t* cacheSdTakeAckedFromIrq(uint32_t* sector)
{
    uint8_t* data = sAckedData;
    if (!data)
        return NULL;
#ifdef CACHE_SD_HOST
    if (sHw->dma0Busy())
#else
    if (dma_channel_is_busy(0))
#endif
        return NULL; // never overwrite a live DMA, including a non-SD owner
    sAckedData = NULL;
    uint8_t slot = sCs.binding.slot;
    sCs.slots[slot].state = CS_SLOT_DMA;
    sDmaSlot = slot;
    sDmaVersion = sCs.binding.slot_version;
    sDmaValid = 1;
    *sector = sCs.binding.id.sector;
    sCs.binding_valid = 0;
    sCs.acked_offer_id = 0;
    sCs.intent_valid = 0;
    sCs.intent_consumed = 0;
    sCs.resp_state = CS_RESP_SENDING;
    sCs.last_invalidate_reason = CS_INV_CONSUMED;
    gCacheSdReadReady = 0;
    return data;
}

CACHE_RAM_CODE void cacheSdRejectSendFromIrq(void)
{
    uint32_t sector, version;
    uint8_t slot;
    if (!cs_e5_precheck(&sCs))
        (void)cs_consume_read(&sCs, &sector, &slot, &version); // classify only
    else
    {
        // Valid logical response but no usable lease / DMA still busy.
        sCs.c_protocol_fault++;
        cs_note_fault_ex(&sCs, CS_FAULT_E5_RECOVERY, 1,
                         CS_PRODUCER_IRQ_E5, sDmaValid, 0);
    }
}

CACHE_RAM_CODE void cacheSdRecordSendFromIrq(uint32_t sector, const uint8_t* data)
{
    // Called after DMA starts, still in the same IRQ before any next demand.
    sCs.c_e5_ok++;
    sLastTx.sector = sector;
    memcpy((void*)&sLastTx.head, data, 4);
    memcpy((void*)&sLastTx.tail, data + CS_SECTOR_BYTES - 4, 4);
    sLastTx.offer = sCs.binding.offer_id;
    sLastTx.sends++;
}

CACHE_RAM_CODE const volatile cacheSdTxSnapshot* cacheSdLastTx(void) { return &sLastTx; }

CACHE_RAM_CODE bool cacheSdConsume(uint32_t* sector, uint8_t* slot, uint32_t* slot_version)
{
    // Reclaim a finished previous transmission and bind the new one under the
    // same critical section, so the tracking fields can never refer to a
    // mixture of the old and the new send.
    uint32_t save = sHw->irqSave();
    dmaReclaimLocked();
    bool ok = cs_consume_read(&sCs, sector, slot, slot_version);
    if (ok)
    {
        sDmaSlot = *slot;
        sDmaVersion = *slot_version;
        sDmaValid = 1;
    }
    refreshFastReady();
    sHw->irqRestore(save);
    return ok;
}

CACHE_RAM_CODE uint32_t cacheSdWriteBegin(void)
{
    uint32_t save = sHw->irqSave();
#ifdef CACHE_BOOT_SECTOR0_SEED
    sBootSector0Valid = 0;
#endif
    uint32_t before = sCs.write_epoch;
    uint32_t token = cs_write_begin(&sCs);
    if (sCs.write_epoch != before)
    {
        uint32_t e = sCacheEpoch + 1u;
        if (e == 0)
            e = 1;
        sCacheEpoch = e;
#ifdef CACHE_SHADOW_STRIPES_4CHIP
        // Coverage belongs to one authoritative SD write generation.
        for (uint32_t chip = 0; chip < 4u; chip++)
            sShadowBanks[chip] = sShadowVerify[chip] = 0;
#endif
    }
    refreshFastReady();
    sHw->irqRestore(save);
    return token;
}

CACHE_RAM_CODE void cacheSdWriteEnd(uint32_t token)
{
    uint32_t save = sHw->irqSave();
    cs_write_end(&sCs, token);
    refreshFastReady();
    sHw->irqRestore(save);
}

CACHE_RAM_CODE void cacheSdResetCart(void)
{
    uint32_t save = sHw->irqSave();
    cs_reset_cart(&sCs);
    refreshFastReady();
    sHw->irqRestore(save);
}

CACHE_RAM_CODE void cacheSdSetMediaEpoch(uint32_t media_epoch)
{
    uint32_t save = sHw->irqSave();
#ifdef CACHE_BOOT_SECTOR0_SEED
    sBootSector0Valid = 0;
#endif
    cs_set_media_epoch(&sCs, media_epoch);
    // Medium identity changed: any cached sector may belong to the previous
    // medium, so advance the namespace and let entries be re-matched by a new
    // request (design section 9.1).
    uint32_t e = sCacheEpoch + 1u;
    if (e == 0)
        e = 1;
    sCacheEpoch = e;
#ifdef CACHE_SHADOW_STRIPES_4CHIP
    for (uint32_t chip = 0; chip < 4u; chip++)
        sShadowBanks[chip] = sShadowVerify[chip] = 0;
#endif
    refreshFastReady();
    sHw->irqRestore(save);
}

// ---------------------------------------------------------------------------
// Main-loop service
// ---------------------------------------------------------------------------

// Record a main-loop fault under the interrupt mask. The first-fault slot has
// two producers (cartridge IRQ and core0 main loop); masking makes each record
// atomic instead of treating them as one lock-free SPSC ring (design 9.2).
static CACHE_RAM_CODE void noteFaultLocked(uint8_t kind, uint32_t detail)
{
    uint32_t save = sHw->irqSave();
    cs_note_fault(&sCs, kind, detail);
    sHw->irqRestore(save);
}

// Retire a job that produced no publishable result (cancel/fault). A still-live
// intent is re-armed for a bounded retry; anything else is retired (13.5/R4).
static CACHE_RAM_CODE void jobFinish(uint8_t result)
{
    (void)result;
#ifdef CACHE_FULL_PAGE_4CHIP
    cachePageStoreCancel(&sPageStore, &sDemandPageTask);
#endif
    uint32_t save = sHw->irqSave();
    cs_job_requeue(&sCs, &sJob);
    refreshFastReady();
    sHw->irqRestore(save);

    sJobActive = 0;
    sJobSdStarted = 0;
}

static CACHE_RAM_CODE void fillAbortReason(uint8_t reason)
{
    if (sFillState == FILL_NONE)
        return;
#ifdef CACHE_FULL_PAGE_4CHIP
    cachePageStoreCancel(&sPageStore, &sFillPageTask);
#else
    if (sState[sFillIdx] == 1)
        sState[sFillIdx] = 0;
#endif
    sFillDropReason = reason;
    sFillIsProbe = 0;
    sFillState = FILL_DROP;
}

// Admit (or discard) a snapshot whose source response has finished its send.
// Only used from the DMA-completion path, inside the caller's critical section.
static CACHE_RAM_CODE void fillAdmitLocked(void)
{
    if (sFillState != FILL_SNAPSHOT_READY)
        return;
    if (!sDegraded && sL2Mode >= CACHE_L2_MODE_M3)
    {
        // Admission is unconditional on the DMA end; the idle gate is applied
        // only to the ADMITTED -> TRANSFERRING start, so a slot released here
        // can never make fillReconcile() drop an admitted snapshot.
        sFillState = FILL_ADMITTED;
        sFillAdmit++;
    }
    else
    {
        // M2, degraded or SNAPSHOT_ONLY: pay the copy cost but never touch
        // PSRAM data.
        fillAbortReason(sL2Mode >= CACHE_L2_MODE_M3 ? FILL_DROP_DEGRADE : FILL_DROP_MODE);
        sFillDrop++;
    }
}

// Publish a verified demand read: copy the staging buffer into a response slot,
// commit it, then take an independent backfill snapshot. The snapshot's
// transport is NOT started here; admission waits for the published response's
// E5/DMA0 transfer to end (design sections 7.2/7.3, H1 isolation).
static CACHE_RAM_CODE bool publishVerified(void)
{
    memcpy(sSlotBuf[sJob.slot], jobBuf(), CS_SECTOR_BYTES);

    uint32_t save = sHw->irqSave();
    cs_completion c;
    c.id = sJob.id;
    c.job_id = sJob.job_id;
    c.slot = sJob.slot;
    c.slot_version = sJob.slot_version;
    c.result = CS_RES_OK;
    bool committed = cs_commit(&sCs, &c);
    if (!committed)
        cs_job_requeue(&sCs, &sJob);
    refreshFastReady();
    sHw->irqRestore(save);

    if (committed && sJobSrc == 0)
        sDemandNonPsramOk++;

    if (committed && sEnabled && !sDegraded && sJobSrc == 0 &&
        sL2Mode >= CACHE_L2_MODE_M2
#if defined(CACHE_M4_PROMOTE_M5_AFTER_PROBES) && !defined(CACHE_M5_QUIET_FILL_AFTER_PROMOTION)
        // The promotion trial serves only M4-warmed entries. A fresh M5 miss
        // must not pin the sole snapshot buffer behind its stricter idle gate
        // or invalidate a ready entry merely to queue an unfinishable refill.
        && !sM4Promoted
#endif
        )
    {
#ifdef CACHE_FULL_PAGE_4CHIP
        if (sFillState == FILL_NONE)
        {
            memcpy(sFillStage, jobBuf(), CS_SECTOR_BYTES);
            sFillSector = sJob.id.sector;
            sFillIdx = cacheIdx(sFillSector);
            sFillOff = 0;
            sFillCrc = 0xFFFFFFFFu;
            sFillEpoch = sJob.source_epoch;
            sFillMediaEpoch = sJob.id.media_epoch;
            sFillWriteEpoch = sJob.id.write_epoch;
            sFillReqId = sJob.id.request_id;
            sFillJobId = sJob.job_id;
            sFillSnapshotId++;
            bool probe = sL2Mode == CACHE_L2_MODE_M4 &&
                         cachePageStoreBeginProbe(&sPageStore, &sFillPageTask,
                             sFillSector, sFillEpoch, sFillStage);
            bool started = probe || cachePageStoreBeginFill(&sPageStore,
                &sFillPageTask, sFillSector, sFillEpoch, sFillStage);
            if (started)
            {
                sFillIsProbe = probe ? 1u : 0u;
                sFillWaitSlot = sJob.slot;
                sFillWaitVersion = sJob.slot_version;
                sFillState = FILL_SNAPSHOT_READY;
                if (probe) { sL2HitAttempt++; sProbeTry++; }
            }
            else
            {
                sFillDropReason = FILL_DROP_NO_SLOT;
                sFillDefer++;
            }
        }
        else
        {
            sFillDropReason = FILL_DROP_EXISTING;
            sFillDefer++;
        }
#else
        uint32_t pidx = cacheIdx(sJob.id.sector);
        bool haveEntry = (sState[pidx] == 2 && sTagEpoch[pidx] == sJob.source_epoch &&
                          sTag[pidx] == sJob.id.sector);
        // M4 verifies an existing entry against the SD reference (design 10);
        // M3/M5 schedule a normal write-backfill.
        bool wantProbe = (sL2Mode == CACHE_L2_MODE_M4) && haveEntry;
        bool want = true; // cold M4 must first populate entries to probe
        if (want && sFillState == FILL_NONE)
        {
            // The demand staging buffer is still held by this job, so the
            // snapshot copy cannot alias an SD/DMA destination (design 7.2).
            memcpy(sFillStage, jobBuf(), CS_SECTOR_BYTES);
            sFillSector = sJob.id.sector;
            sFillIdx = cacheIdx(sJob.id.sector);
#ifdef CACHE_SHADOW_STRIPES_4CHIP
            if (wantProbe)
                sFillStripe = sStripe[sFillIdx];
            else
            {
                uint32_t chip = sFillIdx & 3u;
                sFillStripe = sNextStripe[chip];
                sNextStripe[chip] = (sNextStripe[chip] + 1u) & 15u;
            }
#endif
            sFillOff = 0;
            sFillCrc = 0xFFFFFFFFu;
            sFillEpoch = sJob.source_epoch;
            // Immutable source identity (design 7.1), captured at snapshot time.
            sFillMediaEpoch = sJob.id.media_epoch;
            sFillWriteEpoch = sJob.id.write_epoch;
            sFillReqId = sJob.id.request_id;
            sFillJobId = sJob.job_id;
            sFillSnapshotId++;
            // Only a write-backfill marks the entry as filling; an M4 probe must
            // leave the already-valid entry untouched.
            if (!wantProbe)
                sState[sFillIdx] = 1;
            sFillWaitSlot = sJob.slot;
            sFillWaitVersion = sJob.slot_version;
            sFillState = FILL_SNAPSHOT_READY;
            sFillIsProbe = wantProbe ? 1 : 0;
            if (wantProbe)
            {
                sL2HitAttempt++;
                sProbeTry++;
            }
        }
        else if (want)
        {
            sFillDropReason = FILL_DROP_EXISTING;
            sFillDefer++;
        }
#endif
    }

    sDidPublish = 1;
    sJobActive = 0;
    sJobSdStarted = 0;
    sJobUseFallback = 0;
    return committed;
}

static CACHE_RAM_CODE void jobStart(const cs_job* job)
{
    sJob = *job;
    sJobActive = 1;
    sJobSdStarted = 0;
    sJobUseFallback = 0;
    sJobOff = 0;
    sJobL2Epoch = job->source_epoch;

    uint32_t sector = job->id.sector;
#ifndef CACHE_FULL_PAGE_4CHIP
    uint32_t idx = cacheIdx(sector);
#endif
    // M1/M2/M3 keep the demand on SD so the mode matrix is a real control
    // variable. M4 VERIFIES entries against SD instead of serving them; only M5
    // serves a verified hit. A degraded backend never serves a hit (section 11).
#ifdef CACHE_FULL_PAGE_4CHIP
    if (sEnabled && !sDegraded && sL2Mode == CACHE_L2_MODE_M5 &&
        cachePageStoreBeginRead(&sPageStore, &sDemandPageTask, sector,
                                sJobL2Epoch, sDemandStage))
    {
        sJobSrc = 1;
        sHits++;
        sL2HitAttempt++;
        return;
    }
#else
    if (sEnabled && !sDegraded && sL2Mode == CACHE_L2_MODE_M5 && sState[idx] == 2 &&
        sTagEpoch[idx] == sJobL2Epoch && sTag[idx] == sector)
    {
        sJobSrc = 1;
        sJobL2Idx = idx;
        sJobCrc = 0xFFFFFFFFu;
        sHits++;
        sL2HitAttempt++;
        return;
    }
#endif

    sJobSrc = 0;
    sMisses++;
#ifdef CACHE_BOOT_SECTOR0_SEED
    if (sector == 0 && sBootSector0Valid)
    {
        // Consume once and go through the ordinary cs_commit/E4/E5 path.
        // No SD transfer is started in the host's first poll window.
        memcpy(sDemandStage, sBootSector0, CS_SECTOR_BYTES);
        sBootSector0Valid = 0;
        sJobSrc = 2;
    }
#endif
}

static CACHE_RAM_CODE void jobStep(void)
{
#ifdef CACHE_BOOT_SECTOR0_SEED
    if (sJobSrc == 2)
    {
        sJobSrc = 0; // SD-origin data may follow the normal snapshot policy.
        sBootSeedUse++;
        publishVerified();
        return;
    }
#endif
    if (sJobSrc == 0)
    {
        if (!sJobSdStarted)
        {
            if (!sHw->sdBeginRead(jobBuf(), sJob.id.sector))
                return; // SD engine busy; retry next step
            sJobSdToken = sHw->sdTransferId();
            sJobSdStarted = 1;
            return;
        }

        // Superseded by another SD owner: our result is no longer identifiable.
        // Distinguish a benign obsolete retirement from a live-owner conflict
        // (design sections 2.2 and 9.1): the latter must be fixed, not excused.
        if (sHw->sdTransferId() != sJobSdToken)
        {
            bool live = sCs.intent_valid && cs_identity_equal(&sJob.id, &sCs.intent);
            if (live)
            {
                sSdTokenLiveConflict++;
                noteFaultLocked(CS_FAULT_SD_TOKEN, 1);
            }
            else
            {
                sSdTokenObsolete++;
            }
            sErrors++;
            jobFinish(CS_RES_CANCELLED);
            return;
        }
        if (sHw->sdReady())
        {
            // SD completion must carry an error status, not just "Idle"
            // (design section 11). A failed transfer is never published.
            if (sHw->sdError())
            {
                sSdTransferError++;
                sErrors++;
                jobFinish(CS_RES_FAULT);
                return;
            }
            publishVerified();
        }
        return;
    }

    // L2 hit read: chunked PSRAM -> staging, CRC checked against the directory.
#ifdef CACHE_FULL_PAGE_4CHIP
    if (sJobL2Epoch != sCacheEpoch ||
        sJob.id.media_epoch != sCs.media_epoch ||
        sJob.id.write_epoch != sCs.write_epoch)
    {
        cachePageStoreCancel(&sPageStore, &sDemandPageTask);
        sJobSrc = 0;
        sJobSdStarted = 0;
        sJobUseFallback = 1;
        return;
    }
    cachePageIo io = { sHw->psramRead, sHw->psramWrite };
    uint32_t before = sDemandPageTask.read_bytes;
    cachePageResult pageResult = cachePageStoreStep(&sPageStore,
                                                    &sDemandPageTask, &io);
    sReadBytes += sDemandPageTask.read_bytes - before;
    if (pageResult == CACHE_PAGE_BUSY) return;
    if (pageResult == CACHE_PAGE_DONE && sJobL2Epoch == sCacheEpoch &&
        sJob.id.media_epoch == sCs.media_epoch &&
        sJob.id.write_epoch == sCs.write_epoch)
    {
        sL2HitVerified++;
        if (publishVerified())
        {
            sDemandHitOk++;
            sPageHitOk[cachePageMapChip(sDemandPageTask.lease.slot)]++;
        }
        return;
    }
    if (pageResult == CACHE_PAGE_IO_FAIL)
    {
        sErrors++;
        sPsramReadFail++;
        sFillQuarantine++;
        noteFaultLocked(CS_FAULT_PSRAM, sJob.id.sector);
        cacheSdDegrade(CS_FAULT_PSRAM);
    }
    else if (pageResult == CACHE_PAGE_CORRUPT)
    {
        sErrors++;
        sL2CrcFail++;
        noteFaultLocked(CS_FAULT_L2_CRC, sJob.id.sector);
    }
    sJobSrc = 0;
    sJobSdStarted = 0;
    sJobUseFallback = 1;
    return;
#else
    uint32_t frag = CS_SECTOR_BYTES - sJobOff;
    if (frag > PSRAM_CACHE_FRAG_BYTES)
        frag = PSRAM_CACHE_FRAG_BYTES;
    uint32_t chip = sJobL2Idx & 3u;
    uint32_t base = cacheAddr(sJobL2Idx, sJobOff);
    uint8_t* dst = &sDemandStage[sJobOff];

    if (!sHw->psramRead(chip, base, dst, frag))
    {
        // Bounded transport failure: degrade the L2 path, drop the entry and
        // continue this demand over SD from a buffer the failed transport was
        // never allowed to touch (design section 11).
        sErrors++;
        sPsramReadFail++;
        sFillQuarantine++;
        noteFaultLocked(CS_FAULT_PSRAM, sJob.id.sector);
        cacheSdDegrade(CS_FAULT_PSRAM);
        if (sState[sJobL2Idx] != 1)
            sState[sJobL2Idx] = 0;
        sJobSrc = 0;
        sJobSdStarted = 0;
        sJobUseFallback = 1;
        return;
    }

    sReadBytes += frag;
    sJobCrc = crc32Update(sJobCrc, dst, frag);
    sJobOff += frag;

    if (sJobOff < CS_SECTOR_BYTES)
        return;

    sJobCrc ^= 0xFFFFFFFFu;
    bool ok = (sState[sJobL2Idx] == 2) &&
              sTagEpoch[sJobL2Idx] == sJobL2Epoch &&
              sTag[sJobL2Idx] == sJob.id.sector &&
              sJobCrc == sCrc[sJobL2Idx];
    if (ok)
    {
        sL2HitVerified++;
        if (publishVerified())
            sDemandHitOk++;
        return;
    }

    // Corrupt/stale entry: drop it and retry the same request over SD once.
    sErrors++;
    sL2CrcFail++;
    noteFaultLocked(CS_FAULT_L2_CRC, sJob.id.sector);
    if (sState[sJobL2Idx] != 1)
        sState[sJobL2Idx] = 0;
    sJobSrc = 0;
    sJobSdStarted = 0;
    sJobUseFallback = 1;
#endif
}

// M4 READ_PROBE (design section 10): read the existing L2 entry in bounded
// fragments into an independent probe buffer and compare the FULL 512 B against
// the SD reference snapshot. The demand staging and the E5 response slot are
// never touched. A media/write epoch change cancels the comparison instead of
// reporting a legitimate update as a CRC error.
static CACHE_RAM_CODE void probeStep(void)
{
#ifdef CACHE_FULL_PAGE_4CHIP
    cachePageIo io = { sHw->psramRead, sHw->psramWrite };
    uint32_t before = sFillPageTask.read_bytes;
    cachePageResult result = cachePageStoreStep(&sPageStore, &sFillPageTask, &io);
    sReadBytes += sFillPageTask.read_bytes - before;
    sFillOff = sFillPageTask.offset;
    if (result == CACHE_PAGE_BUSY) return;
    bool epochOk = sFillEpoch == sCacheEpoch &&
                   sFillMediaEpoch == sCs.media_epoch &&
                   sFillWriteEpoch == sCs.write_epoch;
    if (!epochOk || result == CACHE_PAGE_CANCELLED)
    {
        sFillDropReason = FILL_DROP_EPOCH;
        sFillDrop++;
    }
    else if (result == CACHE_PAGE_DONE)
    {
        sL2HitVerified++;
        sProbeOk++;
        sPageProbe[cachePageMapChip(sFillPageTask.lease.slot)]++;
    }
    else if (result == CACHE_PAGE_IO_FAIL)
    {
        sErrors++;
        sPsramReadFail++;
        sFillQuarantine++;
        noteFaultLocked(CS_FAULT_PSRAM, sFillSector);
        cacheSdDegrade(CS_FAULT_PSRAM);
        sFillDrop++;
        sFillDropReason = FILL_DROP_DEGRADE;
    }
    else
    {
#ifdef CACHE_PROBE_MISMATCH_DIAG
        if (!sProbeFirstValid && sFillPageTask.first_bad_offset < CS_SECTOR_BYTES)
        {
            sProbeFirstSector = sFillSector;
            sProbeFirstOffset = sFillPageTask.first_bad_offset;
            sProbeFirstExpected = sFillStage[sProbeFirstOffset];
            sProbeFirstActual = sFillPageTask.scratch[sProbeFirstOffset & 31u];
            sProbeFirstRefCrc = crc32Update(0xFFFFFFFFu, sFillStage,
                                           CS_SECTOR_BYTES) ^ 0xFFFFFFFFu;
            sProbeFirstTagCrc = sFillPageTask.expected_table_crc;
            sProbeFirstValid = 1;
        }
#endif
        sErrors++;
        sL2CrcFail++;
        noteFaultLocked(CS_FAULT_L2_CRC, sFillSector);
        cacheSdDegrade(CS_FAULT_L2_CRC);
        sFillDrop++;
        sFillDropReason = FILL_DROP_SUPERSEDED;
    }
    sFillIsProbe = 0;
    sFillState = FILL_RETIRED;
    return;
#else
    uint32_t frag = CS_SECTOR_BYTES - sFillOff;
    if (frag > PSRAM_CACHE_FRAG_BYTES)
        frag = PSRAM_CACHE_FRAG_BYTES;
    uint32_t chip = sFillIdx & 3u;
    uint32_t base = fillAddr(sFillOff);

    if (!sHw->psramRead(chip, base, sProbeChunk, frag))
    {
        sErrors++;
        sPsramReadFail++;
        sFillQuarantine++;
        noteFaultLocked(CS_FAULT_PSRAM, sFillSector);
        cacheSdDegrade(CS_FAULT_PSRAM);
        if (sState[sFillIdx] == 1)
            sState[sFillIdx] = 0;
        sFillDrop++;
        sFillDropReason = FILL_DROP_DEGRADE;
        sFillState = FILL_RETIRED;
        return;
    }

    sReadBytes += frag;
    uint32_t fragmentOffset = sFillOff;
    bool match = memcmp(sProbeChunk, &sFillStage[fragmentOffset], frag) == 0;
#ifdef CACHE_PROBE_MISMATCH_DIAG
    uint32_t mismatchOffset = 0, expected = 0, actual = 0, refCrc = 0;
    if (!match && !sProbeFirstValid)
    {
        while (mismatchOffset < frag &&
               sProbeChunk[mismatchOffset] == sFillStage[fragmentOffset + mismatchOffset])
            mismatchOffset++;
        expected = sFillStage[fragmentOffset + mismatchOffset];
        actual = sProbeChunk[mismatchOffset];
        refCrc = crc32Update(0xFFFFFFFFu, sFillStage, CS_SECTOR_BYTES) ^ 0xFFFFFFFFu;
    }
#endif
    sFillOff += frag;
    bool complete = (sFillOff >= CS_SECTOR_BYTES);
    if (!match)
        complete = true;
    if (!complete)
        return; // keep probing in bounded fragments

    uint32_t save = sHw->irqSave();
    bool epochOk = (sFillEpoch == sCacheEpoch) &&
                   (sFillMediaEpoch == sCs.media_epoch) &&
                   (sFillWriteEpoch == sCs.write_epoch);
    if (!epochOk)
    {
        // A write or media change can occur during psramRead. Discard the
        // comparison even if the bytes now differ from the old SD snapshot.
        sFillDropReason = FILL_DROP_EPOCH;
        sFillDrop++;
    }
    else if (match)
    {
        sL2HitVerified++;
        sProbeOk++;
#ifdef CACHE_SHADOW_STRIPES_4CHIP
        sShadowBanks[sFillIdx & 3u] |= 1u << sFillStripe;
        sShadowVerify[sFillIdx & 3u]++;
#endif
#ifdef CACHE_M4_PROMOTE_M5_AFTER_PROBES
        // Qualify the transport with independent SD comparisons before trying
        // the demand hit path. A separate experimental switch controls whether
        // promoted M5 may keep filling in the acknowledged quiet window.
        if (sL2Mode == CACHE_L2_MODE_M4 && sProbeOk >= 32u &&
            !sL2CrcFail && !sPsramReadFail && !sPsramFillFail && !sDegraded)
        {
            sM4Promoted = 1;
            sL2Mode = CACHE_L2_MODE_M5;
        }
#endif
    }
    else
    {
        // Real content mismatch: refuse the entry and fall back to SD.
#ifdef CACHE_PROBE_MISMATCH_DIAG
        if (!sProbeFirstValid)
        {
            sProbeFirstSector = sFillSector;
            sProbeFirstOffset = fragmentOffset + mismatchOffset;
            sProbeFirstExpected = expected;
            sProbeFirstActual = actual;
            sProbeFirstRefCrc = refCrc;
            sProbeFirstTagCrc = sCrc[sFillIdx];
            sProbeFirstValid = 1;
        }
#endif
        if (sState[sFillIdx] != 1)
            sState[sFillIdx] = 0;
        sL2CrcFail++;
        sErrors++;
        cs_note_fault(&sCs, CS_FAULT_L2_CRC, sFillSector);
        sFillDrop++;
        sFillDropReason = FILL_DROP_SUPERSEDED;
    }
    sHw->irqRestore(save);
    sFillIsProbe = 0;
    sFillState = FILL_RETIRED;
#ifdef CACHE_SHADOW_STRIPES_4CHIP
    if (epochOk && !match)
        cacheSdDegrade(CS_FAULT_L2_CRC);
#endif
#endif
}

static CACHE_RAM_CODE void fillStep(void)
{
    if (sFillIsProbe)
    {
        probeStep();
        return;
    }
#ifdef CACHE_FULL_PAGE_4CHIP
    cachePageIo io = { sHw->psramRead, sHw->psramWrite };
    uint32_t beforeRead = sFillPageTask.read_bytes;
    uint32_t beforeWrite = sFillPageTask.write_bytes;
    cachePageResult result = cachePageStoreStep(&sPageStore, &sFillPageTask, &io);
    sReadBytes += sFillPageTask.read_bytes - beforeRead;
    sWriteBytes += sFillPageTask.write_bytes - beforeWrite;
    sFillOff = sFillPageTask.offset;
    if (result == CACHE_PAGE_BUSY) return;
    if (result == CACHE_PAGE_DONE)
    {
        bool epochOk = sFillEpoch == sCacheEpoch &&
                       sFillMediaEpoch == sCs.media_epoch &&
                       sFillWriteEpoch == sCs.write_epoch;
        if (!epochOk)
        {
            cachePageMapInvalidate(&sPageStore.map, &sFillPageTask.lease);
            sFillDrop++;
            sFillDropReason = FILL_DROP_EPOCH;
            sFillState = FILL_DROP;
            return;
        }
        sFills++;
        sFillCommit++;
        uint32_t chip = cachePageMapChip(sFillPageTask.lease.slot);
        uint32_t set = sFillPageTask.lease.slot / CACHE_PAGE_CHIPS;
        sPageFill[chip]++;
        if (set > sPageMaxSet[chip]) sPageMaxSet[chip] = set;
        sFillState = FILL_COMMITTED;
        if (sL2Mode == CACHE_L2_MODE_M4 &&
            cachePageStoreBeginProbe(&sPageStore, &sFillPageTask,
                                      sFillSector, sFillEpoch, sFillStage))
        {
            sFillOff = 0;
            sFillIsProbe = 1;
            sFillState = FILL_ADMITTED;
            sL2HitAttempt++;
            sProbeTry++;
            return;
        }
        sFillState = FILL_RETIRED;
        return;
    }
    if (result == CACHE_PAGE_IO_FAIL)
    {
        sErrors++;
        sPsramFillFail++;
        sFillQuarantine++;
        noteFaultLocked(CS_FAULT_FILL, sFillSector);
        cacheSdDegrade(CS_FAULT_FILL);
        sFillDropReason = FILL_DROP_DEGRADE;
    }
    else if (result == CACHE_PAGE_CORRUPT)
    {
        sErrors++;
        sL2CrcFail++;
        noteFaultLocked(CS_FAULT_L2_CRC, sFillSector);
        cacheSdDegrade(CS_FAULT_L2_CRC);
        sFillDropReason = FILL_DROP_DEGRADE;
    }
    else
    {
        sFillDropReason = FILL_DROP_EPOCH;
    }
    sFillDrop++;
    sFillState = FILL_QUARANTINED;
    return;
#else
    uint32_t frag = CS_SECTOR_BYTES - sFillOff;
    if (frag > PSRAM_CACHE_FRAG_BYTES)
        frag = PSRAM_CACHE_FRAG_BYTES;
    uint32_t chip = sFillIdx & 3u;
    uint32_t base = fillAddr(sFillOff);

    if (!sHw->psramWrite(chip, base, &sFillStage[sFillOff], frag))
    {
        sErrors++;
        sPsramFillFail++;
        sFillQuarantine++;
        noteFaultLocked(CS_FAULT_FILL, sFillSector);
        // A failed transport cannot be assumed quiescent: quarantine the
        // backend rather than reusing the fragment path (design 8.1/11).
        cacheSdDegrade(CS_FAULT_FILL);
        sFillState = FILL_QUARANTINED;
        if (sState[sFillIdx] == 1)
            sState[sFillIdx] = 0;
        sFillDrop++;
        sFillDropReason = FILL_DROP_DEGRADE;
        return;
    }

    sWriteBytes += frag;
    sFillCrc = crc32Update(sFillCrc, &sFillStage[sFillOff], frag);
    sFillOff += frag;

    if (sFillOff < CS_SECTOR_BYTES)
        return;

    sFillCrc ^= 0xFFFFFFFFu;
    sFillState = FILL_VERIFIED;

    // Check and commit in one critical section: a write barrier arriving here
    // must not produce a checked-old / committed-new entry (design 13.3/R2).
    // The source identity captured at snapshot time must still match.
    uint32_t save = sHw->irqSave();
    if (sState[sFillIdx] == 1 && sFillEpoch == sCacheEpoch &&
        sFillMediaEpoch == sCs.media_epoch &&
        sFillWriteEpoch == sCs.write_epoch)
    {
        sTag[sFillIdx] = sFillSector;
        sTagEpoch[sFillIdx] = sFillEpoch;
        sCrc[sFillIdx] = sFillCrc;
#ifdef CACHE_SHADOW_STRIPES_4CHIP
        sStripe[sFillIdx] = sFillStripe;
#endif
        sState[sFillIdx] = 2;
        sFills++;
        sFillCommit++;
        sFillState = FILL_COMMITTED;
    }
    else
    {
        if (sState[sFillIdx] == 1)
            sState[sFillIdx] = 0;
        sFillDrop++;
        sFillDropReason = FILL_DROP_EPOCH;
        sFillState = FILL_DROP;
    }
    sHw->irqRestore(save);
#ifdef CACHE_SHADOW_STRIPES_4CHIP
    if (sFillState == FILL_COMMITTED)
    {
        // Reuse the immutable SD snapshot for an immediate full-sector read
        // probe. A newly written high stripe is never counted as verified
        // until all 512 B have returned and matched the independent SD copy.
        sFillOff = 0;
        sFillIsProbe = 1;
        sFillState = FILL_ADMITTED;
        sL2HitAttempt++;
        sProbeTry++;
        return;
    }
#endif
    // Cleanup finished; the dedicated snapshot buffer is reusable.
    sFillIsProbe = 0;
    sFillState = FILL_RETIRED;
#endif
}

// A snapshot whose source response was retired without a completed send can
// never be admitted; drop it (design section 7.3). Runs on the main loop.
static CACHE_RAM_CODE void fillReconcile(void)
{
    // An admitted snapshot can remain blocked across writes/media changes.
    // Retire its stale namespace before considering it for transport again;
    // otherwise it monopolizes the single snapshot buffer indefinitely.
    if ((sFillState == FILL_SNAPSHOT_READY || sFillState == FILL_ADMITTED ||
         sFillState == FILL_TRANSFERRING) &&
        (sFillEpoch != sCacheEpoch || sFillMediaEpoch != sCs.media_epoch ||
         sFillWriteEpoch != sCs.write_epoch))
    {
        fillAbortReason(FILL_DROP_EPOCH);
        sFillDrop++;
        return;
    }
    if (sFillState != FILL_SNAPSHOT_READY)
        return;
    const cs_slot* sl = &sCs.slots[sFillWaitSlot];
    if (sl->version == sFillWaitVersion &&
        (sl->state == CS_SLOT_DMA || sl->state == CS_SLOT_READY))
        return; // response still pending with the host
    fillAbortReason(FILL_DROP_SUPERSEDED);
    sFillDrop++;
}

CACHE_RAM_CODE cacheSdStepResult cacheSdStep(void)
{
    bool did = false;
    sDidPublish = 0;
    // Terminal backfill states are transient labels; the single bounded slot is
    // reusable for the next task once the previous one is retired.
    if (sFillState == FILL_RETIRED || sFillState == FILL_DROP ||
        sFillState == FILL_COMMITTED || sFillState == FILL_QUARANTINED)
        sFillState = FILL_NONE;
    uint64_t t0 = sHw->nowUs();

    // 1. Reclaim a finished DMA0 response. This is where a snapshot waiting on
    //    the response's handover becomes admitted (design section 7.3).
    {
        uint32_t save = sHw->irqSave();
        dmaReclaimLocked();
        sHw->irqRestore(save);
    }

    // 2. Service the foreground demand first (E3/E4/E5 have priority over the
    //    background fill); this also lets a demand replace a pending fill.
    if (!sJobActive)
    {
        cs_job job;
        uint32_t save = sHw->irqSave();
        bool ok = cs_accept_intent(&sCs, &job);
        if (ok)
            job.source_epoch = sCacheEpoch; // immutable at accept time
        sHw->irqRestore(save);
        if (ok)
        {
            jobStart(&job);
            did = true;
        }
    }

    if (sJobActive)
    {
        jobStep();
        did = true;
    }

    // 3. Reconcile a snapshot that can no longer be admitted.
    fillReconcile();

    // 4. Advance an admitted background transport only when this step did NOT
    //    publish a demand, unless the M3 legacy contrast build explicitly asks
    //    for the old behaviour. This is the decoupling of section 7.2. Work is
    //    additionally bounded by the measured step budget (design 7.4).
    bool overBudget = (sTStepMaxUs != 0) &&
                      ((uint32_t)(sHw->nowUs() - t0) >= sTStepMaxUs);
    // Prepared/acked responses remain foreground work after job completion.
    // Recheck at every fragment, including fragments of an older fill.
#ifdef CACHE_FILL_QUIET_EXPERIMENT
    bool dmaBusy = sHw->dma0Busy();
    bool cartIdle = sHw->cartIdle();
    bool foreground = sCs.intent_valid || sCs.completion_valid ||
                      sCs.binding_valid || sCs.write_pending ||
                      dmaBusy || !cartIdle;
    // Experimental M3/M4 path: the automatically queued next sector has an
    // acknowledged, immutable binding, but no E5 is in progress. The response
    // slot and binding are untouched. Each main-loop pass may launch at most
    // one 32-byte fragment after a full quiet interval; a new host command
    // resets the interval. Real IRQ/PSRAM coexistence still needs board proof.
    uint32_t quietNow = (uint32_t)sHw->nowUs();
    uint32_t hostSeq = sHostCommandSeq;
    bool quietMode = (sL2Mode == CACHE_L2_MODE_M3 ||
                      sL2Mode == CACHE_L2_MODE_M4);
#ifdef CACHE_M5_QUIET_FILL_AFTER_PROMOTION
    quietMode = quietMode || (sL2Mode == CACHE_L2_MODE_M5 && sM4Promoted);
#endif
#if defined(CACHE_FULL_PAGE_4CHIP) && defined(CACHE_PAGE_M5_EXPERIMENT)
    quietMode = quietMode || (sL2Mode == CACHE_L2_MODE_M5);
#endif
    bool quietBinding = quietMode && sCs.intent_valid &&
                        sCs.intent_consumed && sCs.binding_valid &&
                        !sCs.completion_valid && !sCs.write_pending &&
                        cs_identity_equal(&sCs.binding.id, &sCs.intent) &&
                        sCs.binding.offer_id == sCs.acked_offer_id &&
                        sCs.resp_state == CS_RESP_ACKED && !dmaBusy && cartIdle &&
                        (uint32_t)(quietNow - sLastHostCommandUs) >= CACHE_FILL_QUIET_US &&
                        (uint32_t)(quietNow - sLastQuietFragmentUs) >= CACHE_FILL_QUIET_US;
    bool allowFill = !foreground || quietBinding;
#else
    bool foreground = sCs.intent_valid || sCs.completion_valid ||
                      sCs.binding_valid || sCs.write_pending ||
                      sHw->dma0Busy() || !sHw->cartIdle();
    bool allowFill = !foreground;
#endif
#if defined(CACHE_WATCH_ACTIVE_MARKERS) || defined(CACHE_SD_HOST)
    // Observe the existing gate, without extra hardware calls or changing its
    // decision. Sample one in 1024 eligible steps, not millions of SRAM
    // read/modify/writes per second. Mask describes the last sampled step.
    if (sFillState != FILL_ADMITTED && sFillState != FILL_TRANSFERRING)
    {
        sGateMask = sGatePhase = 0;
    }
    else if ((sGatePhase++ & 1023u) == 0)
    {
        uint32_t mask = (sCs.intent_valid ? CACHE_GATE_INTENT : 0u) |
                        (sCs.completion_valid ? CACHE_GATE_COMPLETION : 0u) |
                        (sCs.binding_valid ? CACHE_GATE_BINDING : 0u) |
                        (sCs.write_pending ? CACHE_GATE_WRITE : 0u);
        if (foreground && !mask) mask |= CACHE_GATE_BUS;
        if (sJobActive) mask |= CACHE_GATE_JOB;
        if (sDidPublish) mask |= CACHE_GATE_PUBLISH;
        if (overBudget) mask |= CACHE_GATE_BUDGET;
        sGateMask = mask;
        sGateChecks++;
        if ((!FILL_LEGACY_PUBLISH && (sDidPublish || !allowFill)) ||
            sJobActive || overBudget)
            sGateBlocked++;
    }
#endif
    if ((!sDidPublish || FILL_LEGACY_PUBLISH) && !sJobActive &&
        (allowFill || FILL_LEGACY_PUBLISH) && !overBudget &&
        (sFillState == FILL_ADMITTED || sFillState == FILL_TRANSFERRING))
    {
        if (sFillState == FILL_ADMITTED && sTIdleAdmitUs != 0 && !sHw->cartIdle())
        {
            sFillDefer++;
        }
#ifdef CACHE_FILL_QUIET_EXPERIMENT
        else if (hostSeq != sHostCommandSeq || sHw->dma0Busy() ||
                 !sHw->cartIdle() || sCs.write_pending ||
                 sFillEpoch != sCacheEpoch)
        {
            // An IRQ changed the state after the first gate sample. It can
            // still arrive during a fragment; that overlap is counted below.
            sFillDefer++;
        }
#endif
        else
        {
            sFillState = FILL_TRANSFERRING;
#ifdef CACHE_FILL_QUIET_EXPERIMENT
            uint32_t fragmentStart = (uint32_t)sHw->nowUs();
            sQuietFragmentActive = quietBinding ? 1u : 0u;
#endif
            fillStep();
#ifdef CACHE_FILL_QUIET_EXPERIMENT
            sQuietFragmentActive = 0;
            if (quietBinding)
            {
                uint32_t elapsed = (uint32_t)sHw->nowUs() - fragmentStart;
                sLastQuietFragmentUs = (uint32_t)sHw->nowUs();
                sQuietFillFragments++;
                if (elapsed > sQuietFillMaxUs) sQuietFillMaxUs = elapsed;
            }
#endif
            did = true;
        }
    }

    if (sDidPublish)
        return CACHE_SD_STEP_DEMAND_PUBLISHED;
    return did ? CACHE_SD_STEP_WORKED : CACHE_SD_STEP_IDLE;
}

CACHE_RAM_CODE uint8_t* cacheSdSlotBuffer(uint8_t slot)
{
    return sSlotBuf[slot < CS_SLOT_COUNT ? slot : 0];
}

// ---------------------------------------------------------------------------
// Diagnostics
// ---------------------------------------------------------------------------

CACHE_RAM_CODE uint32_t cacheSdHits(void) { return sHits; }
CACHE_RAM_CODE uint32_t cacheSdMisses(void) { return sMisses; }
CACHE_RAM_CODE uint32_t cacheSdFills(void) { return sFills; }
CACHE_RAM_CODE uint32_t cacheSdErrors(void) { return sErrors; }

CACHE_RAM_CODE uint32_t cacheSdOfferId(void)
{
    return sCs.binding_valid ? sCs.binding.offer_id
                             : (sCs.completion_valid ? sCs.completion.offer_id : 0);
}

CACHE_RAM_CODE uint32_t cacheSdAckedOfferId(void) { return sCs.acked_offer_id; }

CACHE_RAM_CODE const cs_fault_record* cacheSdFirstFault(void) { return &sCs.first_fault; }

CACHE_RAM_CODE void cacheSdGetCounters(cacheSdCounters* out)
{
    if (!out)
        return;
    out->sd_token_obsolete = sSdTokenObsolete;
    out->sd_token_live_conflict = sSdTokenLiveConflict;
    out->sd_transfer_error = sSdTransferError;
    out->l2_hit_attempt = sL2HitAttempt;
    out->l2_hit_verified = sL2HitVerified;
    out->l2_crc_fail = sL2CrcFail;
    out->psram_read_fail = sPsramReadFail;
    out->psram_fill_fail = sPsramFillFail;
    out->fill_admit = sFillAdmit;
    out->fill_drop = sFillDrop;
    out->fill_defer = sFillDefer;
    out->fill_commit = sFillCommit;
    out->fill_quarantine = sFillQuarantine;
    out->fill_drop_reason = sFillDropReason;
    out->mode = sL2Mode;
    out->enabled = sEnabled;
    out->degraded = sDegraded;
    out->epoch = sCacheEpoch;
    out->sd_miss = sMisses;
#ifdef CACHE_BOOT_SECTOR0_SEED
    out->boot_seed_use = sBootSeedUse;
#else
    out->boot_seed_use = 0;
#endif
    out->demand_hit_try = sHits;
    out->demand_hit_ok = sDemandHitOk;
    out->demand_non_psram_ok = sDemandNonPsramOk;
    out->probe_try = sProbeTry;
    out->probe_ok = sProbeOk;
    out->read_bytes = sReadBytes;
    out->write_bytes = sWriteBytes;
#ifdef CACHE_PROBE_MISMATCH_DIAG
    out->probe_first_valid = sProbeFirstValid;
    out->probe_first_sector = sProbeFirstSector;
    out->probe_first_offset = sProbeFirstOffset;
    out->probe_first_expected = sProbeFirstExpected;
    out->probe_first_actual = sProbeFirstActual;
    out->probe_first_ref_crc = sProbeFirstRefCrc;
    out->probe_first_tag_crc = sProbeFirstTagCrc;
#else
    out->probe_first_valid = out->probe_first_sector = out->probe_first_offset = 0;
    out->probe_first_expected = out->probe_first_actual = 0;
    out->probe_first_ref_crc = out->probe_first_tag_crc = 0;
#endif
#ifdef CACHE_SHADOW_STRIPES_4CHIP
    for (uint32_t chip = 0; chip < 4u; chip++)
    {
        out->shadow_banks[chip] = sShadowBanks[chip];
        out->shadow_verify[chip] = sShadowVerify[chip];
    }
#else
    for (uint32_t chip = 0; chip < 4u; chip++)
        out->shadow_banks[chip] = out->shadow_verify[chip] = 0;
#endif
#ifdef CACHE_FULL_PAGE_4CHIP
    for (uint32_t chip = 0; chip < 4u; chip++)
    {
        out->page_fill[chip] = sPageFill[chip];
        out->page_probe[chip] = sPageProbe[chip];
        out->page_hit_ok[chip] = sPageHitOk[chip];
        out->page_max_set[chip] = sPageMaxSet[chip];
    }
    out->page_valid_sectors = cachePageStoreValidSectors(&sPageStore, sCacheEpoch);
#else
    for (uint32_t chip = 0; chip < 4u; chip++)
        out->page_fill[chip] = out->page_probe[chip] =
            out->page_hit_ok[chip] = out->page_max_set[chip] = 0;
    out->page_valid_sectors = 0;
#endif
    out->fill_state = sFillState;
    out->fill_sector = sFillSector;
    out->fill_offset = sFillOff;
    out->snapshot = sFillSnapshotId;
#ifdef CACHE_FILL_QUIET_EXPERIMENT
    out->quiet_fill_fragments = sQuietFillFragments;
    out->quiet_fill_max_us = sQuietFillMaxUs;
    out->quiet_irq_overlap = sQuietIrqOverlap;
    out->quiet_e5_overlap = sQuietE5Overlap;
#else
    out->quiet_fill_fragments = out->quiet_fill_max_us = 0;
    out->quiet_irq_overlap = out->quiet_e5_overlap = 0;
#endif
#if defined(CACHE_WATCH_ACTIVE_MARKERS) || defined(CACHE_SD_HOST)
    out->gate_measured = 1;
    out->gate_checks = sGateChecks;
    out->gate_blocked = sGateBlocked;
    out->gate_mask = sGateMask;
#else
    out->gate_measured = 0;
    out->gate_checks = out->gate_blocked = out->gate_mask = 0;
    // Read-only observer: no per-step writes or extra hardware callbacks.
    // Does not report DMA/bus/budget or the actual last admission decision.
    if (sFillState == FILL_ADMITTED || sFillState == FILL_TRANSFERRING)
        out->gate_mask = (sCs.intent_valid ? CACHE_GATE_INTENT : 0u) |
                        (sCs.completion_valid ? CACHE_GATE_COMPLETION : 0u) |
                        (sCs.binding_valid ? CACHE_GATE_BINDING : 0u) |
                        (sCs.write_pending ? CACHE_GATE_WRITE : 0u) |
                        (sJobActive ? CACHE_GATE_JOB : 0u);
#endif
}

CACHE_RAM_CODE uint32_t cacheSdSectors(void)
{
#ifdef CACHE_FULL_PAGE_4CHIP
    return cachePageStoreValidSectors(&sPageStore, sCacheEpoch);
#else
    uint32_t n = 0;
    uint32_t ep = sCacheEpoch;
    for (uint32_t i = 0; i < CACHE_SD_SECTORS; i++)
        if (sState[i] == 2 && sTagEpoch[i] == ep)
            n++;
    return n;
#endif
}

CACHE_RAM_CODE const cs_state* cacheSdState(void)
{
    return &sCs;
}

#ifndef CACHE_SD_HOST
// ---------------------------------------------------------------------------
// Target adapter (real SD/PIO/DMA/IRQ). Declared here so the host build never
// references the pico SDK.
// ---------------------------------------------------------------------------
static CACHE_RAM_CODE bool hwSdBeginRead(uint8_t* dst, uint32_t sector);
static CACHE_RAM_CODE bool hwSdReady(void);
static CACHE_RAM_CODE uint32_t hwSdTransferId(void);
static CACHE_RAM_CODE bool hwPsramRead(uint32_t chip, uint32_t addr, void* buf, uint32_t len);
static CACHE_RAM_CODE bool hwPsramWrite(uint32_t chip, uint32_t addr, const void* buf, uint32_t len);
static CACHE_RAM_CODE bool hwDma0Busy(void);
static CACHE_RAM_CODE bool hwCartIdle(void);
static CACHE_RAM_CODE bool hwSdError(void);
static CACHE_RAM_CODE uint32_t hwIrqSave(void);
static CACHE_RAM_CODE void hwIrqRestore(uint32_t save);
static CACHE_RAM_CODE uint64_t hwNowUs(void);

CACHE_RAM_CODE const cacheSdHw* cacheSdDefaultHw(void);

// The callbacks are also read while interrupts are masked; keep the table in
// SRAM along with the callback code, not in Flash .rodata.
static cacheSdHw sTargetHw = {
    hwSdBeginRead, hwSdReady, hwSdTransferId,
    hwPsramRead, hwPsramWrite,
    hwDma0Busy, hwCartIdle, hwSdError,
    hwIrqSave, hwIrqRestore, hwNowUs,
};

CACHE_RAM_CODE const cacheSdHw* cacheSdDefaultHw(void) { return &sTargetHw; }

extern bool ntrc_cacheSdBegin(uint8_t* dst, uint32_t sector);
extern bool ntrc_cacheSdReady(void);
extern uint32_t ntrc_cacheSdTransferId(void);
extern bool ntrc_cacheSdError(void);
extern volatile uint32_t gCartSdRecoveryPending;

static CACHE_RAM_CODE bool hwSdBeginRead(uint8_t* dst, uint32_t sector) { return ntrc_cacheSdBegin(dst, sector); }
static CACHE_RAM_CODE bool hwSdReady(void) { return ntrc_cacheSdReady(); }
static CACHE_RAM_CODE uint32_t hwSdTransferId(void) { return ntrc_cacheSdTransferId(); }
static CACHE_RAM_CODE bool hwPsramRead(uint32_t chip, uint32_t addr, void* buf, uint32_t len) { return psramRead(chip, addr, buf, len); }
static CACHE_RAM_CODE bool hwPsramWrite(uint32_t chip, uint32_t addr, const void* buf, uint32_t len) { return psramWrite(chip, addr, buf, len); }
static CACHE_RAM_CODE bool hwDma0Busy(void) { return dma_channel_is_busy(0); }
static CACHE_RAM_CODE bool hwCartIdle(void) { return !gCartSdRecoveryPending && gpio_get(PIN_CEB) && gpio_get(PIN_CS2); }
static CACHE_RAM_CODE bool hwSdError(void) { return ntrc_cacheSdError(); }
static CACHE_RAM_CODE uint32_t hwIrqSave(void) { return save_and_disable_interrupts(); }
static CACHE_RAM_CODE void hwIrqRestore(uint32_t save) { restore_interrupts(save); }
static CACHE_RAM_CODE uint64_t hwNowUs(void) { return time_us_64(); }
#endif // !CACHE_SD_HOST

#endif // CACHE_SD_ENABLED
