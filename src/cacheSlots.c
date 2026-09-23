#include "cacheSlots.h"

// ---------------------------------------------------------------------------
// internal helpers (bounded, no I/O)
// ---------------------------------------------------------------------------

static CACHE_RAM_CODE void slotReleaseMatching(cs_state* s, uint8_t slot, uint32_t version)
{
    if (slot >= CS_SLOT_COUNT)
        return;
    cs_slot* sl = &s->slots[slot];
    if (sl->state == CS_SLOT_DMA)
        return; // hardware may still own it
    if (sl->version != version)
        return;
    sl->state = CS_SLOT_FREE;
    sl->request_id = 0;
}

static CACHE_RAM_CODE int slotAlloc(cs_state* s, uint32_t request_id, uint32_t* version_out)
{
    for (uint32_t i = 0; i < CS_SLOT_COUNT; i++)
    {
        if (s->slots[i].state == CS_SLOT_FREE)
        {
            s->slots[i].state = CS_SLOT_RESERVED;
            s->slots[i].version = ++s->next_slot_version;
            s->slots[i].request_id = request_id;
            if (version_out)
                *version_out = s->slots[i].version;
            return (int)i;
        }
    }
    return -1;
}

// Note the last revocation reason (design section 6.1). Revocation always
// closes fast ready and the ack credential before any slot is released.
static CACHE_RAM_CODE void noteInvoke(cs_state* s, uint8_t reason)
{
    s->last_invalidate_reason = reason;
}

// Drop a binding/completion whose identity no longer matches the current
// intent, releasing any slot it still owns.
static CACHE_RAM_CODE void dropStale(cs_state* s, cs_completion* c, uint8_t* valid, uint8_t reason)
{
    if (!*valid)
        return;
    if (!s->intent_valid || !cs_identity_equal(&c->id, &s->intent) ||
        c->id.write_epoch != s->write_epoch)
    {
        slotReleaseMatching(s, c->slot, c->slot_version);
        *valid = 0;
        // The acknowledgement is only meaningful while its binding lives.
        if (c == &s->binding)
        {
            s->acked_offer_id = 0;
            noteInvoke(s, reason);
            s->resp_state = CS_RESP_EMPTY;
        }
    }
}

CACHE_RAM_CODE void cs_note_fault_ex(cs_state* s, uint8_t kind, uint32_t detail,
                                     uint8_t producer, uint32_t ctx0, uint32_t ctx1)
{
    s->fault_seq++;
    if (s->first_fault.valid)
        return;
    cs_fault_record* f = &s->first_fault;
    f->valid = 1;
    f->kind = kind;
    f->producer = producer;
    f->resp_state = s->resp_state;
    f->last_invalidate = s->last_invalidate_reason;
    f->write_pending = s->write_pending;
    f->slot = s->binding_valid ? s->binding.slot : 0;
    f->intent_consumed = s->intent_consumed;
    f->seq = s->fault_seq;
    f->id = s->intent;
    f->offer_id = s->binding_valid ? s->binding.offer_id
                                   : (s->completion_valid ? s->completion.offer_id : 0);
    f->acked_offer_id = s->acked_offer_id;
    f->sampled_ready = 0;
    f->completion_valid = s->completion_valid;
    f->binding_valid = s->binding_valid;
    f->slot_version = s->binding_valid ? s->binding.slot_version
                                       : (s->completion_valid ? s->completion.slot_version : 0);
    f->detail = detail;
    f->ctx[0] = ctx0;
    f->ctx[1] = ctx1;
}

CACHE_RAM_CODE void cs_note_fault(cs_state* s, uint8_t kind, uint32_t detail)
{
    cs_note_fault_ex(s, kind, detail, CS_PRODUCER_MAIN, 0, 0);
}

CACHE_RAM_CODE void cs_note_e4_ack_failed(cs_state* s, uint32_t sampled_ready)
{
    s->fault_seq++;
    if (s->first_fault.valid)
        return;
    cs_fault_record* f = &s->first_fault;
    f->valid = 1;
    f->kind = CS_FAULT_E4_ACK_MISMATCH;
    f->producer = CS_PRODUCER_IRQ_E4;
    f->resp_state = s->resp_state;
    f->last_invalidate = s->last_invalidate_reason;
    f->write_pending = s->write_pending;
    f->slot = s->binding_valid ? s->binding.slot : 0;
    f->intent_consumed = s->intent_consumed;
    f->seq = s->fault_seq;
    f->id = s->intent;
    f->offer_id = s->binding_valid ? s->binding.offer_id
                                   : (s->completion_valid ? s->completion.offer_id : 0);
    f->acked_offer_id = s->acked_offer_id;
    f->sampled_ready = sampled_ready;
    f->completion_valid = s->completion_valid;
    f->binding_valid = s->binding_valid;
    f->slot_version = s->binding_valid ? s->binding.slot_version
                                       : (s->completion_valid ? s->completion.slot_version : 0);
    f->detail = 0;
    f->ctx[0] = 0;
    f->ctx[1] = 0;
}

CACHE_RAM_CODE const cs_fault_record* cs_first_fault(const cs_state* s) { return &s->first_fault; }

static CACHE_RAM_CODE void invalidateSession(cs_state* s, uint8_t reason)
{
    if (s->binding_valid)
    {
        slotReleaseMatching(s, s->binding.slot, s->binding.slot_version);
        s->binding_valid = 0;
        s->acked_offer_id = 0;
    }
    if (s->completion_valid)
    {
        slotReleaseMatching(s, s->completion.slot, s->completion.slot_version);
        s->completion_valid = 0;
    }
    // Reserved (filling) slots are released by the backend when it aborts the
    // corresponding job; retag the intent so its result can never bind.
    s->intent_valid = 0;
    s->intent_consumed = 0;
    s->resp_state = CS_RESP_EMPTY;
    noteInvoke(s, reason);
}

// ---------------------------------------------------------------------------

CACHE_RAM_CODE void cs_init(cs_state* s)
{
    for (uint32_t i = 0; i < CS_SLOT_COUNT; i++)
    {
        s->slots[i].state = CS_SLOT_FREE;
        s->slots[i].version = 0;
        s->slots[i].request_id = 0;
    }
    s->next_request_id = 1;
    s->next_job_id = 1;
    s->next_slot_version = 0;
    s->intent_valid = 0;
    s->intent_consumed = 0;
    s->binding_valid = 0;
    s->completion_valid = 0;
    s->cart_epoch = 1;
    s->media_epoch = 1;
    s->write_epoch = 1;
    s->write_token = 0;
    s->next_write_token = 0;
    s->write_pending = 0;
    s->intent_faulted = 0;
    s->retry_count = 0;
    s->c_intent_merged = s->c_intent_replaced = 0;
    s->c_completion_obsolete = s->c_slot_wait = 0;
    s->c_protocol_fault = s->c_barrier_reject = s->c_slot_reuse = 0;
    s->c_write_blocked = s->c_sd_retry = s->c_sd_fault = 0;
    s->c_dma_reclaim = 0;
    s->next_offer_id = 0;
    s->acked_offer_id = 0;
    s->c_e4_busy = s->c_e4_ready_queued = s->c_e4_ack_failed = 0;
    s->c_e5_ok = s->c_e5_no_ack = 0;
    s->c_e5_identity_mismatch = s->c_e5_slot_mismatch = 0;
    s->c_fifo_recovery = s->c_offer_assign = s->c_offer_retire = 0;
    s->c_e5_recovery = 0;
    s->resp_state = CS_RESP_EMPTY;
    s->last_invalidate_reason = CS_INV_NONE;
    s->degraded = 0;
    s->fault_seq = 0;
    for (uint32_t i = 0; i < sizeof(s->first_fault); i++)
        ((uint8_t*)&s->first_fault)[i] = 0;
}

CACHE_RAM_CODE uint32_t cs_read_intent(cs_state* s, uint32_t sector)
{
    if (s->intent_valid && !s->intent_faulted &&
        s->intent.sector == sector &&
        s->intent.cart_epoch == s->cart_epoch &&
        s->intent.media_epoch == s->media_epoch &&
        s->intent.write_epoch == s->write_epoch)
    {
        // Same logical request: merge. Never disturb a fixed/bound response.
        s->c_intent_merged++;
        return CS_INTENT_MERGED;
    }

    bool had = s->intent_valid;

    // Offer wraparound (design section 6.2): before the 32-bit counter can
    // reuse a value, retire every old reference and start a new session. Only
    // then is a new offer allowed to be assigned.
    if (s->next_offer_id >= 0xFFFF0000u)
    {
        invalidateSession(s, CS_INV_INTENT_REPLACED);
        s->next_offer_id = 0;
        s->c_offer_retire++;
    }

    // A new request supersedes any response the host did not consume yet.
    if (s->binding_valid)
    {
        slotReleaseMatching(s, s->binding.slot, s->binding.slot_version);
        s->binding_valid = 0;
        s->acked_offer_id = 0;
        noteInvoke(s, CS_INV_INTENT_REPLACED);
        s->resp_state = CS_RESP_EMPTY;
    }

    s->intent.request_id = s->next_request_id++;
    s->intent.cart_epoch = s->cart_epoch;
    s->intent.media_epoch = s->media_epoch;
    s->intent.write_epoch = s->write_epoch;
    s->intent.sector = sector;
    s->intent_valid = 1;
    s->intent_consumed = 0;
    // A new logical request starts a fresh retry budget (design section 6.1).
    s->intent_faulted = 0;
    s->retry_count = 0;
    if (s->resp_state == CS_RESP_RETIRED)
        s->resp_state = CS_RESP_EMPTY;

    if (had)
    {
        s->c_intent_replaced++;
        return CS_INTENT_REPLACED;
    }
    return CS_INTENT_QUEUED;
}

CACHE_RAM_CODE bool cs_poll_read(cs_state* s)
{
    if (!s->intent_valid)
        return false;
    // While a write sequence is pending every read completion is blocked
    // (design section 8.2); a stale binding cannot exist because
    // cs_write_begin() dropped it.
    if (s->write_pending)
        return false;

    if (s->binding_valid)
    {
        if (cs_identity_equal(&s->binding.id, &s->intent) &&
            s->binding.id.write_epoch == s->write_epoch)
        {
            // Idempotent: a repeated E4 for the same offer keeps the ack.
            s->acked_offer_id = s->binding.offer_id;
            s->resp_state = CS_RESP_ACKED;
            return true;
        }
        slotReleaseMatching(s, s->binding.slot, s->binding.slot_version);
        s->binding_valid = 0;
        s->acked_offer_id = 0;
        noteInvoke(s, CS_INV_INTENT_REPLACED);
        s->resp_state = CS_RESP_EMPTY;
    }

    if (s->completion_valid)
    {
        if (cs_identity_equal(&s->completion.id, &s->intent) &&
            s->completion.id.write_epoch == s->write_epoch &&
            s->completion.result == CS_RES_OK)
        {
            // The immutable binding copy was prepared by cs_commit().
            s->binding_valid = 1;
            s->completion_valid = 0;
            // E4 is the only caller of cs_poll_read(); recording the ack here
            // is the lightweight confirmation of section 6.2. The binding
            // cannot change between this ack and E5 (same highest-priority
            // cartridge IRQ, no main-loop writer).
            s->acked_offer_id = s->binding.offer_id;
            s->resp_state = CS_RESP_ACKED;
            return true;
        }
        // Obsolete/duplicate: retire it rather than leaving it to block the
        // mailbox; the backend already released its slot at abort time if the
        // result was not usable.
        if (!cs_identity_equal(&s->completion.id, &s->intent))
        {
            slotReleaseMatching(s, s->completion.slot, s->completion.slot_version);
            s->completion_valid = 0;
            s->c_completion_obsolete++;
            noteInvoke(s, CS_INV_MAILBOX_STALE);
        }
    }

    return false;
}

// Non-mutating E5 legality predicate for diagnostics/tests. The production
// IRQ arms length first; cs_consume_read() authorizes the data DMA.
CACHE_RAM_CODE bool cs_e5_precheck(const cs_state* s)
{
    if (!s->binding_valid || !s->intent_valid)
        return false;
    if (s->acked_offer_id == 0 || s->acked_offer_id != s->binding.offer_id)
        return false;
    if (!cs_identity_equal(&s->binding.id, &s->intent) ||
        s->binding.id.write_epoch != s->write_epoch)
        return false;
    uint8_t sl = s->binding.slot;
    if (sl >= CS_SLOT_COUNT ||
        s->slots[sl].state != CS_SLOT_READY ||
        s->slots[sl].version != s->binding.slot_version)
        return false;
    return true;
}

CACHE_RAM_CODE bool cs_consume_read(cs_state* s, uint32_t* sector, uint8_t* slot,
                     uint32_t* slot_version)
{
    // Strict legality (design section 5.4): a response may only be consumed
    // against the exact offer E4 acknowledged. PREPARED without ACKED, a
    // replaced intent, a write barrier or a re-allocated slot all reject.
    if (!s->binding_valid || !s->intent_valid)
    {
        s->c_e5_no_ack++;
        s->c_protocol_fault++;
        cs_note_fault_ex(s, CS_FAULT_E5_NO_ACK, 0, CS_PRODUCER_IRQ_E5, 0, 0);
        return false;
    }
    if (s->acked_offer_id == 0 || s->acked_offer_id != s->binding.offer_id)
    {
        s->c_e5_no_ack++;
        s->c_protocol_fault++;
        cs_note_fault_ex(s, CS_FAULT_E5_NO_ACK, s->binding.offer_id,
                         CS_PRODUCER_IRQ_E5, s->acked_offer_id, s->binding.offer_id);
        return false;
    }
    if (!cs_identity_equal(&s->binding.id, &s->intent) ||
        s->binding.id.write_epoch != s->write_epoch)
    {
        s->c_e5_identity_mismatch++;
        s->c_protocol_fault++;
        cs_note_fault_ex(s, CS_FAULT_E5_IDENTITY, s->binding.id.sector,
                         CS_PRODUCER_IRQ_E5, s->binding.id.request_id,
                         s->intent.request_id);
        return false;
    }

    uint8_t sl = s->binding.slot;
    if (sl >= CS_SLOT_COUNT ||
        s->slots[sl].state != CS_SLOT_READY ||
        s->slots[sl].version != s->binding.slot_version)
    {
        s->c_e5_slot_mismatch++;
        s->c_protocol_fault++;
        cs_note_fault_ex(s, CS_FAULT_E5_SLOT, sl, CS_PRODUCER_IRQ_E5,
                         s->slots[sl < CS_SLOT_COUNT ? sl : 0].version,
                         s->binding.slot_version);
        s->binding_valid = 0;
        s->acked_offer_id = 0;
        noteInvoke(s, CS_INV_SLOT_MISMATCH);
        s->resp_state = CS_RESP_EMPTY;
        return false;
    }

    // SENDING: hand the slot to DMA0.
    s->slots[sl].state = CS_SLOT_DMA;
    *sector = s->binding.id.sector;
    *slot = sl;
    *slot_version = s->binding.slot_version;
    s->binding_valid = 0;
    s->acked_offer_id = 0;
    s->c_e5_ok++;
    s->resp_state = CS_RESP_SENDING;
    // The request has been fulfilled and handed to the host. Clear the intent
    // so a later E3 for the same sector is a new logical request instead of
    // merging into an already-served one (design section 6.1). E5 is expected
    // to queue the next sector separately via a fresh E3.
    s->intent_valid = 0;
    s->intent_consumed = 0;
    noteInvoke(s, CS_INV_CONSUMED);
    return true;
}

CACHE_RAM_CODE void cs_dma_done(cs_state* s, uint8_t slot, uint32_t slot_version)
{
    if (slot >= CS_SLOT_COUNT)
        return;
    if (s->slots[slot].state == CS_SLOT_DMA &&
        s->slots[slot].version == slot_version)
    {
        s->slots[slot].state = CS_SLOT_FREE;
        s->slots[slot].request_id = 0;
        if (s->resp_state == CS_RESP_SENDING)
            s->resp_state = CS_RESP_RETIRED;
    }
}

CACHE_RAM_CODE bool cs_accept_intent(cs_state* s, cs_job* job)
{
    if (!s->intent_valid || s->intent_consumed)
        return false;
    if (s->write_pending)
    {
        s->c_write_blocked++;
        return false;
    }
    if (s->intent_faulted)
        return false;
    // Degrade only refuses the L2 data path; the backend still accepts the
    // demand and serves it from SD (design section 11).
    uint32_t version = 0;
    int sl = slotAlloc(s, s->intent.request_id, &version);
    if (sl < 0)
    {
        s->c_slot_wait++;
        return false;
    }
    if (version != s->slots[sl].version) { /* unreachable */ }
    // Count reuse for diagnostics.
    if (version > CS_SLOT_COUNT)
        s->c_slot_reuse++;

    job->id = s->intent;
    job->job_id = s->next_job_id++;
    job->slot = (uint8_t)sl;
    job->slot_version = version;
    job->source_epoch = 0; // filled in by the backend under the same section
    s->intent_consumed = 1;
    s->resp_state = CS_RESP_FILLING;
    return true;
}

CACHE_RAM_CODE bool cs_commit(cs_state* s, const cs_completion* c)
{
    if (c->slot >= CS_SLOT_COUNT)
        return false;
    cs_slot* sl = &s->slots[c->slot];
    if (sl->state != CS_SLOT_RESERVED || sl->version != c->slot_version)
        return false;

    bool matches = s->intent_valid && !s->write_pending &&
                   c->result == CS_RES_OK &&
                   c->id.request_id == s->intent.request_id &&
                   c->id.sector == s->intent.sector &&
                   c->id.write_epoch == s->write_epoch &&
                   c->id.cart_epoch == s->cart_epoch &&
                   c->id.media_epoch == s->media_epoch;

    if (!matches)
    {
        slotReleaseMatching(s, c->slot, c->slot_version);
        s->c_completion_obsolete++;
        noteInvoke(s, CS_INV_COMMIT_OBSOLETE);
        if (s->resp_state == CS_RESP_FILLING)
            s->resp_state = CS_RESP_EMPTY;
        return false;
    }

    if (s->completion_valid)
    {
        // Mailbox occupied. Overwrite only if the resident entry is already
        // stale; otherwise ask the caller to retry next step.
        if (cs_identity_equal(&s->completion.id, &s->intent))
            return false;
        slotReleaseMatching(s, s->completion.slot, s->completion.slot_version);
        s->completion_valid = 0;
        noteInvoke(s, CS_INV_MAILBOX_STALE);
    }

    s->completion = *c;
    // Immutable, non-zero publication identity (design section 6.2). 0 is never
    // a valid offer; cs_read_intent() retires the session before wraparound.
    uint32_t offer = ++s->next_offer_id;
    if (offer == 0)
        offer = ++s->next_offer_id;
    s->completion.offer_id = offer;
    // Prepare the descriptor before publishing ready, outside the E4 IRQ.
    // This does NOT acknowledge it; only cs_poll_read may set binding_valid.
    s->binding = s->completion;
    s->binding_valid = 0;
    s->acked_offer_id = 0;
    s->completion_valid = 1;
    s->c_offer_assign++;
    sl->state = CS_SLOT_READY;
    s->retry_count = 0;
    // PREPARED: descriptor is complete and the offer is published. Fast ready
    // is derived from this object (see cacheSd), never an independent truth.
    s->resp_state = CS_RESP_PREPARED;
    return true;
}

CACHE_RAM_CODE void cs_job_abort(cs_state* s, const cs_job* job)
{
    if (job->slot >= CS_SLOT_COUNT)
        return;
    cs_slot* sl = &s->slots[job->slot];
    if (sl->state == CS_SLOT_RESERVED && sl->version == job->slot_version)
    {
        sl->state = CS_SLOT_FREE;
        sl->request_id = 0;
        if (s->resp_state == CS_RESP_FILLING)
            s->resp_state = CS_RESP_EMPTY;
    }
}

CACHE_RAM_CODE bool cs_job_requeue(cs_state* s, const cs_job* job)
{
    // The abandoned hardware/software job no longer owns its slot.
    cs_job_abort(s, job);

    // Never resurrect a request a newer E3 already replaced.
    if (!s->intent_valid || !cs_identity_equal(&job->id, &s->intent))
        return false;
    if (s->intent_faulted)
        return false;

    // A pending write is a temporary block, not a failure: re-arm the intent
    // without spending retry budget. It is served once the write is durable.
    if (s->write_pending)
    {
        s->intent_consumed = 0;
        s->c_write_blocked++;
        return true;
    }

    s->retry_count++;
    if (s->retry_count > CS_MAX_RETRY)
    {
        // Diagnosable terminal fault: no silent "busy with no job" (section 9.3).
        s->intent_faulted = 1;
        s->c_sd_fault++;
        return false;
    }

    // Re-arm the live intent so the backend accepts it again.
    s->intent_consumed = 0;
    s->c_sd_retry++;
    return true;
}

// ---- Write lifecycle ------------------------------------------------------

CACHE_RAM_CODE uint32_t cs_write_begin(cs_state* s)
{
    if (s->write_pending)
        return s->write_token; // same write sequence: keep the barrier/token

    uint32_t t = ++s->next_write_token;
    if (t == 0)
        t = ++s->next_write_token;
    s->write_token = t;
    s->write_pending = 1;

    s->write_epoch++;
    if (s->write_epoch == 0)
        s->write_epoch = 1;

    // Revocation first closes fast ready and the ack credential (design 6.2),
    // then the slot ownership is handled by dropStale().
    noteInvoke(s, CS_INV_WRITE_BARRIER);
    dropStale(s, &s->binding, &s->binding_valid, CS_INV_WRITE_BARRIER);
    dropStale(s, &s->completion, &s->completion_valid, CS_INV_WRITE_BARRIER);

    // Re-attempt the current intent under the new barrier generation once the
    // write completes; until then cs_accept_intent refuses it.
    if (s->intent_valid)
    {
        s->intent.write_epoch = s->write_epoch;
        s->intent_consumed = 0;
        if (s->resp_state == CS_RESP_FILLING)
            s->resp_state = CS_RESP_EMPTY;
    }
    s->intent_faulted = 0;
    s->retry_count = 0;
    s->c_barrier_reject++;
    return t;
}

CACHE_RAM_CODE void cs_write_end(cs_state* s, uint32_t token)
{
    if (s->write_pending && token == s->write_token)
    {
        s->write_pending = 0;
        s->write_token = 0;
    }
}

CACHE_RAM_CODE bool cs_write_is_pending(const cs_state* s) { return s->write_pending != 0; }
CACHE_RAM_CODE bool cs_is_faulted(const cs_state* s) { return s->intent_faulted != 0; }

CACHE_RAM_CODE void cs_set_degraded(cs_state* s, bool degraded)
{
    s->degraded = degraded ? 1 : 0;
    if (degraded)
    {
        // Close new hit/fill admission. Already-published SRAM responses are
        // deliberately kept (design section 11).
        s->acked_offer_id = 0;
        noteInvoke(s, CS_INV_DEGRADE);
    }
}

CACHE_RAM_CODE bool cs_is_degraded(const cs_state* s) { return s->degraded != 0; }

CACHE_RAM_CODE const char* cs_response_state_name(const cs_state* s)
{
    switch (s->resp_state)
    {
        case CS_RESP_EMPTY: return "EMPTY";
        case CS_RESP_FILLING: return "FILLING";
        case CS_RESP_PREPARED: return "PREPARED";
        case CS_RESP_ACKED: return "ACKED";
        case CS_RESP_SENDING: return "SENDING";
        case CS_RESP_RETIRED: return "RETIRED";
        default: return "?";
    }
}

CACHE_RAM_CODE void cs_reset_cart(cs_state* s)
{
    s->cart_epoch++;
    if (s->cart_epoch == 0)
        s->cart_epoch = 1;
    invalidateSession(s, CS_INV_RESET);
}

CACHE_RAM_CODE void cs_set_media_epoch(cs_state* s, uint32_t media_epoch)
{
    s->media_epoch = media_epoch ? media_epoch : 1;
    invalidateSession(s, CS_INV_MEDIA);
}
