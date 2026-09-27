#include "cacheDiagLog.h"
#ifdef CACHE_FULL_PAGE_4CHIP
#include "cachePageMap.h"
#endif

bool cacheWatchGateWarm(cacheWatchGate* g, uint32_t now)
{
    if (!g->started) { g->started = true; g->boot_us = now; }
    if ((uint32_t)(now - g->boot_us) >= CACHE_WATCH_BOOT_QUIET_US) g->warm = true;
    return g->warm;
}

bool cacheWatchGateIdle(cacheWatchGate* g, uint32_t now, bool available,
                        const uint32_t activity[7])
{
    bool changed = !g->observed;
    for (unsigned i = 0; i < 7; i++) {
        changed |= g->activity[i] != activity[i];
        g->activity[i] = activity[i];
    }
    g->observed = true;
    if (changed || !g->warm) g->progress_us = now;
    if (changed || !available || !g->warm) g->changed_us = now;
    return g->warm && available && (uint32_t)(now - g->changed_us) >= CACHE_WATCH_IDLE_US;
}

bool cacheWatchGateStalled(const cacheWatchGate* g, uint32_t now)
{
    return g->warm && g->observed &&
        (uint32_t)(now - g->progress_us) >= CACHE_WATCH_STALLED_US;
}

void cacheTextStart(cacheTextStream* s, const char* format, unsigned count)
{
    s->format = format;
    s->count = count > 96 ? 96 : count;
    s->index = s->digits_left = s->zeroes = 0;
    s->negative = false;
}

int cacheTextNext(cacheTextStream* s)
{
    if (s->negative) { s->negative = false; return '-'; }
    if (s->zeroes) { s->zeroes--; return '0'; }
    if (s->digits_left) return s->digits[--s->digits_left];
    if (!s->format || !*s->format) { s->format = NULL; return -1; }
    int ch = *s->format++;
    if (ch != '%') return ch;
    if (*s->format == '%') { s->format++; return '%'; }
    unsigned width = 0;
    if (*s->format == '0') {
        s->format++;
        if (*s->format >= '1' && *s->format <= '8') width = *s->format++ - '0';
    }
    if (*s->format == 'l') s->format++;
    char spec = *s->format;
    if (!spec) { s->format = NULL; return '?'; }
    s->format++;
    if ((spec != 'u' && spec != 'd' && spec != 'X') || s->index >= s->count)
        return '?';
    uint32_t value = s->values[s->index++];
    if (spec == 'd' && (int32_t)value < 0) {
        s->negative = true;
        value = 0u - value;
    }
    const unsigned base = spec == 'X' ? 16u : 10u;
    do {
        unsigned digit = value % base;
        s->digits[s->digits_left++] = digit < 10 ? '0' + digit : 'A' + digit - 10;
        value /= base;
    } while (value); // at most 10 uint32 digits
    s->zeroes = width > s->digits_left ? width - s->digits_left : 0;
    if (s->negative) { s->negative = false; return '-'; }
    if (s->zeroes) { s->zeroes--; return '0'; }
    return s->digits[--s->digits_left];
}

void cacheSummaryStartLine(cacheTextStream* s, uint32_t capacity_bytes,
                           const cacheSdCounters* c)
{
    uint32_t used = c->page_valid_sectors * 512u;
    if (used > capacity_bytes) used = capacity_bytes;
    uint32_t occupancy_bp = capacity_bytes ?
        (uint32_t)(((uint64_t)used * 10000u + capacity_bytes / 2u) /
                   capacity_bytes) : 0u;
    uint64_t demands = (uint64_t)c->demand_hit_ok + c->demand_non_psram_ok;
    uint32_t hit_bp = demands ?
        (uint32_t)(((uint64_t)c->demand_hit_ok * 10000u + demands / 2u) /
                   demands) : 0u;
    uint32_t total_mib_hundredths = (uint32_t)
        (((uint64_t)capacity_bytes * 100u + 524288u) / 1048576u);
    s->values[0] = total_mib_hundredths / 100u;
    s->values[1] = total_mib_hundredths % 100u;
    s->values[2] = used;
    s->values[3] = occupancy_bp / 100u;
    s->values[4] = occupancy_bp % 100u;
    s->values[5] = hit_bp / 100u;
    s->values[6] = hit_bp % 100u;
    s->values[7] = c->demand_hit_ok;
    s->values[8] = (uint32_t)demands;
    cacheTextStart(s,
        "[cache] total=%u.%02uMiB used=%uB occupancy=%u.%02u%% hitRate=%u.%02u%% hits=%u/%u\n", 9);
}

void cacheDiagStartLine(cacheTextStream* s, unsigned line, uint32_t q,
                        const cacheSdCounters* c)
{
    unsigned n = 0;
    const char* format = "";
#define V(x) (s->values[n++] = (x))
    V(q);
    switch (line) {
    case 0:
        format = "[l2] v=4 q=%u mode=M%u enabled=%u degraded=%u epoch=%u sdMiss=%u hitTry=%u hitOK=%u probeTry=%u probeOK=%u readB=%u writeB=%u seedUse=%u\n";
        V(c->mode); V(c->enabled); V(c->degraded); V(c->epoch); V(c->sd_miss);
        V(c->demand_hit_try); V(c->demand_hit_ok); V(c->probe_try); V(c->probe_ok);
        V(c->read_bytes); V(c->write_bytes); V(c->boot_seed_use);
        break;
    case 1:
        format = "[fill] v=4 q=%u state=%u snap=%u sec=%08X off=%u admit=%u commit=%u drop=%u defer=%u reason=%u quarantine=%u\n";
        V(c->fill_state); V(c->snapshot); V(c->fill_sector); V(c->fill_offset);
        V(c->fill_admit); V(c->fill_commit); V(c->fill_drop); V(c->fill_defer);
        V(c->fill_drop_reason); V(c->fill_quarantine);
        break;
    case 2:
        format = "[gate] v=4 q=%u measured=%u checks=%u blocked=%u mask=%02X stride=1024 quietFrag=%u maxFragUs=%u irqOverlap=%u e5Overlap=%u\n";
        V(c->gate_measured); V(c->gate_checks); V(c->gate_blocked); V(c->gate_mask);
        V(c->quiet_fill_fragments); V(c->quiet_fill_max_us);
        V(c->quiet_irq_overlap); V(c->quiet_e5_overlap);
        break;
    case 3:
        format = "[cache-err] v=4 q=%u sdObsolete=%u sdConflict=%u sdTransfer=%u crc=%u psRead=%u psWrite=%u\n";
        V(c->sd_token_obsolete); V(c->sd_token_live_conflict); V(c->sd_transfer_error);
        V(c->l2_crc_fail); V(c->psram_read_fail); V(c->psram_fill_fail);
        break;
#ifdef CACHE_PROBE_MISMATCH_DIAG
    case 4:
        format = "[probe-first] q=%u valid=%u sec=%08X off=%u sd=%02X ps=%02X sdCrc=%08X tagCrc=%08X\n";
        V(c->probe_first_valid); V(c->probe_first_sector); V(c->probe_first_offset);
        V(c->probe_first_expected); V(c->probe_first_actual);
        V(c->probe_first_ref_crc); V(c->probe_first_tag_crc);
        break;
#endif
#ifdef CACHE_SHADOW_STRIPES_4CHIP
    case 5:
        format = "[shadow] q=%u banks=%04X/%04X/%04X/%04X verify=%u/%u/%u/%u source=SD\n";
        for (unsigned i = 0; i < 4; i++) V(c->shadow_banks[i]);
        for (unsigned i = 0; i < 4; i++) V(c->shadow_verify[i]);
        break;
#endif
#ifdef CACHE_FULL_PAGE_4CHIP
    case 5:
        format = "[page] q=%u valid=%u fill=%u/%u/%u/%u probe=%u/%u/%u/%u hit=%u/%u/%u/%u maxSet=%u/%u/%u/%u mode=M%u sets=%u capB=%u\n";
        V(c->page_valid_sectors);
        for (unsigned i = 0; i < 4; i++) V(c->page_fill[i]);
        for (unsigned i = 0; i < 4; i++) V(c->page_probe[i]);
        for (unsigned i = 0; i < 4; i++) V(c->page_hit_ok[i]);
        for (unsigned i = 0; i < 4; i++) V(c->page_max_set[i]);
        V(c->mode);
        V(CACHE_PAGE_ACTIVE_SETS);
        V(CACHE_PAGE_ACTIVE_SETS * CACHE_PAGE_CHIPS * CACHE_PAGE_DATA_BYTES);
        break;
#endif
    }
#undef V
    cacheTextStart(s, format, n);
}
