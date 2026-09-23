#include "cacheDiagLog.h"

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

void cacheDiagStartLine(cacheTextStream* s, unsigned line, uint32_t q,
                        const cacheSdCounters* c)
{
    unsigned n = 0;
    const char* format = "";
#define V(x) (s->values[n++] = (x))
    V(q);
    switch (line) {
    case 0:
        format = "[l2] v=4 q=%u mode=M%u enabled=%u degraded=%u epoch=%u sdMiss=%u hitTry=%u hitOK=%u probeTry=%u probeOK=%u readB=%u writeB=%u\n";
        V(c->mode); V(c->enabled); V(c->degraded); V(c->epoch); V(c->sd_miss);
        V(c->demand_hit_try); V(c->demand_hit_ok); V(c->probe_try); V(c->probe_ok);
        V(c->read_bytes); V(c->write_bytes);
        break;
    case 1:
        format = "[fill] v=4 q=%u state=%u snap=%u sec=%08X off=%u admit=%u commit=%u drop=%u defer=%u reason=%u quarantine=%u\n";
        V(c->fill_state); V(c->snapshot); V(c->fill_sector); V(c->fill_offset);
        V(c->fill_admit); V(c->fill_commit); V(c->fill_drop); V(c->fill_defer);
        V(c->fill_drop_reason); V(c->fill_quarantine);
        break;
    case 2:
        format = "[gate] v=4 q=%u measured=%u checks=%u blocked=%u mask=%02X stride=1024\n";
        V(c->gate_measured); V(c->gate_checks); V(c->gate_blocked); V(c->gate_mask);
        break;
    case 3:
        format = "[cache-err] v=4 q=%u sdObsolete=%u sdConflict=%u sdTransfer=%u crc=%u psRead=%u psWrite=%u\n";
        V(c->sd_token_obsolete); V(c->sd_token_live_conflict); V(c->sd_transfer_error);
        V(c->l2_crc_fail); V(c->psram_read_fail); V(c->psram_fill_fail);
        break;
    }
#undef V
    cacheTextStart(s, format, n);
}
