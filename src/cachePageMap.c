#include "cachePageMap.h"
#include <string.h>

_Static_assert(sizeof(cachePageEntry) == 20u, "page directory entry must be 20 B");
_Static_assert(sizeof(cachePageMap) <= 192u * 1024u,
               "page directory exceeds SRAM budget");
_Static_assert(CACHE_PAGE_RESERVED_BYTES == 4640u, "qualification tail changed");
_Static_assert(CACHE_PAGE_COUNT < 65536u, "lease slot must fit in uint16_t");
_Static_assert(CACHE_PAGE_ACTIVE_SETS > 0u &&
               CACHE_PAGE_ACTIVE_SETS <= CACHE_PAGE_SETS,
               "active page sets must fit qualified PSRAM capacity");

static inline uint32_t setOf(uint32_t tag) { return tag % CACHE_PAGE_ACTIVE_SETS; }
static inline uint8_t validOf(const cachePageEntry* e) { return (uint8_t)e->state; }
static inline uint8_t busyOf(const cachePageEntry* e) { return (uint8_t)(e->state >> 8); }
static inline bool pinnedOf(const cachePageEntry* e) { return (e->state & (1u << 16)) != 0; }
static inline bool referencedOf(const cachePageEntry* e) { return (e->state & (1u << 17)) != 0; }
static inline uint8_t handOf(const cachePageMap* m, uint32_t set)
{
    return (m->hand[set >> 2] >> ((set & 3u) * 2u)) & 3u;
}
static inline void setHand(cachePageMap* m, uint32_t set, uint8_t way)
{
    uint8_t shift = (uint8_t)((set & 3u) * 2u);
    uint8_t mask = (uint8_t)(3u << shift);
    m->hand[set >> 2] = (uint8_t)((m->hand[set >> 2] & ~mask) |
                                  (way << shift));
}
static inline uint32_t nextVersion(uint32_t v) { return v == UINT32_MAX ? 1u : v + 1u; }
static inline uint32_t count8(uint32_t bits)
{
    bits &= 0xffu;
    bits = bits - ((bits >> 1) & 0x55u);
    bits = (bits & 0x33u) + ((bits >> 2) & 0x33u);
    return (bits + (bits >> 4)) & 0x0fu;
}

void cachePageMapInit(cachePageMap* m, uint8_t usable_mask)
{
    if (!m) return;
    memset(m, 0, sizeof(*m));
    m->usable_mask = usable_mask & 0x0fu;
}

static void makeLease(const cachePageEntry* e, uint16_t slot, uint8_t lane,
                      uint8_t had_page, uint8_t is_pin, cachePageLease* out)
{
    out->slot = slot;
    out->lane = lane;
    out->had_page = had_page;
    out->is_pin = is_pin;
    out->version = e->version;
    out->tag = e->tag;
    out->epoch = e->epoch;
}

bool cachePageMapLookup(cachePageMap* m, uint32_t sector, uint32_t epoch,
                        cachePageLease* out, uint32_t* table_crc)
{
    if (!m || !out || !table_crc) return false;
    uint32_t tag = sector >> 3;
    uint8_t lane = (uint8_t)(sector & 7u);
    uint32_t first = setOf(tag) * CACHE_PAGE_CHIPS;
    for (uint32_t way = 0; way < CACHE_PAGE_CHIPS; way++) {
        cachePageEntry* e = &m->entry[first + way];
        if (!(m->usable_mask & (1u << way))) continue;
        if (e->tag != tag || e->epoch != epoch || busyOf(e) ||
            !(validOf(e) & (1u << lane)) || pinnedOf(e)) continue;
        e->state |= 1u << 16;
        e->state |= 1u << 17;
        makeLease(e, (uint16_t)(first + way), lane, 1u, 1u, out);
        *table_crc = e->table_crc;
        return true;
    }
    return false;
}

bool cachePageMapUnpin(cachePageMap* m, const cachePageLease* lease)
{
    if (!cachePageMapLeaseCurrent(m, lease) || !lease->is_pin) return false;
    cachePageEntry* e = &m->entry[lease->slot];
    if (!pinnedOf(e)) return false;
    e->state &= ~(1u << 16);
    return true;
}

bool cachePageMapBegin(cachePageMap* m, uint32_t sector, uint32_t epoch,
                       cachePageLease* out)
{
    if (!m || !out) return false;
    if (m->count_epoch != epoch) {
        m->count_epoch = epoch;
        m->valid_count = 0;
    }
    uint32_t tag = sector >> 3;
    uint8_t lane = (uint8_t)(sector & 7u);
    uint32_t set = setOf(tag);
    uint32_t first = set * CACHE_PAGE_CHIPS;
    cachePageEntry* chosen = 0;
    uint32_t chosenWay = 0;

    // Preserve a matching page. If that page is busy, the caller must defer
    // instead of allocating a duplicate tag in another way.
    for (uint32_t way = 0; way < CACHE_PAGE_CHIPS; way++) {
        cachePageEntry* e = &m->entry[first + way];
        if (!(m->usable_mask & (1u << way))) continue;
        if (e->tag == tag && e->epoch == epoch && e->version) {
            if (busyOf(e) || pinnedOf(e)) return false;
            chosen = e;
            chosenWay = way;
            break;
        }
    }
    if (!chosen) {
        // Prefer an empty/stale page. No busy or pinned page is evicted.
        // Stripe cold logical pages across all four chips. The physical way
        // remains fixed; this only changes the search order within a set.
        uint32_t preferred = tag & (CACHE_PAGE_CHIPS - 1u);
        for (uint32_t n = 0; n < CACHE_PAGE_CHIPS; n++) {
            uint32_t way = (preferred + n) & (CACHE_PAGE_CHIPS - 1u);
            cachePageEntry* e = &m->entry[first + way];
            if (!(m->usable_mask & (1u << way)) || busyOf(e) || pinnedOf(e)) continue;
            if (!e->version || e->epoch != epoch || !validOf(e)) {
                chosen = e;
                chosenWay = way;
                break;
            }
        }
        if (!chosen) {
            // One bounded CLOCK pass. Referenced ways lose their bit and are
            // considered on a later step; there is never a second scan here.
            uint8_t hand = handOf(m, set);
            for (uint32_t n = 0; n < CACHE_PAGE_CHIPS; n++) {
                uint32_t way = (hand + n) & (CACHE_PAGE_CHIPS - 1u);
                cachePageEntry* e = &m->entry[first + way];
                if (!(m->usable_mask & (1u << way)) || busyOf(e) || pinnedOf(e))
                    continue;
                if (referencedOf(e)) {
                    e->state &= ~(1u << 17);
                    continue;
                }
                chosen = e;
                chosenWay = way;
                break;
            }
            if (chosen) setHand(m, set, (uint8_t)((chosenWay + 1u) & 3u));
        }
        if (!chosen) return false;
    }

    uint8_t had_page = chosen->tag == tag && chosen->epoch == epoch &&
                       validOf(chosen) != 0;
    if (chosen->epoch == epoch)
        m->valid_count -= count8(chosen->state &
                                  (had_page ? (1u << lane) : 0xffu));
    if (!had_page) {
        chosen->tag = tag;
        chosen->epoch = epoch;
        chosen->table_crc = 0;
        chosen->state = 0;
    }
    chosen->version = nextVersion(chosen->version);
    chosen->state &= ~(1u << lane);
    chosen->state |= 1u << (lane + 8u);
    chosen->state &= ~(1u << 17);
    makeLease(chosen, (uint16_t)(first + chosenWay), lane, had_page, 0u, out);
    return true;
}

bool cachePageMapLeaseCurrent(const cachePageMap* m, const cachePageLease* lease)
{
    if (!m || !lease || lease->slot >= CACHE_PAGE_COUNT || lease->lane >= 8u)
        return false;
    const cachePageEntry* e = &m->entry[lease->slot];
    return e->version == lease->version && e->tag == lease->tag &&
           e->epoch == lease->epoch;
}

bool cachePageMapCommit(cachePageMap* m, const cachePageLease* lease,
                        uint32_t table_crc)
{
    if (!cachePageMapLeaseCurrent(m, lease) || lease->is_pin) return false;
    cachePageEntry* e = &m->entry[lease->slot];
    uint32_t busy = 1u << (lease->lane + 8u);
    if (!(e->state & busy)) return false;
    e->table_crc = table_crc;
    e->state = (e->state & ~busy) | (1u << lease->lane);
    if (e->epoch == m->count_epoch) m->valid_count++;
    e->state |= 1u << 17;
    return true;
}

bool cachePageMapAbort(cachePageMap* m, const cachePageLease* lease)
{
    if (!cachePageMapLeaseCurrent(m, lease) || lease->is_pin) return false;
    cachePageEntry* e = &m->entry[lease->slot];
    if (!(e->state & (1u << (lease->lane + 8u)))) return false;
    if (e->epoch == m->count_epoch) m->valid_count -= count8(e->state);
    e->state = 0;
    e->table_crc = 0;
    e->version = nextVersion(e->version);
    return true;
}

bool cachePageMapInvalidate(cachePageMap* m, const cachePageLease* lease)
{
    if (!cachePageMapLeaseCurrent(m, lease)) return false;
    cachePageEntry* e = &m->entry[lease->slot];
    if (busyOf(e) || pinnedOf(e)) return false;
    if (e->epoch == m->count_epoch) m->valid_count -= count8(e->state);
    e->state = 0;
    e->table_crc = 0;
    e->version = nextVersion(e->version);
    return true;
}

uint8_t cachePageMapChip(uint16_t slot) { return (uint8_t)(slot % CACHE_PAGE_CHIPS); }
uint32_t cachePageMapDataAddr(uint16_t slot, uint8_t lane)
{
    return ((uint32_t)slot / CACHE_PAGE_CHIPS) * CACHE_PAGE_STRIDE_BYTES +
           (uint32_t)lane * 512u;
}
uint32_t cachePageMapTableAddr(uint16_t slot)
{
    return ((uint32_t)slot / CACHE_PAGE_CHIPS) * CACHE_PAGE_STRIDE_BYTES +
           CACHE_PAGE_DATA_BYTES;
}
