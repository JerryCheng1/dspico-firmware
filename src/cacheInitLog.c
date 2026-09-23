#include "cacheInitLog.h"

#if CACHE_INIT_LOG_ENABLED
#include <stdio.h>
#include <string.h>
#include "hardware/uart.h"
#include "hardware/clocks.h"
#include "hardware/sync.h"
#include "cacheResourcePlan.h"
#if CACHE_STAGE >= 3
#include "cacheSd.h"
#endif

#ifndef CACHE_BUILD_GIT_REV
#define CACHE_BUILD_GIT_REV "unknown"
#endif
#ifndef UART_LOG_BUILD_TAG
#define UART_LOG_BUILD_TAG "cache-send-r5"
#endif

static cache_init_log_ring_t sRing;
static u32 sFirstDropEvent;

static char sLine[256];
static u16 sLineLen;
static u16 sLinePos;
static bool sLineActive;
static u64 sLastStatsUs;

static const char* resultName(u8 result)
{
    switch (result)
    {
        case 0: return "OK";
        case 1: return "DEGRADED";
        case 2: return "ERROR";
        case 3: return "CACHE_OFF";
        default: return "?";
    }
}

static int formatEvent(const cache_init_log_event_t* ev, char* out, int cap)
{
    switch (ev->event)
    {
        case CACHE_LOG_EV_FIFO:
            return snprintf(out, (size_t)cap, "[cache] fifo rejected=%lu pending=%lu\n",
                            (unsigned long)ev->arg[0], (unsigned long)ev->arg[1]);
        case CACHE_LOG_EV_BOOT:
            return snprintf(out, (size_t)cap,
                            "[boot] build=%s stage=%u sys_khz=%lu init_log=1 rev=%s\n",
                            UART_LOG_BUILD_TAG, (unsigned)ev->step,
                            (unsigned long)ev->arg[0], CACHE_BUILD_GIT_REV);
        case CACHE_LOG_EV_RESOURCE:
        {
            const cacheResourcePlan* p = cacheResourcePlanGet();
            return snprintf(out, (size_t)cap,
                            "[cache] resource pio=%u/%u/%u dma=%04lX gpio=%08lX log=%u stage=%u\n",
                            (unsigned)p->pio0Words, (unsigned)p->pio1Words,
                            (unsigned)p->pio2Words, (unsigned long)p->dmaMask,
                            (unsigned long)p->gpioMask, (unsigned)p->initLogBytes,
                            (unsigned)p->stage);
        }
        case CACHE_LOG_EV_INIT_SUMMARY:
            return snprintf(out, (size_t)cap,
                            "[cache] init local_result=%s stage=%u drop=%lu first=%lu\n",
                            resultName(ev->result), (unsigned)ev->step,
                            (unsigned long)sRing.dropped,
                            (unsigned long)sFirstDropEvent);
        case CACHE_LOG_EV_STATS:
            return snprintf(out, (size_t)cap,
                            "[cache] stats pushed=%lu drop=%lu depth=%lu\n",
                            (unsigned long)sRing.totalPushed, (unsigned long)sRing.dropped,
                            (unsigned long)((sRing.head - sRing.tail) % CACHE_INIT_LOG_EVENTS));
        case CACHE_LOG_EV_SD_STATS:
            return snprintf(out, (size_t)cap,
                            "[cache] sd hits=%lu miss=%lu pf=%lu fill=%lu err=%lu sec=%lu\n",
                            (unsigned long)ev->arg[0], (unsigned long)ev->arg[1],
                            (unsigned long)ev->arg[2], (unsigned long)ev->arg[3],
                            (unsigned long)ev->arg[4], (unsigned long)ev->arg[5]);
        case CACHE_LOG_EV_L2_STATE:
            return snprintf(out, (size_t)cap,
                            "[cache] l2 mode=M%u enabled=%lu state=%lu mask=%02lX result=%s\n",
                            (unsigned)ev->step, (unsigned long)ev->arg[0],
                            (unsigned long)ev->elapsedUs, (unsigned long)ev->arg[1], resultName(ev->result));
        case CACHE_LOG_EV_SD_STATE:
            return snprintf(out, (size_t)cap,
                            "[cache] req sec=%08lX flags=%02lX sd=%lu proto=%lu retry=%lu fault=%lu\n",
                            (unsigned long)ev->arg[0], (unsigned long)ev->arg[1],
                            (unsigned long)ev->arg[2], (unsigned long)ev->arg[3],
                            (unsigned long)ev->arg[4], (unsigned long)ev->arg[5]);
        case CACHE_LOG_EV_PSRAM:
            return snprintf(out, (size_t)cap,
                            "[psram] chip=%u step=%u result=%u t=%lu a=%08lX b=%08lX c=%08lX d=%08lX\n",
                            (unsigned)ev->chip, (unsigned)ev->step, (unsigned)ev->result,
                            (unsigned long)ev->elapsedUs, (unsigned long)ev->arg[0],
                            (unsigned long)ev->arg[1], (unsigned long)ev->arg[2],
                            (unsigned long)ev->arg[3]);
        case CACHE_LOG_EV_SD_COUNTS:
            return snprintf(out, (size_t)cap,
                            "[cache] sd tokenObsolete=%lu tokenLive=%lu xferErr=%lu hitTry=%lu hitOk=%lu crc=%lu\n",
                            (unsigned long)ev->arg[0], (unsigned long)ev->arg[1],
                            (unsigned long)ev->arg[2], (unsigned long)ev->arg[3],
                            (unsigned long)ev->arg[4], (unsigned long)ev->arg[5]);
        case CACHE_LOG_EV_FILL_COUNTS:
            return snprintf(out, (size_t)cap,
                            "[cache] fill psramR=%lu psramW=%lu admit=%lu drop=%lu defer=%lu commit=%lu\n",
                            (unsigned long)ev->arg[0], (unsigned long)ev->arg[1],
                            (unsigned long)ev->arg[2], (unsigned long)ev->arg[3],
                            (unsigned long)ev->arg[4], (unsigned long)ev->arg[5]);
        case CACHE_LOG_EV_HANDOVER:
            return snprintf(out, (size_t)cap,
                            "[cache] e4 busy=%lu ready=%lu ackFail=%lu e5 ok=%lu noAck=%lu idMis=%lu slotMis=%lu\n",
                            (unsigned long)ev->arg[0], (unsigned long)ev->arg[1],
                            (unsigned long)ev->arg[2], (unsigned long)ev->arg[3],
                            (unsigned long)ev->arg[4], (unsigned long)ev->arg[5],
                            (unsigned long)ev->elapsedUs);
        case CACHE_LOG_EV_FAULT_FIRST:
            return snprintf(out, (size_t)cap,
                            "[cache] first_fault kind=%u detail=%08lX offer=%08lX ack=%08lX sampled=%lu seq=%lu\n",
                            (unsigned)ev->step, (unsigned long)ev->arg[0],
                            (unsigned long)ev->arg[1], (unsigned long)ev->arg[2],
                            (unsigned long)ev->arg[3], (unsigned long)ev->arg[4]);
        case CACHE_LOG_EV_CACHE_HEALTH:
        {
            const char* fill = "?";
#if CACHE_STAGE >= 3
            fill = cacheSdFillStateName();
#endif
            return snprintf(out, (size_t)cap,
                            "[cache] health degraded=%lu respState=%lu fill=%s dropReason=%lu quarantine=%lu lastInv=%lu\n",
                            (unsigned long)ev->arg[0], (unsigned long)ev->arg[1], fill,
                            (unsigned long)ev->arg[2], (unsigned long)ev->arg[3],
                            (unsigned long)ev->arg[4]);
        }
        case CACHE_LOG_EV_FAULT:
            return snprintf(out, (size_t)cap,
                            "[cache] fault step=%u chip=%u reason=%u a=%08lX b=%08lX c=%08lX\n",
                            (unsigned)ev->step, (unsigned)ev->chip, (unsigned)ev->result,
                            (unsigned long)ev->arg[0], (unsigned long)ev->arg[1],
                            (unsigned long)ev->arg[2]);
        default:
            return snprintf(out, (size_t)cap,
                            "[cache] ev=%u step=%u chip=%u result=%u t=%lu a=%08lX b=%08lX\n",
                            (unsigned)ev->event, (unsigned)ev->step, (unsigned)ev->chip,
                            (unsigned)ev->result, (unsigned long)ev->elapsedUs,
                            (unsigned long)ev->arg[0], (unsigned long)ev->arg[1]);
    }
}

#endif // CACHE_INIT_LOG_ENABLED

void cacheInitLogInit(void)
{
#if CACHE_INIT_LOG_ENABLED
    memset(&sRing, 0, sizeof(sRing));
    sFirstDropEvent = 0;
    sLineLen = 0;
    sLinePos = 0;
    sLineActive = false;
    sLastStatsUs = 0;
#endif
}

bool cacheInitLogEmit(u8 eventId, u8 step, u8 chip, u8 result, u32 elapsedUs,
                      u32 a, u32 b, u32 c, u32 d, u32 e, u32 f)
{
#if CACHE_INIT_LOG_ENABLED
    cache_init_log_event_t ev;
    ev.event = eventId;
    ev.step = step;
    ev.chip = chip;
    ev.result = result;
    ev.elapsedUs = elapsedUs;
    ev.arg[0] = a;
    ev.arg[1] = b;
    ev.arg[2] = c;
    ev.arg[3] = d;
    ev.arg[4] = e;
    ev.arg[5] = f;

    __dmb();
    bool ok = cacheInitLogRingPush(&sRing, &ev);
    if (!ok && !sFirstDropEvent)
        sFirstDropEvent = eventId;
    return ok;
#else
    (void)eventId; (void)step; (void)chip; (void)result; (void)elapsedUs;
    (void)a; (void)b; (void)c; (void)d; (void)e; (void)f;
    return false;
#endif
}

void cacheInitLogPoll(void)
{
#if CACHE_INIT_LOG_ENABLED
    // Only start a new line when the cartridge bus is idle. A line already in
    // progress is also paused the moment the bus becomes active; no byte ever
    // delays a command response.
    if (!sLineActive)
    {
        cache_init_log_event_t ev;
        if (!cacheInitLogRingPop(&sRing, &ev))
            return;

        int n = formatEvent(&ev, sLine, (int)sizeof(sLine));
        if (n <= 0)
            return;
        if ((size_t)n >= sizeof(sLine))
            n = (int)sizeof(sLine) - 1;
        sLineLen = (u16)n;
        sLinePos = 0;
        sLineActive = true;
    }

    u32 budget = CACHE_LOG_UART_BYTES_PER_POLL;
    while (budget && sLinePos < sLineLen)
    {
        // CEB and CS2 high means the host is not clocking the cartridge.
        if (!gpio_get(PIN_CEB) || !gpio_get(PIN_CS2))
            break;
        if (!uart_is_writable(uart1))
            break;
        uart_putc_raw(uart1, sLine[sLinePos++]);
        budget--;
    }
    if (sLinePos >= sLineLen)
        sLineActive = false;
#endif
}

void cacheInitLogStatsStep(void)
{
#if CACHE_INIT_LOG_ENABLED && defined(CACHE_STATS_LOG)
    u64 now = time_us_64();
    if (sLastStatsUs != 0 && (now - sLastStatsUs) < 5000000ull)
        return;
    sLastStatsUs = now;
    cacheInitLogEmit(CACHE_LOG_EV_STATS, 0, 0, 0, (u32)now, 0, 0, 0, 0, 0, 0);
#endif
}

u32 cacheInitLogDropped(void)
{
#if CACHE_INIT_LOG_ENABLED
    return sRing.dropped;
#else
    return 0;
#endif
}

u32 cacheInitLogPushed(void)
{
#if CACHE_INIT_LOG_ENABLED
    return sRing.totalPushed;
#else
    return 0;
#endif
}
