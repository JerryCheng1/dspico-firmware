#pragma once

#include "pico/stdlib.h"
#include "pico/time.h"
#ifdef ENABLE_UART_LOG
#include "uartLog.h"
#endif

// Cartridge-runtime diagnostics (ENABLE_CART_TRACE, see CMakeLists.txt): PIO
// IRQ rx trace, E4 FIFO ring, E3/E4/E5 counters + SD event ring, GPIO0
// logic-analyzer trigger, running heartbeat, auto dumps, TIMER0_IRQ_1 stuck
// detector and the deferred log ring. Strictly separate from ENABLE_UART_LOG:
// a plain UART-log build must not add a single instruction to the hard
// real-time cartridge path (PIO0 IRQ, GPIO IRQ, DMA IRQ, timer IRQ, E3/E4/E5
// handlers).
#if defined(ENABLE_UART_LOG) && defined(ENABLE_CART_TRACE)
// Deferred diagnostic log: LOG formats into an in-RAM ring immediately but the
// UART drain happens ONLY while the cartridge bus has been quiet (no CEB fall
// for >1 ms). Blocking prints in the main loop stretch the E3->ready=1 answer
// past the DLDI driver's per-read deadline (115200 baud is ~87 us/char; four
// [sdio] lines per read = ~10 ms vs a ~8 ms budget - r66's intermittent mount
// failures). Deferring keeps full diagnostics at zero bus-latency cost. When
// the main loop wedges, the priority-0 wake probe force-flushes the ring, so
// the last line still shows the last step that actually ran (the r63 property
// the blocking LOG was introduced for).
void dbgDeferLog(const char* fmt, ...);
// TRACE_QUIET_LOG (diag builds only): keep every tracer (rx/e4fl rings,
// counters, quiet dump, fault dump) but silence the per-event LOG stream.
// The full diag build formats each event with vsnprintf on the main loop
// (~10 us per line, on the block-completion path); that slowdown is the last
// remaining difference between the 20/20 diag build and the still-flaky
// nodebug build. This variant keeps the main loop at nodebug speed so a
// nodebug-only failure leaves its rings behind for the quiet dump.
#ifdef TRACE_QUIET_LOG
#define LOG(...)    ((void)0)
#else
#define LOG(...)    dbgDeferLog(__VA_ARGS__)
#endif
#else
#define LOG(...)    ((void)0)
#endif

typedef uint8_t u8;
typedef int8_t s8;
typedef uint16_t u16;
typedef int16_t s16;
typedef uint32_t u32;
typedef int32_t s32;
typedef uint64_t u64;
typedef int64_t s64;

typedef volatile uint8_t vu8;
typedef volatile int8_t vs8;
typedef volatile uint16_t vu16;
typedef volatile int16_t vs16;
typedef volatile uint32_t vu32;
typedef volatile int32_t vs32;
typedef volatile uint64_t vu64;
typedef volatile int64_t vs64;

// SD-side event ring: one SRAM word per event, no formatting, safe from the
// cart IRQ. Survives TRACE_QUIET_LOG so a sleeping-main-loop build still
// shows where the read pipeline stopped (traceq failing boot: 107 busy E4
// polls, zero SD visibility). Printed by cartTraceDump on the failure dumps.
// Codes: 1=RD_BEGIN(sector) 2=RD_RXCONT(left) 3=RD_CMD18_OK(tries)
//        4=RD_CMD18_FAIL(tries) 5=RD_BLOCK_OK(done) 6=RD_CRC_FAIL
//        7=RD_TIMEOUT 8=RD_ALLDONE(us) 9=E3_REQ(sector) 10=E4_READY(us)
//        11=E4_READY_CACHED(us) 12=KICK(cmd0hi<<8|relPc) 13=E5_FETCH(word)
#if defined(ENABLE_UART_LOG) && defined(ENABLE_CART_TRACE)
#ifdef __cplusplus
extern "C" {
#endif
extern volatile u32 gSdEvtIdx;
extern u32 gSdEvtRing[64];
#ifdef __cplusplus
}
#endif
// Atomic slot claim: writers are the cart IRQ (0x00), gpioIrq (0x80) and the
// main loop - a plain idx++ races across all three and scrambles exactly the
// same-microsecond failure clusters the ring exists to capture.
#define SD_EVT(code, arg) \
    (gSdEvtRing[__atomic_fetch_add(&gSdEvtIdx, 1u, __ATOMIC_RELAXED) & 63u] = \
        (((u32)(code) << 28) | ((u32)(arg) & 0x0FFFFFFFu)))
#else
#define SD_EVT(code, arg) ((void)0)
#endif

#define SD_USE_SDIO

#define SDIO_CLK 3
#define SDIO_CMD 4
#define SDIO_D0  5
#define SDIO_D1  6
#define SDIO_D2  7
#define SDIO_D3  8
#define SDIO_PIN_MASK  ((1u << SDIO_D3) | (1u << SDIO_D2) | (1u << SDIO_D1) | (1u << SDIO_D0) | \
                        (1u << SDIO_CMD) | (1u << SDIO_CLK))

#define PIN_RST     9
#define PIN_CEB     10
#define PIN_WREB    11 //clk
#define PIN_D0      12
#define PIN_D1      13
#define PIN_D2      14
#define PIN_D3      15
#define PIN_D4      16
#define PIN_D5      17
#define PIN_D6      18
#define PIN_D7      19
#define PIN_IRQ     20
#define PIN_CS2     21
#define NTRC_PIN_MASK  ((1u << PIN_CS2) | (1u << PIN_IRQ) | (1u << PIN_D7) | (1u << PIN_D6) | \
                        (1u << PIN_D5) | (1u << PIN_D4) | (1u << PIN_D3) | (1u << PIN_D2) | \
                        (1u << PIN_D1) | (1u << PIN_D0) | (1u << PIN_WREB) | (1u << PIN_CEB) | \
                        (1u << PIN_RST))

#define PIN_USB_VBUS    24

#define PIN_DEV_TX0     0
#define PIN_DEV_RX0     1
#define DEV_UART_PIN_MASK  ((1u << PIN_DEV_RX0) | (1u << PIN_DEV_TX0))

#define PIN_INPUT_MASK  0x2FFE00

#define CARD_ID_NTR     0x800000C2
#define CARD_ID_TWL     0xC00000C2

typedef void (*sd_callback_t)(uint32_t bytes_complete);

static inline uint millis(void)
{
    return us_to_ms(time_us_64());
}

#ifdef __cplusplus
#include "sd/SdCard.h"
extern SdCard gSdCard;
#endif
