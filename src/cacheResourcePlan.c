#include "cacheResourcePlan.h"
#include "cacheInitLog.h"

#include "ntrCard.pio.h"
#include "rp2350_sdio.pio.h"
#if NTRC_TRACE_ENABLED
#include "ntrCardTrace.pio.h"
#endif
#if CACHE_STAGE >= 3
#include "psram.pio.h"
#endif

#define PLAN_COUNT_OF(a) (sizeof(a) / sizeof((a)[0]))

// SDIO already occupies PIO1's 32 instruction words (design evidence H1: the
// 4 SMs of a PIO share one 32-word store). PSRAM therefore must live on PIO2.
_Static_assert(PLAN_COUNT_OF(sdio_cmd_clk_program_instructions) +
                       PLAN_COUNT_OF(sdio_data_rx_program_instructions) +
                       PLAN_COUNT_OF(sdio_data_tx_program_instructions) ==
                   32u,
               "SDIO must own exactly 32 PIO1 instruction words");

#if NTRC_TRACE_ENABLED && defined(DSPICO_ENABLE_WRFUXXED)
_Static_assert(0, "trace (8) + WRFUXXED (8) + cartridge (24) exceed PIO0's "
                  "32 instruction words; disable one (design section 11.3)");
#endif

#if CACHE_STAGE >= 3
_Static_assert(PLAN_COUNT_OF(psram_quad_program_instructions) <= 32u,
               "PSRAM engine must fit PIO2's 32 instruction words");
#endif

#ifndef UART_LOG_BUILD_TAG
#define UART_LOG_BUILD_TAG "stage1"
#endif

static cacheResourcePlan sPlan;

const cacheResourcePlan* cacheResourcePlanGet(void)
{
    return &sPlan;
}

void cacheResourcePlanInit(void)
{
    u16 words0 = (u16)PLAN_COUNT_OF(ntr_card_program_instructions);
#if NTRC_TRACE_ENABLED
    words0 += (u16)PLAN_COUNT_OF(ntr_card_trace_program_instructions);
#endif
#ifdef DSPICO_ENABLE_WRFUXXED
    words0 += (u16)PLAN_COUNT_OF(ntr_card_spi_program_instructions);
#endif

    sPlan.pio0Words = words0;
    sPlan.pio1Words = 32;
#if CACHE_STAGE >= 3
    // PSRAM transaction engine (S2/S3): PIO2 SM0, measured at 27 words.
    sPlan.pio2Words = (u16)PLAN_COUNT_OF(psram_quad_program_instructions);
#else
    // PIO2 is still reserved for the PSRAM engine but not built yet.
    sPlan.pio2Words = 0;
#endif

    // DMA0 = cartridge, DMA2/3 = SDIO, DMA4/5 reserved for PSRAM TX/RX.
    sPlan.dmaMask = (1u << 0) | (1u << 2) | (1u << 3) | (1u << 4) | (1u << 5);
#if NTRC_TRACE_ENABLED
    // The trace side channel must claim a registered channel, never rely on
    // the incidental dma_claim_unused_channel() order.
    sPlan.dmaMask |= (1u << 1);
#endif

    sPlan.gpioMask = NTRC_PIN_MASK | SDIO_PIN_MASK | DEV_UART_PIN_MASK;
    sPlan.initLogBytes = CACHE_INIT_LOG_ENABLED ? (u16)CACHE_LOG_RING_BYTES : 0;
    sPlan.traceEnabled = (NTRC_TRACE_ENABLED != 0);
    sPlan.conflict = false;
    sPlan.buildTag = UART_LOG_BUILD_TAG;
    sPlan.stage = CACHE_STAGE;
}
