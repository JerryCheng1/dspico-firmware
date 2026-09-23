#pragma once
#include "common.h"

// Design stage S1: a single read-only description of every PIO instruction
// word, state machine, DMA channel and GPIO the firmware owns. The plan never
// claims a peripheral at runtime; it only makes ownership explicit so a future
// stage (PSRAM engine, cache backend) is forced to declare a conflict at
// compile time instead of silently stealing DMA0 / SDIO / PIO0 SM0.

typedef struct
{
    u16 pio0Words;
    u16 pio1Words;
    u16 pio2Words;
    u32 dmaMask;        // bit N set => DMA channel N owned
    u32 gpioMask;       // Bank-0 GPIOs owned by the cartridge/SDIO/UART
    u16 initLogBytes;   // SRAM held by the bounded init-log ring
    bool traceEnabled;
    bool conflict;      // runtime-visible result of the static checks
    const char* buildTag;
    u8 stage;           // CACHE_STAGE compiled in
} cacheResourcePlan;

#ifdef __cplusplus
extern "C" {
#endif

// Fills the plan for the current build. Pure computation, no side effects.
void cacheResourcePlanInit(void);

const cacheResourcePlan* cacheResourcePlanGet(void);

#ifdef __cplusplus
}
#endif
