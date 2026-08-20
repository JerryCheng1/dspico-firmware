#include "common.h"
#include "hardware/structs/scb.h"

_Static_assert(PICO_RP2350, "DSpico power policy requires RP2350 silicon");

static void keepRp2350WakeClocksAvailable(void)
{
    // The RP2354A port intentionally keeps the normal RP2350 clock policy
    // until deep-sleep wake behavior is qualified on the production board.
    // Ordinary WFI still idles the active core without hiding PIO/DMA events.
    scb_hw->scr &= ~ARM_CPU_PREFIXED(SCR_SLEEPDEEP_BITS);
}

void pwr_initPowerSaving(void)
{
    keepRp2350WakeClocksAvailable();
}

void pwr_enableAfterBootPowerSaving(void)
{
    keepRp2350WakeClocksAvailable();
}

void pwr_disableAfterBootPowerSaving(void)
{
    keepRp2350WakeClocksAvailable();
}
