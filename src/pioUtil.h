#pragma once

#include "hardware/pio.h"

// pico-sdk 2.3 implements pio_sm_set_enabled() as a read-modify-write of the
// shared PIO CTRL register. DSpico may change different state machines from a
// foreground path and an IRQ (or from separate cores), so a concurrent update
// can otherwise restore stale bits. RP2350 atomic aliases change only the
// requested SM enable bit.
static inline void dspicoPioSmSetEnabled(PIO pio, uint sm, bool enabled)
{
    uint32_t mask = 1u << (PIO_CTRL_SM_ENABLE_LSB + sm);
    if (enabled)
        hw_set_bits(&pio->ctrl, mask);
    else
        hw_clear_bits(&pio->ctrl, mask);
}

// pio_sm_init() begins with the same non-atomic enable-bit RMW. This local
// equivalent is used when a new SM is configured while another SM in the same
// block is live, so its enable state cannot be lost in that RMW window.
static inline void dspicoPioSmInit(PIO pio, uint sm, uint initialPc,
                                   const pio_sm_config* config)
{
    dspicoPioSmSetEnabled(pio, sm, false);
    pio_sm_set_config(pio, sm, config);
    pio_sm_clear_fifos(pio, sm);

    const uint32_t fdebugMask =
        (1u << PIO_FDEBUG_TXOVER_LSB) |
        (1u << PIO_FDEBUG_RXUNDER_LSB) |
        (1u << PIO_FDEBUG_TXSTALL_LSB) |
        (1u << PIO_FDEBUG_RXSTALL_LSB);
    pio->fdebug = fdebugMask << sm;

    pio_sm_restart(pio, sm);
    pio_sm_clkdiv_restart(pio, sm);
    pio_sm_exec(pio, sm, pio_encode_jmp(initialPc));
}
