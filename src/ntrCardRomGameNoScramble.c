#include "common.h"
#include <stdio.h>
#include "hardware/pio.h"
#include "ntrCardRom.h"
#include "powerSaving.h"
#include "ntrCardRomGameNoScramble.h"

#ifdef ENABLE_UART_LOG
extern volatile u32 gCartSdDummyCmd0;
extern volatile u32 gCartSdUnknownCmd1;
extern volatile u32 gCartSdUnknownWord;
extern volatile u32 gCartSdUnknownCmd0;
#endif

// Functional (not just diagnostic): set while an E4 poll pre-armed length
// word sits in the TX FIFO. pio_sm_clear_fifos below drops that word, so the
// flag must go with it.
extern volatile u32 gCartSdE4LenArmed;

void ntrc_gameNoScrambleCmd1Unknown(ntr_rom_emu_t* romEmu, u32 word, pio_hw_t* pio)
{
#ifdef ENABLE_UART_LOG
    gCartSdUnknownCmd1++;
    // Raw words for the deferred [cart] line: real command caught in a
    // desync vs. pure garbage distinguishes decoder misalignment from a
    // console-side protocol we do not implement.
    gCartSdUnknownWord = word;
    gCartSdUnknownCmd0 = romEmu->cmd0;
#endif
    // Upstream disabled PIO0_IRQ_0 here ("do not receive further commands
    // until card reset") and blocking-printed - a transient desync (a lost
    // transaction leaving a partial RX stream, r66 boot 3: D=1 -> U=1) then
    // killed the cart PERMANENTLY: every later command stalled the SM on the
    // length autopull (final state PC=8, RXF=2, loader "failed to open pico
    // loader file"). Instead: drop whatever half-command is left in the FIFOs
    // and re-align the dispatcher; the CEB-rise recovery in gpioIrq kicks the
    // SM if it is not parked. The console retries the transaction.
    pio_sm_clear_fifos(pio, 0);
    gCartSdE4LenArmed = 0;
    romEmu->wordIdx = 0;
}

void __time_critical_func(ntrc_gameNoScrambleCmd0Dummy)(ntr_rom_emu_t* romEmu, u32 word, pio_hw_t* pio)
{
#ifdef ENABLE_UART_LOG
    gCartSdDummyCmd0++;
#endif
    ntrc_finishGameNoScrambleCmd0(romEmu);
}

void __time_critical_func(ntrc_gameNoScrambleCmd1Dummy0)(ntr_rom_emu_t* romEmu, u32 word, pio_hw_t* pio)
{
    ntrc_finishGameNoScrambleCmd1(romEmu);
}

void __time_critical_func(ntrc_gameNoScrambleCmd1Dummy4)(ntr_rom_emu_t* romEmu, u32 word, pio_hw_t* pio)
{
    ntrc_finishGameNoScrambleCmd1(romEmu);
}

void __scratch_y("cpu0") ntrc_gameNoScrambleReadIdCmd0(ntr_rom_emu_t* romEmu, u32 word, pio_hw_t* pio)
{
    ntrc_beginWrite(pio, 4);
    ntrc_writeWord(pio, romEmu->cardId);
    ntrc_finishGameNoScrambleCmd0(romEmu);
}

extern void ntrc_gameNoScrambleCmd0Handler(ntr_rom_emu_t* romEmu, u32 word, pio_hw_t* pio);
extern void ntrc_gameNoScrambleCmd1Handler(ntr_rom_emu_t* romEmu, u32 word, pio_hw_t* pio);

void __time_critical_func(ntrc_setGameNoScrambleMode)(void)
{
    gNtrRomEmu.cmd0Handler = ntrc_gameNoScrambleCmd0Handler;
    gNtrRomEmu.cmd1Handler = ntrc_gameNoScrambleCmd1Handler;
    gNtrRomEmu.mode = NTR_CARD_MODE_GAME_NO_SCRAMBLE;
    pwr_enableAfterBootPowerSaving();
}
