#include "common.h"
#include <stdio.h>
#include "hardware/pio.h"
#include "ntrCardRom.h"
#include "powerSaving.h"
#include "ntrCardRomGameNoScramble.h"

// Functional (not just diagnostic): set while an E4 poll pre-armed length
// word sits in the TX FIFO. pio_sm_clear_fifos below drops that word, so the
// flag must go with it.
extern volatile u32 gCartSdE4LenArmed;
extern volatile u32 gCartSdRecoveryPending;
extern volatile u32 gCartSdFifoRecovery;

void ntrc_gameNoScrambleCmd1Unknown(ntr_rom_emu_t* romEmu, u32 word, pio_hw_t* pio)
{
    // Unknown framing is a whole-transaction failure. Never clear RX while
    // the SM can still capture command bytes; CEB-high recovery owns reset.
    (void)word;
    (void)pio;
    if (!gCartSdRecoveryPending)
        gCartSdFifoRecovery++;
    gCartSdRecoveryPending = 1;
    romEmu->wordIdx = 0;
}

void __time_critical_func(ntrc_gameNoScrambleCmd0Dummy)(ntr_rom_emu_t* romEmu, u32 word, pio_hw_t* pio)
{
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
