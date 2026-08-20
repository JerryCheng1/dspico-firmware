#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/// @brief Selects the conservative RP2350 ordinary-WFI power policy.
void pwr_initPowerSaving(void);

/// @brief Keeps RP2350 PIO/DMA wake clocks available in unscrambled game mode.
void pwr_enableAfterBootPowerSaving(void);

/// @brief Keeps RP2350 PIO/DMA wake clocks available when scrambling resumes.
void pwr_disableAfterBootPowerSaving(void);

#ifdef __cplusplus
}
#endif
