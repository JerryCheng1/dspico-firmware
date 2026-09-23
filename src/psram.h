#pragma once
#include "common.h"

// APS6404L PSRAM subsystem for the RP2354A board wiring in
// docs/RP2354A_全硬件无干扰缓存设计.md section 2.5.1:
//   SCLK  = GPIO0
//   SIO0..3 = GPIO22..25   (shared data bus)
//   CE0..3  = GPIO26..29   (one per device, active low, idle high)
//
// This is the S2 qualification backend. It deliberately does not touch the
// cartridge GPIO9..21, the SDIO GPIO3..8 or the UART pins; a caller is
// responsible for keeping the NDS bus high-Z when this runs standalone.

#define PSRAM_PIN_SCLK   0
#define PSRAM_PIN_IO0    22
#define PSRAM_PIN_IO1    23
#define PSRAM_PIN_IO2    24
#define PSRAM_PIN_IO3    25
#define PSRAM_PIN_CE0    26
#define PSRAM_PIN_CE1    27
#define PSRAM_PIN_CE2    28
#define PSRAM_PIN_CE3    29

#define PSRAM_IO_MASK    (0xFu << PSRAM_PIN_IO0)
#define PSRAM_SCLK_MASK  (1u << PSRAM_PIN_SCLK)
#define PSRAM_CE_MASK    (0xFu << PSRAM_PIN_CE0)
#define PSRAM_PIN_MASK   (PSRAM_IO_MASK | PSRAM_SCLK_MASK | PSRAM_CE_MASK)

#define PSRAM_CHIP_COUNT 4
// APS6404L-3SQR-ZR is 64 Mbit = 8 MiB (design section 2.4).
#define PSRAM_SIZE_BYTES (8u * 1024u * 1024u)

// Largest CE#-low fragment used by the engine. The APS6404L tCEM budget is
// 8 us (standard grade) and is only measured with the final board; 32 B keeps
// a large margin at the initial 20 MHz SCLK and never crosses the 1 KiB
// internal page because 32 divides 1024.
#define PSRAM_PIO_FRAG_BYTES 32
#define PSRAM_BB_FRAG_BYTES  16

// PIO2 SCLK generation. The qual sweep (docs/cache-stage-S2.md) measured the
// read-data limit at div=2 and a safe pass at div=3 (rd_sclk ~= 22 MHz on this
// board); div=3 leaves one step of margin, so the runtime cache uses it.
#define PSRAM_PIO_CLKDIV 3.0f

typedef struct
{
    u8 cePin;
    u8 refdes;           // 'U2'..'U6' printed in logs
    bool present;        // probe/ID indicates a device
    bool idValid;
    u8 id[16];           // raw 9Fh response (over-read: part replies may be
                         // preceded by idle bytes on this board)
    u32 idSizeBytes;     // decoded density, 0 when unknown
    bool quadReadOk;
    bool quadWriteOk;
} psramChipStatus;

#ifdef __cplusplus
extern "C" {
#endif

extern psramChipStatus gPsramChips[PSRAM_CHIP_COUNT];
// Selected runtime data path. true = PIO2 engine, false = SIO bit-bang.
extern volatile bool gPsramUsePio;

// Initialization layering (design section 8.4). One boolean could not express
// "resource is claimed" versus "the devices answered" versus "data actually
// round-trips" versus "the cache is allowed to use it".
typedef enum
{
    PSRAM_STATE_NONE = 0,
    PSRAM_STATE_RESOURCE_READY, // PIO2 SM + DMA4/5 claimed, pins muxed
    PSRAM_STATE_DEVICE_READY,   // every device answered the 9Fh ID probe
    PSRAM_STATE_SELFTEST_OK,    // isolated write/read-back verified
    PSRAM_STATE_RUNTIME_ENABLED,// cache backend may use the data path
} psramRuntimeState;

psramRuntimeState psramGetRuntimeState(void);
// Device bring-up: reset + raw-ID probe on every chip. Sets DEVICE_READY only
// when all four reply. Uses the qualification SIO path; independent of the
// data path, so a failure leaves SD service untouched.
bool psramInitDevice(void);
// Write/read-back an isolated block (never a live cache entry). Bounded by the
// transport spin limit; sets SELFTEST_OK only on a full compare.
bool psramSelfTest(void);
// Allow the cache data path. Ignored unless SELFTEST_OK (the caller decides for
// init-only modes, which never transport data).
void psramSetRuntimeEnabled(bool enabled);
u8 psramChipReadyMask(void);

// Configure the PSRAM GPIOs (SIO, CE high, SCLK low) and put the bus in a
// safe idle state. Must be called before any transaction.
void psramGpioInit(void);

// Build the PIO2 + DMA4/5 engine. Returns false when PIO2 is unavailable or
// DMA4/5 cannot be claimed; the caller then stays on the bit-bang path.
bool psramEngineInit(void);

// Reconfigure the PIO2 SM0 clock divider at runtime (qualification sweep).
// SCLK = sysclk / (2 * div) for the one-bit phases; the read phase is 3 SM
// cycles per nibble, so its SCLK is sysclk / (3 * div).
void psramSetClockDiv(float div);
float psramGetClockDiv(void);

// Chip select helpers (SIO). Select asserts exactly one CE# low.
void psramSelectChip(u32 chip);
void psramDeselectAll(void);

// Power-up software reset for one device: 66h then 99h, each inside its own
// CE# low pulse, followed by the reset recovery delay.
void psramResetChip(u32 chip);

// Reads the 16-byte raw 9Fh response. The APM manufacturer/KGD signature is
// located by scanning (some boards/devices reply with leading idle bytes), so
// the caller must treat the layout as measured, not assumed.
void psramReadId(u32 chip, u8 id[16]);

// Quad 1-4-4 data access, split into tCEM-safe fragments.
bool psramRead(u32 chip, u32 addr, void* buf, u32 len);
bool psramWrite(u32 chip, u32 addr, const void* buf, u32 len);

// Forced single-path variants used by the qualification firmware to compare
// the PIO engine against the proven bit-bang backend.
bool psramBitBangRead(u32 chip, u32 addr, void* buf, u32 len);
bool psramBitBangWrite(u32 chip, u32 addr, const void* buf, u32 len);

// Decode the density from a raw APS6404L ID; returns bytes or 0 if unknown.
// \p id is the 16-byte raw capture; the 0x0D/0x5D signature is searched for.
u32 psramDecodeIdSize(const u8 id[16], u32* manufacturer, u32* kgd);

#ifdef __cplusplus
}
#endif
