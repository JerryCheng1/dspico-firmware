#pragma once

#include "cachePageMap.h"

typedef struct {
    bool (*read)(uint32_t chip, uint32_t addr, void* dst, uint32_t len);
    bool (*write)(uint32_t chip, uint32_t addr, const void* src, uint32_t len);
} cachePageIo;

typedef enum {
    CACHE_PAGE_TASK_NONE = 0,
    CACHE_PAGE_TASK_FILL,
    CACHE_PAGE_TASK_READ,
    CACHE_PAGE_TASK_PROBE,
} cachePageTaskKind;

typedef enum {
    CACHE_PAGE_BUSY = 0,
    CACHE_PAGE_DONE,
    CACHE_PAGE_IO_FAIL,
    CACHE_PAGE_CORRUPT,
    CACHE_PAGE_CANCELLED,
} cachePageResult;

typedef struct {
    cachePageLease lease;
    const uint8_t* source; // immutable SD snapshot for FILL/PROBE
    uint8_t* destination;  // independent 512 B stage for READ
    uint8_t table[32];
    uint8_t scratch[32];
    uint32_t expected_table_crc;
    uint32_t sector_crc;
    uint32_t offset;
    uint32_t first_bad_offset;
    uint32_t read_bytes;
    uint32_t write_bytes;
    uint8_t kind;
    uint8_t phase;
} cachePageTask;

typedef struct {
    cachePageMap map;
} cachePageStore;

#ifdef __cplusplus
extern "C" {
#endif

CACHE_PAGE_RAM_CODE void cachePageStoreInit(cachePageStore* store, uint8_t usable_mask);
CACHE_PAGE_RAM_CODE bool cachePageStoreBeginFill(cachePageStore* store, cachePageTask* task,
                             uint32_t sector, uint32_t epoch,
                             const uint8_t* source);
CACHE_PAGE_RAM_CODE bool cachePageStoreBeginRead(cachePageStore* store, cachePageTask* task,
                             uint32_t sector, uint32_t epoch,
                             uint8_t* destination);
CACHE_PAGE_RAM_CODE bool cachePageStoreBeginProbe(cachePageStore* store, cachePageTask* task,
                              uint32_t sector, uint32_t epoch,
                              const uint8_t* reference);
// Exactly one PSRAM transport of at most 32 B per call. The caller applies
// its foreground/quiet-window gate before each step.
CACHE_PAGE_RAM_CODE cachePageResult cachePageStoreStep(cachePageStore* store, cachePageTask* task,
                                   const cachePageIo* io);
CACHE_PAGE_RAM_CODE void cachePageStoreCancel(cachePageStore* store, cachePageTask* task);
CACHE_PAGE_RAM_CODE uint32_t cachePageStoreValidSectors(const cachePageStore* store, uint32_t epoch);

#ifdef __cplusplus
}
#endif
