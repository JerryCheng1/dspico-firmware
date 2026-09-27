#include "cachePageStore.h"

#include <string.h>

enum { PHASE_DATA = 0, PHASE_TABLE_READ, PHASE_TABLE_WRITE, PHASE_TABLE_VERIFY };

static CACHE_PAGE_RAM_CODE uint32_t crcUpdate(uint32_t crc, const uint8_t* data, uint32_t size)
{
    for (uint32_t i = 0; i < size; i++) {
        crc ^= data[i];
        for (uint32_t bit = 0; bit < 8u; bit++)
            crc = (crc >> 1) ^ (0xedb88320u & (0u - (crc & 1u)));
    }
    return crc;
}

static CACHE_PAGE_RAM_CODE uint32_t crc32(const uint8_t* data, uint32_t size)
{
    return crcUpdate(0xffffffffu, data, size) ^ 0xffffffffu;
}

void cachePageStoreInit(cachePageStore* store, uint8_t usable_mask)
{
    if (store) cachePageMapInit(&store->map, usable_mask);
}

static CACHE_PAGE_RAM_CODE void taskStart(cachePageTask* task, const cachePageLease* lease,
                      uint8_t kind, const uint8_t* source, uint8_t* destination,
                      uint32_t table_crc)
{
    memset(task, 0, sizeof(*task));
    task->lease = *lease;
    task->kind = kind;
    task->source = source;
    task->destination = destination;
    task->expected_table_crc = table_crc;
    task->sector_crc = 0xffffffffu;
    task->first_bad_offset = UINT32_MAX;
    task->phase = kind == CACHE_PAGE_TASK_FILL ? PHASE_DATA : PHASE_TABLE_READ;
}

bool cachePageStoreBeginFill(cachePageStore* store, cachePageTask* task,
                             uint32_t sector, uint32_t epoch,
                             const uint8_t* source)
{
    if (!store || !task || !source || task->kind) return false;
    cachePageLease lease;
    if (!cachePageMapBegin(&store->map, sector, epoch, &lease)) return false;
    uint32_t old_crc = store->map.entry[lease.slot].table_crc;
    taskStart(task, &lease, CACHE_PAGE_TASK_FILL, source, 0, old_crc);
    return true;
}

static CACHE_PAGE_RAM_CODE bool beginRead(cachePageStore* store, cachePageTask* task,
                      uint32_t sector, uint32_t epoch, uint8_t kind,
                      const uint8_t* reference, uint8_t* destination)
{
    if (!store || !task || task->kind) return false;
    cachePageLease lease;
    uint32_t table_crc;
    if (!cachePageMapLookup(&store->map, sector, epoch, &lease, &table_crc))
        return false;
    taskStart(task, &lease, kind, reference, destination, table_crc);
    return true;
}

bool cachePageStoreBeginRead(cachePageStore* store, cachePageTask* task,
                             uint32_t sector, uint32_t epoch,
                             uint8_t* destination)
{
    return destination && beginRead(store, task, sector, epoch,
                                    CACHE_PAGE_TASK_READ, 0, destination);
}

bool cachePageStoreBeginProbe(cachePageStore* store, cachePageTask* task,
                              uint32_t sector, uint32_t epoch,
                              const uint8_t* reference)
{
    return reference && beginRead(store, task, sector, epoch,
                                  CACHE_PAGE_TASK_PROBE, reference, 0);
}

void cachePageStoreCancel(cachePageStore* store, cachePageTask* task)
{
    if (!store || !task || !task->kind) return;
    if (task->kind == CACHE_PAGE_TASK_FILL)
        cachePageMapAbort(&store->map, &task->lease);
    else
        cachePageMapUnpin(&store->map, &task->lease);
    task->kind = CACHE_PAGE_TASK_NONE;
}

static CACHE_PAGE_RAM_CODE cachePageResult fail(cachePageStore* store, cachePageTask* task,
                            cachePageResult reason)
{
    if (task->kind == CACHE_PAGE_TASK_FILL) {
        cachePageMapAbort(&store->map, &task->lease);
    } else {
        cachePageMapUnpin(&store->map, &task->lease);
        if (reason == CACHE_PAGE_CORRUPT)
            cachePageMapInvalidate(&store->map, &task->lease);
    }
    task->kind = CACHE_PAGE_TASK_NONE;
    return reason;
}

cachePageResult cachePageStoreStep(cachePageStore* store, cachePageTask* task,
                                   const cachePageIo* io)
{
    if (!store || !task || !io || !io->read || !io->write || !task->kind)
        return CACHE_PAGE_CANCELLED;
    if (!cachePageMapLeaseCurrent(&store->map, &task->lease))
        return fail(store, task, CACHE_PAGE_CANCELLED);
    const uint32_t chip = cachePageMapChip(task->lease.slot);
    const uint32_t addr = cachePageMapDataAddr(task->lease.slot, task->lease.lane);
    const uint32_t table_addr = cachePageMapTableAddr(task->lease.slot);

    if (task->phase == PHASE_TABLE_READ) {
        if (!io->read(chip, table_addr, task->table, 32u))
            return fail(store, task, CACHE_PAGE_IO_FAIL);
        task->read_bytes += 32u;
        if (task->kind != CACHE_PAGE_TASK_FILL || task->lease.had_page) {
            if (crc32(task->table, 32u) != task->expected_table_crc)
                return fail(store, task, CACHE_PAGE_CORRUPT);
        } else {
            memset(task->table, 0, 32u);
        }
        if (task->kind == CACHE_PAGE_TASK_FILL) {
            uint32_t value = task->sector_crc ^ 0xffffffffu;
            memcpy(&task->table[task->lease.lane * 4u], &value, 4u);
            task->phase = PHASE_TABLE_WRITE;
        } else {
            memcpy(&task->expected_table_crc,
                   &task->table[task->lease.lane * 4u], 4u);
            task->phase = PHASE_DATA;
        }
        return CACHE_PAGE_BUSY;
    }

    if (task->phase == PHASE_TABLE_WRITE) {
        if (!io->write(chip, table_addr, task->table, 32u))
            return fail(store, task, CACHE_PAGE_IO_FAIL);
        task->write_bytes += 32u;
        task->phase = PHASE_TABLE_VERIFY;
        return CACHE_PAGE_BUSY;
    }

    if (task->phase == PHASE_TABLE_VERIFY) {
        if (!io->read(chip, table_addr, task->scratch, 32u))
            return fail(store, task, CACHE_PAGE_IO_FAIL);
        task->read_bytes += 32u;
        if (memcmp(task->scratch, task->table, 32u))
            return fail(store, task, CACHE_PAGE_CORRUPT);
        if (!cachePageMapCommit(&store->map, &task->lease,
                                crc32(task->table, 32u)))
            return fail(store, task, CACHE_PAGE_CANCELLED);
        task->kind = CACHE_PAGE_TASK_NONE;
        return CACHE_PAGE_DONE;
    }

    uint32_t remaining = 512u - task->offset;
    uint32_t len = remaining < 32u ? remaining : 32u;
    uint8_t* destination = task->kind == CACHE_PAGE_TASK_READ
                               ? &task->destination[task->offset]
                               : task->scratch;
    if (task->kind == CACHE_PAGE_TASK_FILL) {
        if (!io->write(chip, addr + task->offset, task->source + task->offset, len))
            return fail(store, task, CACHE_PAGE_IO_FAIL);
        task->write_bytes += len;
        task->sector_crc = crcUpdate(task->sector_crc,
                                     task->source + task->offset, len);
    } else {
        if (!io->read(chip, addr + task->offset, destination, len))
            return fail(store, task, CACHE_PAGE_IO_FAIL);
        task->read_bytes += len;
        task->sector_crc = crcUpdate(task->sector_crc, destination, len);
        if (task->kind == CACHE_PAGE_TASK_PROBE &&
            memcmp(destination, task->source + task->offset, len)) {
            for (uint32_t i = 0; i < len; i++) {
                if (destination[i] != task->source[task->offset + i]) {
                    task->first_bad_offset = task->offset + i;
                    break;
                }
            }
            return fail(store, task, CACHE_PAGE_CORRUPT);
        }
    }
    task->offset += len;
    if (task->offset < 512u) return CACHE_PAGE_BUSY;

    if (task->kind == CACHE_PAGE_TASK_FILL) {
        task->phase = PHASE_TABLE_READ;
        return CACHE_PAGE_BUSY;
    }
    if ((task->sector_crc ^ 0xffffffffu) != task->expected_table_crc)
        return fail(store, task, CACHE_PAGE_CORRUPT);
    cachePageMapUnpin(&store->map, &task->lease);
    task->kind = CACHE_PAGE_TASK_NONE;
    return CACHE_PAGE_DONE;
}

uint32_t cachePageStoreValidSectors(const cachePageStore* store, uint32_t epoch)
{
    if (!store) return 0;
    return store->map.count_epoch == epoch ? store->map.valid_count : 0u;
}
