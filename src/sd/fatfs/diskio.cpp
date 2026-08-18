/*-----------------------------------------------------------------------*/
/* Low level disk I/O module SKELETON for FatFs     (C)ChaN, 2019        */
/*-----------------------------------------------------------------------*/
/* If a working storage control module is available, it should be        */
/* attached to the FatFs via a glue function rather than modifying it.   */
/* This is an example of glue functions to attach various exsisting      */
/* storage control modules to the FatFs module with a defined API.       */
/*-----------------------------------------------------------------------*/

#include "../../common.h"
#include <stdio.h>
#include "pico/stdlib.h"
#include "ff.h"			/* Obtains integer types */
#include "diskio.h"		/* Declarations of disk functions */

/* Definitions of physical drive number for each drive */
#define DEV_SD		0

/*-----------------------------------------------------------------------*/
/* Get Drive Status                                                      */
/*-----------------------------------------------------------------------*/

extern "C" DSTATUS disk_status (
    BYTE pdrv		/* Physical drive nmuber to identify the drive */
)
{
    return 0;
}



/*-----------------------------------------------------------------------*/
/* Inidialize a Drive                                                    */
/*-----------------------------------------------------------------------*/

extern "C" DSTATUS disk_initialize (
    BYTE pdrv				/* Physical drive nmuber to identify the drive */
)
{
    switch (pdrv)
    {
        case DEV_SD:
        {
            printf("[SD] disk_initialize: calling TryInitialize...\n");
            bool ok = gSdCard.TryInitialize();
            printf("[SD] disk_initialize: TryInitialize=%d\n", ok ? 1 : 0);
            if (!ok)
            {
                return STA_NOINIT;
            }
            return 0;
        }
    }
    return STA_NOINIT;
}



/*-----------------------------------------------------------------------*/
/* Read Sector(s)                                                        */
/*-----------------------------------------------------------------------*/

extern "C" DRESULT __time_critical_func(disk_read) (
    BYTE pdrv,		/* Physical drive nmuber to identify the drive */
    BYTE *buff,		/* Data buffer to store read data */
    LBA_t sector,	/* Start sector in LBA */
    UINT count		/* Number of sectors to read */
)
{
    DRESULT res;
    int result;

    switch (pdrv)
    {
        case DEV_SD:
        {
            // dspico-debug: bounded retry. The upstream glue retried forever, which
            // wedged the test firmware on a flaky/absent card. Give up after a few
            // attempts so f_mount can fail and the PSRAM test keeps running.
            // Throttled trace so bulk sector reads do not flood the UART.
            static u32 sReadTrace = 0;
            sReadTrace++;
            if (sReadTrace <= 6 || (sReadTrace % 50) == 0)
                printf("[SD] disk_read sec=%lu cnt=%u (#%lu)\n",
                       (unsigned long)sector, (unsigned)count, (unsigned long)sReadTrace);
            for (int retry = 0; retry < 8; retry++)
            {
                if (gSdCard.TryReadSectorsSync(buff, sector, count))
                    return RES_OK;
                printf("[SD] disk_read retry %d failed, re-init card...\n", retry + 1);
                bool init = gSdCard.TryInitialize();
                printf("[SD] disk_read: re-init=%d\n", init ? 1 : 0);
            }
            printf("[SD] disk_read: giving up\n");
            return RES_ERROR;
        }
    }

    return RES_PARERR;
}



/*-----------------------------------------------------------------------*/
/* Write Sector(s)                                                       */
/*-----------------------------------------------------------------------*/

#if FF_FS_READONLY == 0

extern "C" DRESULT disk_write (
    BYTE pdrv,			/* Physical drive nmuber to identify the drive */
    const BYTE *buff,	/* Data to be written */
    LBA_t sector,		/* Start sector in LBA */
    UINT count			/* Number of sectors to write */
)
{
    DRESULT res;
    int result;

    switch (pdrv)
    {
        case DEV_SD:
        {
            // dspico-debug: bounded retry (see disk_read above). Throttled trace.
            static u32 sWriteTrace = 0;
            sWriteTrace++;
            if (sWriteTrace <= 6 || (sWriteTrace % 50) == 0)
                printf("[SD] disk_write sec=%lu cnt=%u (#%lu)\n",
                       (unsigned long)sector, (unsigned)count, (unsigned long)sWriteTrace);
            for (int retry = 0; retry < 8; retry++)
            {
                bool ok = gSdCard.TryWriteSectorsSync(buff, sector, count);
                if (ok)
                {
                    printf("[SD] disk_write ok (#%lu)\n", (unsigned long)sWriteTrace);
                    return RES_OK;
                }
                printf("[SD] disk_write retry %d failed\n", retry + 1);
                // A card that failed a write may be wedged (e.g. holding DAT0
                // busy forever). Give it a full reset/re-init cycle (CMD0 ->
                // ACMD41) before the next attempt instead of hammering it.
                printf("[SD] disk_write: re-init card...\n");
                bool init = gSdCard.TryInitialize();
                printf("[SD] disk_write: re-init=%d\n", init ? 1 : 0);
            }
            printf("[SD] disk_write: giving up\n");
            return RES_ERROR;
        }
    }

    return RES_PARERR;
}

#endif


/*-----------------------------------------------------------------------*/
/* Miscellaneous Functions                                               */
/*-----------------------------------------------------------------------*/

extern "C" DRESULT disk_ioctl (
    BYTE pdrv,		/* Physical drive nmuber (0..) */
    BYTE cmd,		/* Control code */
    void *buff		/* Buffer to send/receive control data */
)
{
    // dspico-debug: implemented the informational commands (sector count /
    // size / block size). The upstream stub returned RES_PARERR for everything.
    if (pdrv != DEV_SD || !gSdCard.IsReady())
        return RES_NOTRDY;

    switch (cmd)
    {
        case CTRL_SYNC:                 // Flush: SDIO writes are already sync; nothing pending.
            return RES_OK;

        case GET_SECTOR_COUNT:          // Total 512-byte sectors on the card.
            *(LBA_t*)buff = (LBA_t)gSdCard.GetSectorCount();
            return RES_OK;

        case GET_SECTOR_SIZE:           // SD sector size is always 512 bytes.
            *(WORD*)buff = 512;
            return RES_OK;

        case GET_BLOCK_SIZE:            // Erase-block alignment hint (in sectors).
            *(DWORD*)buff = 1;          // 1 = no alignment guarantee.
            return RES_OK;

        case CTRL_TRIM:                 // Optional; report unsupported.
        default:
            return RES_PARERR;
    }
}

