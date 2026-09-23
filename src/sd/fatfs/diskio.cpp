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
#if CACHE_STAGE >= 3
#include "../../cacheSd.h"
#endif

/* Definitions of physical drive number for each drive */
#define DEV_SD		0

#if CACHE_STAGE >= 3
// Monotonic medium generation, bumped on every successful drive init.
static u32 sDiskMediaGeneration = 0;
static u32 diskMediaGeneration(void) { return ++sDiskMediaGeneration; }
#endif

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
            if (!gSdCard.TryInitialize())
            {
                return STA_NOINIT;
            }
#if CACHE_STAGE >= 3
            // Medium identity may have changed (re-init): invalidate every
            // cached read so nothing from the previous medium can be served.
            cacheSdSetMediaEpoch(diskMediaGeneration());
#endif
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
    (void)res;
    (void)result;

    switch (pdrv)
    {
        case DEV_SD:
        {
            gSdCard.ReadSectorsBlocking(buff, sector, count);
            return RES_OK;
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
#if CACHE_STAGE >= 3
            // Every FatFs write funnels through here: register the write
            // intent so no cached read can publish stale data afterwards. The
            // barrier spans the whole synchronous write (design section 8.1).
            u32 wtoken = cacheSdWriteBegin();
#endif
            gSdCard.WriteSectorsBlocking(buff, sector, count);
#if CACHE_STAGE >= 3
            cacheSdWriteEnd(wtoken);
#endif
            return RES_OK;
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
    return RES_PARERR;
}

