#pragma once
#include "rp2350_sdio.h"
#include "SdCardInfo.h"

class SdCard
{
public:
#ifdef ENABLE_SD_WRITE_PROTECT
    /// @brief Compile-time flag: \c true when SD card write protection is built in.
    static constexpr bool WRITE_PROTECT_ENABLED = true;
#else
    static constexpr bool WRITE_PROTECT_ENABLED = false;
#endif

    /// @brief Tries to initialize the SD card.
    /// @return \c true if initialization was successful, or \c false otherwise.
    bool TryInitialize();

    /// @brief Returns if the SD card is ready.
    /// @return \c true if the SD card is ready, or \c false otherwise.
    bool IsReady() const { return _state == State::Idle; }

    /// @brief Returns if the SD card is currently writing.
    /// @return \c true if the SD card is writing, or \c false otherwise.
    bool IsWriting() const { return _state == State::WriteBegin || _state == State::WriteBusy; }

    /// @brief Starts concise UART tracing for loader-time SD reads. Physical
    ///        FatFs mount traffic before this call is intentionally omitted.
    void EnableRuntimeDiagnostics() { _runtimeDiagnostics = true; }

    /// @brief Tries to begin a read at the given \p sector and sector \p count. The data will be written to the \p dst buffer.
    /// @param dst The destination buffer.
    /// @param sector The sector to start reading at.
    /// @param count The number of sectors to read.
    /// @return \c true if starting the read was successful, or \c false otherwise.
    bool TryBeginReadSectors(u8* dst, u32 sector, u32 count)
    {
        if (!IsReady())
        {
            return false;
        }

        _buffer = dst;
        _sectorAddress = sector;
        _sectorCount = count;
        _sectorsCompleted = 0;
        _cancelRequested = false;
        _state = State::ReadBegin;
        return true;
    }

    /// @brief Whether SD card write protection is compiled in.
    /// @return \c true when the ENABLE_SD_WRITE_PROTECT feature macro is set.
    static constexpr bool IsWriteProtected() { return WRITE_PROTECT_ENABLED; }

    /// @brief Tries to begin a write to the given \p sector and sector \p count. The data will be read from the \p src buffer.
    /// @param src The source buffer.
    /// @param sector The sector to start writing at.
    /// @param count The number of sectors to write.
    /// @param keepSequentialWriteOpen When \c true, the sequential write is kept open such that
    ///                                more sectors can be sequentially written afterwards.
    ///                                When \c false the sequential write will end.
    /// @return \c true if starting the write was successful, or \c false otherwise.
    bool TryBeginWriteSectors(const u8* src, u32 sector, u32 count, bool keepSequentialWriteOpen)
    {
        if (!IsReady())
        {
            return false;
        }

#ifdef ENABLE_SD_WRITE_PROTECT
        // SD card write protection (compile-time, see ENABLE_SD_WRITE_PROTECT).
        // This is the single chokepoint every write path funnels through: the
        // DS-side SD (F6) commands, the R4 emulated writes and the FatFs
        // disk_write() glue. We silently swallow the request instead of
        // issuing any write command to the card: the sectors are reported as
        // already completed and the state machine returns to Idle, so callers
        // that block on IsReady()/TryWriteSectorsSync() (or that trap a false
        // return with __breakpoint()) all see a fast, successful write while
        // the card contents stay untouched. Reads are unaffected.
        (void)src;
        (void)keepSequentialWriteOpen;
        _buffer = nullptr;
        _sectorAddress = sector;
        _sectorCount = count;
        _sectorsCompleted = count;
        _cancelRequested = false;
        _state = State::Idle;
        return true;
#else
        _buffer = (u8*)src;
        _sectorAddress = sector;
        _sectorCount = count;
        _sectorsCompleted = 0;
        _cancelRequested = false;
        _keepSequentialWriteOpen = keepSequentialWriteOpen;
        _state = State::WriteBegin;
        return true;
#endif
    }

    /// @brief Requests a cancel of the current read or write.
    void Cancel()
    {
        if (!IsReady() && _state != State::Uninitialized)
        {
            _cancelRequested = true;
        }
    }

    /// @brief Updates the SD state machine.
    void Update();

    /// @brief Performs one bounded, non-blocking read-completion poll from the
    ///        cartridge IRQ. This never starts/retries/stops a transaction and
    ///        never waits; it only makes an already DMA-complete read visible
    ///        before the loader's current E4 status response is returned.
    void PollReadCompletionFromCartridgeIrq();

    /// @brief Starts a queued read from the E3/E5 cartridge IRQ before the
    ///        loader can begin its high-priority E4 polling burst. The SD data
    ///        transfer remains asynchronous; this call never waits for data.
    /// @return true when the queued request reached ReadBusy.
    bool KickPendingReadFromCartridgeIrq();

    /// @brief Returns the number of sectors that have been completed in the current read or write.
    /// @return The number of sectors that have been completed in the current read or write.
    u32 GetSectorsCompleted() const { return _sectorsCompleted; }

    // Hang-diagnostics heartbeat (DSPICO_HANG_DIAGNOSTIC): raw state machine
    // snapshot so a TIMER1 IRQ can report where the core0 main loop is even
    // when it is stuck inside Update().
    int DebugState() const { return (int)_state; }
    int DebugSequentialState() const { return (int)_sequentialState; }
    u32 DebugSectorAddress() const { return _sectorAddress; }

    /// @brief Reads the given number of sectors from the given \p sector to the \p dst buffer.
    ///        This function blocks until the read is complete.
    /// @param dst The destination buffer.
    /// @param sector The sector to start reading at.
    /// @param count The number of sectors to read.
    /// @return \c true if the read was successful, or \c false otherwise.
    bool TryReadSectorsSync(u8* dst, u32 sector, u32 count)
    {
        bool success = TryBeginReadSectors(dst, sector, count);
        if (success)
        {
            Update();
        }
        return success;
    }

    /// @brief Writes the given number of sectors from the \p src buffer to the given \p sector.
    ///        This function blocks until the write is complete.
    /// @param src The source buffer.
    /// @param sector The sector to start writing at.
    /// @param count The number of sectors to write.
    /// @return \c true if the write was successful, or \c false otherwise.
    bool TryWriteSectorsSync(const u8* src, u32 sector, u32 count)
    {
        bool success = TryBeginWriteSectors(src, sector, count, false);
        if (success)
        {
            Update();
        }
        return success;
    }

private:
    enum class State
    {
        Uninitialized,
        Idle,
        ReadBegin,
        ReadBusy,
        ReadWriteCancel,
        WriteBegin,
        WriteBusy
    };

    enum class SequentialState
    {
        None,
        SequentialRead,
        SequentialWrite
    };

    volatile State _state = State::Uninitialized;

    u32 _sdioOcr;
    u32 _sdioRca;
    cid_t _sdioCid;
    csd_t _sdioCsd;
    u32 _lastSdSector;

    u32 _sectorAddress;
    u32 _sectorCount;
    volatile u32 _sectorsCompleted;
    u8* _buffer;
    bool _doStopTransmission;
    bool _runtimeDiagnostics = false;
    bool _traceCurrentRead = false;
    u32 _diagnosticReadCount = 0;
    bool _keepSequentialWriteOpen;
    u32 _writeOffset;

    SequentialState _sequentialState = SequentialState::None;
    u32 _nextSequentialSector = 0xFFFFFFFFu;
    volatile bool _stopSequentialRead = false;
    
    volatile bool _cancelRequested = false;

    sdio_status_t Cmd0GoIdleState() const;
    sdio_status_t Cmd2AllSendCid(cid_t& cid) const;
    sdio_status_t Cmd3SendRelativeAddr(u32& rca) const;
    sdio_status_t Cmd7SelectCard(u32 rca) const;
    sdio_status_t Cmd8SendIfCond(u32 argument, u32& response) const;
    sdio_status_t Cmd9SendCsd(u32 rca, csd_t& csd) const;
    sdio_status_t Cmd12StopTransmission() const;
    sdio_status_t Cmd16SetBlocklen(u32 blockLength) const;
    sdio_status_t Cmd17ReadSingleBlock(u32 address) const;
    sdio_status_t Cmd18ReadMultipleBlock(u32 address) const;
    sdio_status_t Cmd24WriteBlock(u32 address) const;
    sdio_status_t Cmd25WriteMultipleBlock(u32 address) const;
    sdio_status_t Cmd55AppCmd(u32 rca) const;
    sdio_status_t ACmd6SetBusWidth(u32 argument) const;
    sdio_status_t ACmd42ClrCardDetect() const;
    sdio_status_t ACmd23SetWrBlkEraseCount(u32 count) const;
    sdio_status_t ACmd41SdSendOpCond(u32 argument, u32& response) const;

    bool IsSdhcCard() const { return (_sdioOcr & (1 << 30)) != 0; }
    bool IsCardBusy() const { return (sio_hw->gpio_in & (1 << SDIO_D0)) == 0; }

    void StateReadBegin(bool singleCommandAttempt = false);
    void StateReadBusy();
    void StateReadWriteCancel();
    void StateWriteBegin();
    void StateWriteBusy();

    void StopSequentialReadWrite();

    void StartSequentialReadAlarm();
    void StopSequentialReadAlarm();
    void NotifySequentialReadAlarm()
    {
        StopSequentialReadAlarm();
        if (IsReady() && _sequentialState == SequentialState::SequentialRead)
        {
            _stopSequentialRead = true;
        }
    }

    u32 CalculateSdCapacity() const;
};
