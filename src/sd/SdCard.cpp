#include "../common.h"
#include "pico/multicore.h"
#include "hardware/clocks.h"
#include "hardware/sync.h"
#include "rp2350_sdio.h"
#include "SdCardInfo.h"
#include "SdCard.h"

#define SEQUENTIAL_READ_TIMEOUT_MICROSECONDS    1000000 // 1 second

// Blocking init-stage logging: unlike LOG (async ring), these flush even if
// the code halts right after the failure, so f_mount=FAIL is diagnosable.
// Kept under ENABLE_UART_LOG (boot-time only) independently of the cartridge
// runtime tracers.
#if defined(ENABLE_UART_LOG) && !defined(CACHE_SUMMARY_LOG)
#define SD_LOG(...) do { uartLogPrintfBlocking(__VA_ARGS__); } while (0)
#else
#define SD_LOG(...) do { } while (0)
#endif

sdio_status_t SdCard::Cmd0GoIdleState() const
{
    return rp2350_sdio_command_R1(CMD0, 0, NULL); // GO_IDLE_STATE
}

sdio_status_t SdCard::Cmd2AllSendCid(cid_t& cid) const
{
    return rp2350_sdio_command_R2(CMD2, 0, (u8*)&cid); // ALL_SEND_CID
}

sdio_status_t SdCard::Cmd3SendRelativeAddr(u32& rca) const
{
    return rp2350_sdio_command_R1(CMD3, 0, &rca);
}

sdio_status_t SdCard::Cmd7SelectCard(u32 rca) const
{
    u32 response;
    return rp2350_sdio_command_R1(CMD7, rca, &response);
}

sdio_status_t SdCard::Cmd8SendIfCond(u32 argument, u32& response) const
{
    return rp2350_sdio_command_R1(CMD8, argument, &response); // SEND_IF_COND
}

sdio_status_t SdCard::Cmd9SendCsd(u32 rca, csd_t& csd) const
{
    return rp2350_sdio_command_R2(CMD9, rca, (u8*)&csd);
}

sdio_status_t SdCard::Cmd12StopTransmission() const
{
    u32 response;
    return rp2350_sdio_command_R1(CMD12, 0, &response);
}

sdio_status_t SdCard::Cmd16SetBlocklen(u32 blockLength) const
{
    u32 response;
    return rp2350_sdio_command_R1(16, blockLength, &response);
}

sdio_status_t SdCard::Cmd17ReadSingleBlock(u32 address) const
{
    u32 response;
    return rp2350_sdio_command_R1(CMD17, address, &response);
}

sdio_status_t SdCard::Cmd18ReadMultipleBlock(u32 address) const
{
    u32 response;
    return rp2350_sdio_command_R1(CMD18, address, &response);
}

sdio_status_t SdCard::Cmd24WriteBlock(u32 address) const
{
    u32 response;
    return rp2350_sdio_command_R1(CMD24, address, &response);
}

sdio_status_t SdCard::Cmd25WriteMultipleBlock(u32 address) const
{
    u32 response;
    return rp2350_sdio_command_R1(CMD25, address, &response);
}

sdio_status_t SdCard::Cmd55AppCmd(u32 rca) const
{
    u32 response;
    return rp2350_sdio_command_R1(CMD55, rca, &response); // APP_CMD
}

sdio_status_t SdCard::ACmd6SetBusWidth(u32 argument) const
{
    u32 response;
    return rp2350_sdio_command_R1(ACMD6, argument, &response);
}

sdio_status_t SdCard::ACmd42ClrCardDetect() const
{
    // Arg 0 disables the card-detect pull-up on DAT3 (SD spec ACMD42). The
    // proven-good debug firmware issues this between CMD7 and ACMD6.
    u32 response;
    return rp2350_sdio_command_R1(42, 0, &response);
}

sdio_status_t SdCard::ACmd23SetWrBlkEraseCount(u32 count) const
{
    u32 response;
    return rp2350_sdio_command_R1(ACMD23, count, &response);
}

sdio_status_t SdCard::ACmd41SdSendOpCond(u32 argument, u32& response) const
{
    return rp2350_sdio_command_R3(ACMD41, argument, &response); // SD_SEND_OP_COND
}

bool SdCard::TryInitialize()
{
    u32 reply;
    sdio_status_t status;

    // Initialize at 400 kHz - EXACTLY the upstream RP2040 speed
    // (rp2040_sdio_init(62) at clk_sys=200 MHz). Use this build to verify the
    // hardware after rework: if the board is sound, original-speed init must
    // pass CMD8/ACMD41 cleanly every time.
    rp2350_sdio_init(62);

    // Establish initial connection with the card
    for (int retries = 0; retries < 5; retries++)
    {
        // sleep_us(1000);
        reply = 0;
        Cmd0GoIdleState();
        status = Cmd8SendIfCond(0x1AA, reply);
        SD_LOG("[SD-CMD] CMD0+CMD8 try=%d status=%d reply=%08lX\n",
               retries, (int)status, (unsigned long)reply);

        if (status == SDIO_OK && reply == 0x1AA)
        {
            break;
        }
    }

    if (reply != 0x1AA || status != SDIO_OK)
    {
        SD_LOG("[sd] init FAIL CMD8 status=%d reply=%08lX\n",
               (int)status, (unsigned long)reply);
        return false;
    }
    SD_LOG("[sd] init CMD8 OK\n");

    // Send ACMD41 to begin card initialization and wait for it to complete
    u32 start = millis();
    u32 loop = 0;
    do
    {
        status = Cmd55AppCmd(0);
        if (status != SDIO_OK)
        {
            SD_LOG("[SD-CMD] CMD55 -> %d loop=%lu ocr=%08lX\n",
                (int)status, (unsigned long)loop, (unsigned long)_sdioOcr);
            return false;
        }
        status = ACmd41SdSendOpCond(0xD0040000, _sdioOcr); // 3.0V voltage
        if (status != SDIO_OK)
        {
            SD_LOG("[SD-CMD] ACMD41 -> %d loop=%lu ocr=%08lX\n",
                (int)status, (unsigned long)loop, (unsigned long)_sdioOcr);
            return false;
        }

        if ((u32)(millis() - start) > 1000)
        {
            SD_LOG("[SD-CMD] ACMD41 TIMEOUT ocr=%08lX\n",
                (unsigned long)_sdioOcr);
            return false;
        }
        loop++;
    } while (!(_sdioOcr & (1 << 31)));
    SD_LOG("[SD-CMD] ACMD41 ready ocr=%08lX loop=%lu CCS=%lu\n",
        (unsigned long)_sdioOcr, (unsigned long)loop,
        (unsigned long)((_sdioOcr >> 30) & 1));

    // Get CID
    status = Cmd2AllSendCid(_sdioCid);
    SD_LOG("[SD-CMD] CMD2 CID -> %d\n", (int)status);
    if (status != SDIO_OK)
    {
        return false;
    }

    // Get relative card address
    status = Cmd3SendRelativeAddr(_sdioRca);
    SD_LOG("[SD-CMD] CMD3 RCA -> %d rca=%04lX\n",
        (int)status, (unsigned long)(_sdioRca >> 16));
    if (status != SDIO_OK)
    {
        return false;
    }

    // Debug-firmware parity: switch to the production transfer rate right
    // after RCA, before CSD/select/bus-width (all at 25 MHz there too).
    rp2350_sdio_init(1);
    SD_LOG("[SD-CMD] clock -> 25 MHz\n");

    // Get CSD
    status = Cmd9SendCsd(_sdioRca, _sdioCsd);
    SD_LOG("[SD-CMD] CMD9 CSD -> %d\n", (int)status);
    if (status != SDIO_OK)
    {
        return false;
    }

    // Select card
    status = Cmd7SelectCard(_sdioRca);
    SD_LOG("[SD-CMD] CMD7 SELECT -> %d\n", (int)status);
    if (status != SDIO_OK)
    {
        return false;
    }

    // Clear the card-detect pull-up (debug-firmware parity)
    status = Cmd55AppCmd(_sdioRca);
    if (status == SDIO_OK)
    {
        status = ACmd42ClrCardDetect();
    }
    SD_LOG("[SD-CMD] ACMD42 CLR_CARD_DETECT -> %d\n", (int)status);
    if (status != SDIO_OK)
    {
        return false;
    }

    // Set 4-bit bus mode
    status = Cmd55AppCmd(_sdioRca);
    if (status == SDIO_OK)
    {
        status = ACmd6SetBusWidth(2);
    }
    SD_LOG("[SD-CMD] ACMD6 BUS_WIDTH=4 -> %d\n", (int)status);
    if (status != SDIO_OK)
    {
        return false;
    }

    SD_LOG("[SD-CMD] INIT OK: %s %lu MiB\n",
        IsSdhcCard() ? "SDHC/SDXC" : "SDSC",
        (unsigned long)(CalculateSdCapacity() >> 1));

    _lastSdSector = CalculateSdCapacity() - 1;

    // rp2350_sdio_init() resets PIO1 and DMA2/3. Never retain a sequential
    // transfer cursor across that reset: the next request would otherwise use
    // RX-continue against a newly initialized state machine.
    _sequentialState = SequentialState::None;
    _nextSequentialSector = 0xFFFFFFFFu;
    _stopSequentialRead = false;
    _doStopTransmission = false;

    irq_set_exclusive_handler(TIMER0_IRQ_0, []
    {
        // INTR is write-one-to-clear, not a normal read/write register.
        timer0_hw->intr = TIMER_INTR_ALARM_0_BITS;
        gSdCard.NotifySequentialReadAlarm();
    });

    _state = State::Idle;

    return true;
}

void SdCard::Update()
{
    while (true)
    {
        switch (_state)
        {
            case State::Idle:
            {
                if (_stopSequentialRead)
                {
                    _stopSequentialRead = false;
                    if (_sequentialState == SequentialState::SequentialRead)
                    {
                        StopSequentialReadWrite();
                    }
                }
                return;
            }
            case State::ReadBegin:
            {
                StateReadBegin();
                break;
            }
            case State::ReadBusy:
            {
                StateReadBusy();
                break;
            }
            case State::ReadWriteCancel:
            {
                StateReadWriteCancel();
                break;
            }
            case State::WriteBegin:
            {
                StateWriteBegin();
                break;
            }
            case State::WriteBusy:
            {
                StateWriteBusy();
                break;
            }
            case State::Uninitialized:
            default:
            {
                return;
            }
        }
    }
}

void __time_critical_func(SdCard::StateReadBegin)(bool singleCommandAttempt)
{
    u32 sectorsLeft = _sectorCount - _sectorsCompleted;
    u32 startSector = _sectorAddress + _sectorsCompleted;
    if (startSector > _lastSdSector)
    {
        _state = State::Idle;
        return;
    }

    if ((u64)startSector + sectorsLeft - 1 > _lastSdSector)
    {
        sectorsLeft = _lastSdSector - startSector + 1;
    }

    if (sectorsLeft == 0)
    {
        _state = State::Idle;
        return;
    }

    u32 sdAddress = startSector;
    if (!IsSdhcCard())
    {
        sdAddress <<= 9; // for non-hc sd cards it's a byte address
    }
    
    if (_cancelRequested)
    {
        _state = State::Idle;
        return;
    }

    bool started = false;
    if (_sequentialState == SequentialState::SequentialRead &&
        _nextSequentialSector == startSector)
    {
        StopSequentialReadAlarm();
        rp2350_sdio_rx_continue(_buffer + _sectorsCompleted * 512, sectorsLeft);
        started = true;
    }
    else
    {
        if (_sequentialState != SequentialState::None)
        {
            StopSequentialReadWrite();
        }

        rp2350_sdio_rx_start(_buffer + _sectorsCompleted * 512, sectorsLeft);

        if (_cancelRequested)
        {
            rp2350_sdio_stop();
            _state = State::Idle;
            return;
        }

        _doStopTransmission = true;
        u32 commandAttempts = 0;
        while (!_cancelRequested)
        {
            commandAttempts++;
            if (Cmd18ReadMultipleBlock(sdAddress) == SDIO_OK)
            {
                started = true;
                break;
            }
            if (singleCommandAttempt)
                break;
        }
    }

    if (singleCommandAttempt && !started && !_cancelRequested)
    {
        // Do not retry SD commands inside PIO0_IRQ_0. Tear down the armed RX
        // DMA and leave ReadBegin for the normal main-loop retry path.
        rp2350_sdio_stop();
        _state = State::ReadBegin;
    }
    else if (_cancelRequested && started)
    {
        _state = State::ReadWriteCancel;
    }
    else
    {
        _state = State::ReadBusy;
    }
}

void SdCard::StateReadBusy()
{
    if (_cancelRequested)
    {
        _state = State::ReadWriteCancel;
        return;
    }
    // Port-regression fix (b7): no save_and_disable_interrupts() here. The
    // poll of a just-completed block runs the 512-byte CRC16 (~tens of us);
    // doing that with core0 interrupts disabled pends the cartridge PIO0_IRQ_0
    // past the NDSL's response window. The SM then stalls on the length
    // autopull, the NDSL clocks an empty/garbage response, and the bus
    // desyncs (b1 froze at poll 8, b5 at the first ready=1, b6 at poll 74 -
    // each exactly when a block completed). The atomic section existed for
    // the E4-side PollReadCompletionFromCartridgeIrq(), which was removed in
    // the cartridge-IRQ fix; Update() on the core0 main loop is now the only
    // caller of this path, so no concurrent cursor access is possible.
    auto blockStatus = rp2350_sdio_rx_poll_one_block();
    switch (blockStatus)
    {
        case SDIO_BLOCK_CRC_FAIL:
        case SDIO_BLOCK_TIMEOUT:
        {
            StopSequentialReadWrite();
            _state = State::ReadBegin; // restart from the failed sector
            break;
        }
        case SDIO_BLOCK_OK:
        {
            _sectorsCompleted++;
            break;
        }
        case SDIO_BLOCK_ALL_DONE:
        {
            _sectorsCompleted = _sectorCount;
            _nextSequentialSector = _sectorAddress + _sectorsCompleted;
            _sequentialState = SequentialState::SequentialRead;
            StartSequentialReadAlarm();
            _state = State::Idle;
            break;
        }
        case SDIO_BLOCK_NOT_READY:
        default:
        {
#if CACHE_STAGE >= 3
            // The DMA completion IRQ may have run after the poll sampled
            // NOT_READY but before this point. WFI would then wait for an
            // unrelated IRQ and prevent even the software timeout recheck.
            // Keep polling until an atomic wait/wakeup protocol exists.
            tight_loop_contents();
#else
            __wfi();
#endif
            break;
        }
    }
}

void SdCard::StateReadWriteCancel()
{
    StopSequentialReadWrite();
    _cancelRequested = false;
    _state = State::Idle;
}

void SdCard::StateWriteBegin()
{
    _writeOffset = _sectorsCompleted;
    u32 sectorsLeft = _sectorCount - _sectorsCompleted;
    u32 startSector = _sectorAddress + _sectorsCompleted;
    if (startSector > _lastSdSector)
    {
        _state = State::Idle;
        return;
    }

    if ((u64)startSector + sectorsLeft - 1 > _lastSdSector)
    {
        sectorsLeft = _lastSdSector - startSector + 1;
    }

    if (sectorsLeft == 0)
    {
        _state = State::Idle;
        return;
    }

    u32 sdAddress = startSector;
    if (!IsSdhcCard())
    {
        sdAddress <<= 9; // for non-hc sd cards it's a byte address
    }

    if (_cancelRequested)
    {
        _state = State::Idle;
        return;
    }

    if (_sequentialState == SequentialState::SequentialWrite &&
        _nextSequentialSector == startSector)
    {
        StopSequentialReadAlarm();
    }
    else
    {
        if (_sequentialState != SequentialState::None)
        {
            StopSequentialReadWrite();
        }

        _doStopTransmission = true;
        bool started = false;
        while (!_cancelRequested)
        {
            if (Cmd25WriteMultipleBlock(sdAddress) == SDIO_OK)
            {
                started = true;
                break;
            }
        }

        if (_cancelRequested)
        {
            if (started)
                _state = State::ReadWriteCancel;
            else
                _state = State::Idle;
            return;
        }
    }

    rp2350_sdio_tx_start(_buffer + _sectorsCompleted * 512, sectorsLeft);

    if (_cancelRequested)
        _state = State::ReadWriteCancel;
    else
        _state = State::WriteBusy;
}

void SdCard::StateWriteBusy()
{
    if (_cancelRequested)
    {
        _state = State::ReadWriteCancel;
        return;
    }

    u32 bytesCompleted = 0;
    auto status = rp2350_sdio_tx_poll(&bytesCompleted);
    switch (status)
    {
        case SDIO_BUSY:
        {
            _sectorsCompleted = _writeOffset + (bytesCompleted >> 9);
            break;
        }
        case SDIO_OK:
        {
            _sectorsCompleted = _sectorCount;
            _nextSequentialSector = _sectorAddress + _sectorsCompleted;
            if (_keepSequentialWriteOpen)
            {
                _sequentialState = SequentialState::SequentialWrite;
            }
            else
            {
                StopSequentialReadWrite();
            }
            _state = State::Idle;
            break;
        }
        default:
        {
            // bad things, retry
            StopSequentialReadWrite();
            _state = State::WriteBegin; // restart from the failed sector
            break;
        }
    }
}

void SdCard::StopSequentialReadWrite()
{
    StopSequentialReadAlarm();
    rp2350_sdio_stop();
    if (_doStopTransmission)
    {
        while (Cmd12StopTransmission() != SDIO_OK);
        while (IsCardBusy());
    }
    _sequentialState = SequentialState::None;
    _stopSequentialRead = false;
}

void SdCard::StartSequentialReadAlarm()
{
    StopSequentialReadAlarm(); // disarm and clear our old source before rearming
    _stopSequentialRead = false;
    // TIMER0 is also the SDK timebase. Keep it clocked in both power states;
    // owning alarm 0 does not give this driver ownership of the whole timer.
    hw_set_bits(&clocks_hw->sleep_en1, CLOCKS_SLEEP_EN1_CLK_SYS_TIMER0_BITS);
    hw_set_bits(&clocks_hw->wake_en1, CLOCKS_WAKE_EN1_CLK_SYS_TIMER0_BITS);
    u32 target = timer0_hw->timerawl + SEQUENTIAL_READ_TIMEOUT_MICROSECONDS;
    timer0_hw->alarm[0] = target;
    hw_set_bits(&timer0_hw->inte, TIMER_INTE_ALARM_0_BITS);
    irq_set_enabled(TIMER0_IRQ_0, true);
}

void SdCard::StopSequentialReadAlarm()
{
    irq_set_enabled(TIMER0_IRQ_0, false);
    hw_clear_bits(&timer0_hw->inte, TIMER_INTE_ALARM_0_BITS);
    timer0_hw->armed = 1u << 0; // ARMED is also write-one-to-clear
    timer0_hw->intr = TIMER_INTR_ALARM_0_BITS;
    irq_clear(TIMER0_IRQ_0);
}

u32 SdCard::CalculateSdCapacity() const
{
    if (_sdioCsd.v1.csd_ver == 0)
    {
        u16 cSize = (_sdioCsd.v1.c_size_high << 10) | (_sdioCsd.v1.c_size_mid << 2) | _sdioCsd.v1.c_size_low;
        u8 cSizeMult = (_sdioCsd.v1.c_size_mult_high << 1) | _sdioCsd.v1.c_size_mult_low;
        return (u32)(cSize + 1) << (cSizeMult + _sdioCsd.v1.read_bl_len - 7);
    }
    else if (_sdioCsd.v2.csd_ver == 1)
    {
        return (((u32)_sdioCsd.v2.c_size_high << 16) + ((u16)_sdioCsd.v2.c_size_mid << 8) + _sdioCsd.v2.c_size_low + 1) << 10;
    }
    else
    {
        return 0;
    }
}
