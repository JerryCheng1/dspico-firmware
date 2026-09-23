#include <stdarg.h>
#include <stdio.h>
#include "hardware/gpio.h"
#include "hardware/regs/io_qspi.h"
#include "hardware/regs/usb.h"
#include "hardware/structs/io_qspi.h"
#include "hardware/structs/usb.h"
#include "hardware/sync.h"
#include "hardware/uart.h"
#include "uartLog.h"

#define UART_LOG_RING_SIZE    2048u
#define UART_LOG_FORMAT_SIZE  160u
#define UART_LOG_BAUD_RATE    115200u
// Overridable from CMake (-DUART_LOG_BUILD_TAG=...) so each baseline build
// prints a distinct tag on the boot banner; keep the r6 default untouched.
#ifndef UART_LOG_BUILD_TAG
#define UART_LOG_BUILD_TAG    "cache-send-r5"
#endif

_Static_assert((UART_LOG_RING_SIZE & (UART_LOG_RING_SIZE - 1u)) == 0,
               "UART log ring size must be a power of two");

static char sLogRing[UART_LOG_RING_SIZE];
static volatile unsigned sLogRead;
static volatile unsigned sLogWrite;
static spin_lock_t* sLogLock;

static inline bool uartLogPushChar(char c)
{
    unsigned next = (sLogWrite + 1u) & (UART_LOG_RING_SIZE - 1u);
    if (next == sLogRead)
        return false;
    sLogRing[sLogWrite] = c;
    sLogWrite = next;
    return true;
}

static inline bool uartLogTryLock(void)
{
    // RP2350 defaults to SDK software spinlocks (PICO_USE_SW_SPIN_LOCKS=1) to
    // work around RP2350-E2. Use the SDK operation rather than assuming direct
    // hardware-spinlock read-to-claim semantics. Never wait here: LOG may be
    // reached from either core (or an error IRQ path).
    return spin_try_lock_unsafe(sLogLock);
}

void uartLogInit(void)
{
    sLogRead = 0;
    sLogWrite = 0;
    sLogLock = spin_lock_instance(spin_lock_claim_unused(true));

    // RP2350 resets with the USB PHY isolated and connected to the USB
    // controller. In addition, pico-sdk powers the transceiver down before
    // main() when USB stdio is disabled. FUNCSEL in IO_QSPI has no physical
    // effect until all three conditions are undone. This is the same proven
    // sequence used by the RP2354A standalone hardware-test firmware.
    hw_clear_bits(&usb_hw->main_ctrl, USB_MAIN_CTRL_PHY_ISO_BITS);
    hw_clear_bits(&usb_hw->sie_ctrl,
                  USB_SIE_CTRL_TRANSCEIVER_PD_BITS |
                      USB_SIE_CTRL_PULLDOWN_EN_BITS);
    usb_hw->muxing = USB_USB_MUXING_USBPHY_AS_GPIO_BITS;

    // Keep DM input-only even if a boot path left an output override behind.
    // The UART must be configured before DP is connected, so the first level
    // driven onto the external pad is the UART's idle-high state.
    hw_write_masked(&io_qspi_hw->usbphy_dm_ctrl,
                    (IO_QSPI_USBPHY_DM_CTRL_FUNCSEL_VALUE_UART1_RX
                         << IO_QSPI_USBPHY_DM_CTRL_FUNCSEL_LSB) |
                        (IO_QSPI_USBPHY_DM_CTRL_OEOVER_VALUE_DISABLE
                         << IO_QSPI_USBPHY_DM_CTRL_OEOVER_LSB),
                    IO_QSPI_USBPHY_DM_CTRL_FUNCSEL_BITS |
                        IO_QSPI_USBPHY_DM_CTRL_OEOVER_BITS);

    uart_init(uart1, UART_LOG_BAUD_RATE);
    uart_set_fifo_enabled(uart1, true);

    // RP2350 Bank 1 exposes the USBPHY pads through IO_QSPI. They are not
    // Bank-0 GPIO numbers and therefore must not be passed to
    // gpio_set_function(). FUNCSEL=2 routes UART1 TX to DP. Force output
    // enable because this firmware deliberately repurposes a USBPHY pad as a
    // raw 3.3 V UART signal rather than enabling the USB controller.
    hw_write_masked(&io_qspi_hw->usbphy_dp_ctrl,
                    (IO_QSPI_USBPHY_DP_CTRL_FUNCSEL_VALUE_UART1_TX
                         << IO_QSPI_USBPHY_DP_CTRL_FUNCSEL_LSB) |
                        (IO_QSPI_USBPHY_DP_CTRL_OEOVER_VALUE_ENABLE
                         << IO_QSPI_USBPHY_DP_CTRL_OEOVER_LSB),
                    IO_QSPI_USBPHY_DP_CTRL_FUNCSEL_BITS |
                        IO_QSPI_USBPHY_DP_CTRL_OEOVER_BITS);

    // Earliest possible wire-level self-test. It intentionally bypasses the
    // asynchronous log ring, both cores and PSRAM qualification. Seeing this
    // line proves that the UART clock, peripheral, Bank-1 mux and TX pad work.
    uart_puts(uart1, "\r\n[UART] RP2354A UART1 ready: USB_DP TX, 115200 8N1; build="
                     UART_LOG_BUILD_TAG "\r\n");
    uart_tx_wait_blocking(uart1);
}

void uartLogPutsBlocking(const char* text)
{
    uart_puts(uart1, text);
    uart_tx_wait_blocking(uart1);
}

void uartLogPrintfBlocking(const char* format, ...)
{
    char formatted[UART_LOG_FORMAT_SIZE];
    va_list args;
    va_start(args, format);
    int length = vsnprintf(formatted, sizeof(formatted), format, args);
    va_end(args);
    if (length <= 0)
        return;
    if ((unsigned)length >= sizeof(formatted))
        length = sizeof(formatted) - 1;

    // Boot-only path: bypass the shared ring and its spinlock completely so
    // PSRAM qualification can identify its exact progress even if a later
    // operation blocks. Keep terminal-friendly CRLF translation here too.
    for (int i = 0; i < length; i++)
    {
        if (formatted[i] == '\n')
            uart_putc_raw(uart1, '\r');
        uart_putc_raw(uart1, formatted[i]);
    }
    uart_tx_wait_blocking(uart1);
}

void uartLogPrintf(const char* format, ...)
{
    if (!sLogLock)
        return;

    char formatted[UART_LOG_FORMAT_SIZE];
    va_list args;
    va_start(args, format);
    int length = vsnprintf(formatted, sizeof(formatted), format, args);
    va_end(args);
    if (length <= 0)
        return;
    if ((unsigned)length >= sizeof(formatted))
        length = sizeof(formatted) - 1;

    if (!uartLogTryLock())
        return;

    for (int i = 0; i < length; i++)
    {
        // Match Pico stdio's terminal-friendly CRLF translation without
        // calling its blocking UART driver.
        if (formatted[i] == '\n' && !uartLogPushChar('\r'))
            break;
        if (!uartLogPushChar(formatted[i]))
            break; // queue full: truncate this diagnostic, never block
    }
    spin_unlock_unsafe(sLogLock);

    // The log build uses WFE in the core0 idle path, so a core1 message wakes
    // it to continue draining even when the cartridge bus is quiet.
    __sev();
}

bool uartLogDrain(void)
{
    if (!sLogLock)
        return false;

    // Only core0 consumes. The lock is held for at most one hardware FIFO
    // fill and interrupts remain enabled; an IRQ-side LOG simply drops its
    // diagnostic if it finds this short critical section active.
    spin_lock_unsafe_blocking(sLogLock);
    while (sLogRead != sLogWrite && uart_is_writable(uart1))
    {
        uart_putc_raw(uart1, sLogRing[sLogRead]);

        sLogRead = (sLogRead + 1u) & (UART_LOG_RING_SIZE - 1u);
    }
    bool pending = sLogRead != sLogWrite;
    spin_unlock_unsafe(sLogLock);
    return pending;
}

void uartLogFlush(void)
{
    while (uartLogDrain())
    {
        // Boot-only diagnostic path: wait for FIFO space so a message emitted
        // immediately before a blocking SD operation is guaranteed visible.
    }
    uart_tx_wait_blocking(uart1);
}

bool uartLogDrainFromIrq(void)
{
    if (!sLogLock)
        return false;

    // Deadlock-safe variant for IRQ context: uartLogDrain() blocks on the
    // spinlock, so if it preempted a thread that holds the lock (e.g. the
    // core0 main loop mid-drain) the IRQ would spin forever while the
    // preempted lock owner can never run. Retry the try-lock a bounded
    // number of times instead; the next heartbeat finishes the drain.
    for (int i = 0; i < 200; i++)
    {
        if (spin_try_lock_unsafe(sLogLock))
        {
            while (sLogRead != sLogWrite && uart_is_writable(uart1))
            {
                uart_putc_raw(uart1, sLogRing[sLogRead]);
                sLogRead = (sLogRead + 1u) & (UART_LOG_RING_SIZE - 1u);
            }
            bool pending = sLogRead != sLogWrite;
            spin_unlock_unsafe(sLogLock);
            return pending;
        }
    }
    return true; // assume pending; retried on the next heartbeat
}
