#include <stdarg.h>
#include <stdio.h>
#include "hardware/gpio.h"
#include "hardware/sync.h"
#include "hardware/uart.h"
#include "uartLog.h"

#define UART_LOG_TX_PIN       0
#define UART_LOG_RX_PIN       1
#define UART_LOG_RING_SIZE    2048u
#define UART_LOG_FORMAT_SIZE  160u

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
    // Reading a free RP2040 hardware spinlock atomically claims it and returns
    // nonzero. Never wait here: LOG may be reached from either core (or an
    // error IRQ path), and logging must not delay cartridge service.
    if (!*sLogLock)
        return false;
    __mem_fence_acquire();
    return true;
}

void uartLogInit(void)
{
    sLogRead = 0;
    sLogWrite = 0;
    sLogLock = spin_lock_instance(spin_lock_claim_unused(true));

    uart_init(uart0, PICO_DEFAULT_UART_BAUD_RATE);
    uart_set_fifo_enabled(uart0, true);
    gpio_set_function(UART_LOG_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_LOG_RX_PIN, GPIO_FUNC_UART);
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
    while (sLogRead != sLogWrite && uart_is_writable(uart0))
    {
        uart_putc_raw(uart0, sLogRing[sLogRead]);
        sLogRead = (sLogRead + 1u) & (UART_LOG_RING_SIZE - 1u);
    }
    bool pending = sLogRead != sLogWrite;
    spin_unlock_unsafe(sLogLock);
    return pending;
}
