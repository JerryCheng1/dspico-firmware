#pragma once

#include <stdint.h>

static inline uint64_t us_to_ms(uint64_t us)
{
    return us / 1000;
}
