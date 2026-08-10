#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef unsigned int uint;

uint64_t time_us_64(void);

static inline void __sev(void)
{
}
