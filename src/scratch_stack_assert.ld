/* Additional GNU ld script: augments Pico SDK's memmap_default.ld. */
ASSERT(__scratch_x_end__ <= __StackOneBottom,
       "SCRATCH_X overlaps the configured core1 stack")
ASSERT(__scratch_y_end__ <= __StackBottom,
       "SCRATCH_Y overlaps the configured core0 stack")
