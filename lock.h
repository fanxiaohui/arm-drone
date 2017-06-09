#pragma once

#include <stm32f0xx.h>

typedef volatile uint32_t lock_state_t;

static inline void lock_acquire(lock_state_t *state)
{
    *state = __get_PRIMASK();
    __disable_irq();
    __DMB();
}

static inline void lock_release(lock_state_t *state)
{
    __DMB();
    __set_PRIMASK(*state);
}
