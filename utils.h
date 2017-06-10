#pragma once

#include <stm32f0xx.h>

#define INLINE __attribute__((always_inline)) static inline

typedef volatile uint32_t crit_state_t;

INLINE void enter_crit()
{
    __disable_irq();
    __DMB();
}

INLINE void exit_crit()
{
    __DMB();
    __enable_irq();
}

INLINE void enter_crit_rec(crit_state_t *state)
{
    *state = __get_PRIMASK();
    __disable_irq();
    __DMB();
}

INLINE void exit_crit_rec(crit_state_t *state)
{
    __DMB();
    __set_PRIMASK(*state);
}
