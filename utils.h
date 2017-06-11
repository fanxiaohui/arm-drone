#pragma once

#include <stm32f0xx.h>

/*----------------------------------------------------------------------
 * General macros
 *----------------------------------------------------------------------*/

#define INLINE __attribute__((always_inline)) static inline

/*----------------------------------------------------------------------
 * GPIO functions
 *----------------------------------------------------------------------*/

extern void gpio_set_mode(GPIO_TypeDef *gpiox, uint32_t pin, uint32_t mode);
extern void gpio_set_af_mode(GPIO_TypeDef *gpiox, uint32_t pin, uint32_t af_mode);

/*----------------------------------------------------------------------
 * Interrupt functions
 *----------------------------------------------------------------------*/

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
