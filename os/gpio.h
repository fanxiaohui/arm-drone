#pragma once

#include "config.h"

/*----------------------------------------------------------------------
 * GPIO functions
 *----------------------------------------------------------------------*/

typedef enum {
    GPIO_MODE_INPUT,
    GPIO_MODE_OUTPUT
} gpio_mode_t;

extern void gpio_set_mode(GPIO_TypeDef *gpiox, uint32_t pin, gpio_mode_t mode);
extern void gpio_set_af_mode(GPIO_TypeDef *gpiox, uint32_t pin, gpio_mode_t mode);
