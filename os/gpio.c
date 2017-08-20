
#include "gpio.h"
#include "config.h"
#include "utils.h"

#include <stm32f1xx_ll_gpio.h>

INLINE void _gpio_set_cr(GPIO_TypeDef *gpiox, uint32_t pin, uint32_t mask)
{
    if (pin <= 7) {
        gpiox->CRL &= ~(0x0f << 4 * pin);
        gpiox->CRL |= mask << 4 * pin;
    } else {
        pin -= 8;
        gpiox->CRH &= ~(0x0f << 4 * pin);
        gpiox->CRH |= mask << 4 * pin;
    }
}

void gpio_set_mode(GPIO_TypeDef *gpiox, uint32_t pin, gpio_mode_t mode)
{
    unsigned int mode_mask;
    switch (mode) {
    case GPIO_MODE_INPUT:
        mode_mask = LL_GPIO_MODE_FLOATING;
        break;
    case GPIO_MODE_OUTPUT:
        mode_mask = LL_GPIO_OUTPUT_PUSHPULL | LL_GPIO_SPEED_FREQ_HIGH;
        break;
    }
    _gpio_set_cr(gpiox, pin, mode_mask);
}

void gpio_set_af_mode(GPIO_TypeDef *gpiox, uint32_t pin, gpio_mode_t mode)
{
    unsigned int mode_mask;
    switch (mode) {
    case GPIO_MODE_INPUT:
        mode_mask = LL_GPIO_MODE_FLOATING;
        break;
    case GPIO_MODE_OUTPUT:
        mode_mask = LL_GPIO_MODE_ALTERNATE;
        break;
    }
    _gpio_set_cr(gpiox, pin, mode_mask);
}
