
#include "utils.h"

#include <stm32f0xx_ll_gpio.h>

void gpio_set_mode(GPIO_TypeDef *gpiox, uint32_t pin, uint32_t mode)
{
    gpiox->MODER |= mode << 2 * pin;
    gpiox->OSPEEDR |= LL_GPIO_SPEED_FREQ_HIGH << 2 * pin;
    gpiox->PUPDR |= LL_GPIO_PULL_NO << 2 * pin;
    gpiox->OTYPER |= LL_GPIO_OUTPUT_PUSHPULL << pin;
    gpiox->BRR |= GPIO_BRR_BR_0 << pin;
}

void gpio_set_af_mode(GPIO_TypeDef *gpiox, uint32_t pin, uint32_t af_mode)
{
    gpio_set_mode(gpiox, pin, LL_GPIO_MODE_ALTERNATE);

    if (pin <= 7) {
	gpiox->AFR[0] |= af_mode << 4 * pin;
    } else {
	gpiox->AFR[1] |= af_mode << 4 * (pin - 8);
    }
}
