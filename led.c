
#include "scheduler.h"

#include <stm32f0xx_ll_gpio.h>

#include <assert.h>
#include <string.h>

static inline void gpio_set_mode(GPIO_TypeDef *gpiox, uint32_t pin, uint32_t mode)
{
    gpiox->MODER |= mode << 2 * pin;
    gpiox->OSPEEDR |= LL_GPIO_SPEED_FREQ_HIGH << 2 * pin;
    gpiox->PUPDR |= LL_GPIO_PULL_NO << 2 * pin;
    gpiox->OTYPER |= LL_GPIO_OUTPUT_PUSHPULL << pin;
    gpiox->BRR |= GPIO_BRR_BR_0 << pin;
}

static inline void gpio_set_af_mode(GPIO_TypeDef *gpiox, uint32_t pin, uint32_t af_mode)
{
    gpio_set_mode(gpiox, pin, LL_GPIO_MODE_ALTERNATE);

    if (pin <= 7) {
	gpiox->AFR[0] |= af_mode << 4 * pin;
    } else {
	gpiox->AFR[1] |= af_mode << 4 * (pin - 8);
    }
}

static void toggle_pin1(task_t *task, state_t state, uint32_t expiry)
{
    GPIOA->ODR ^= GPIO_ODR_1;
}

static void toggle_pin2(task_t *task, state_t state, uint32_t expiry)
{
    GPIOA->ODR ^= GPIO_ODR_2;
}

void main()
{
    static task_t timer1, timer2;

    gpio_set_mode(GPIOA, 1, LL_GPIO_MODE_OUTPUT);
    gpio_set_mode(GPIOA, 2, LL_GPIO_MODE_OUTPUT);
    // gpio_set_mode(GPIOA, 3, LL_GPIO_MODE_OUTPUT);
    
    sched_init();
    sched_task_init(&timer1, &toggle_pin1, NULL);
    sched_task_init(&timer2, &toggle_pin2, NULL);

    uint32_t start = 5100;
    sched_task_schedule(&timer1, start, 400);
    // sched_task_schedule(&timer2, start, 20);
    
    while (1) {
	// __NOP();
	__WFI();
    }
}
