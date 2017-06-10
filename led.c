
#include "scheduler.h"

#include <stm32f0xx_ll_gpio.h>

#include <assert.h>
#include <string.h>

static timer_t timer1, timer2;
static task_t task1, task2;

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

static void set_pin1(timer_t *task, state_t state, uint32_t expiry)
{
    GPIOA->ODR |= GPIO_ODR_1;
}

static void unset_pin1(timer_t *task, state_t state, uint32_t expiry)
{
    GPIOA->ODR &= ~GPIO_ODR_1;
}

static void set_pin1_task(task_t *task)
{
    GPIOA->ODR |= GPIO_ODR_1;
}

static void unset_pin1_task(task_t *task)
{
    GPIOA->ODR &= ~GPIO_ODR_1;
}

static void toggle_pin2(timer_t *task, state_t state, uint32_t expiry)
{
    GPIOA->ODR ^= GPIO_ODR_2;
}

void main()
{
    gpio_set_mode(GPIOA, 1, LL_GPIO_MODE_OUTPUT);
    gpio_set_mode(GPIOA, 2, LL_GPIO_MODE_OUTPUT);
    // gpio_set_mode(GPIOA, 3, LL_GPIO_MODE_OUTPUT);
    
    sched_init();
    sched_timer_init(&timer1, &set_pin1, NULL);
    sched_timer_init(&timer2, &unset_pin1, NULL);
    sched_task_init(&task1, &set_pin1_task, NULL);
    sched_task_init(&task2, &unset_pin1_task, NULL);

    uint32_t start = 5350;
    sched_timer_schedule(&timer1, 5100, 600);
    sched_timer_schedule(&timer2, 5300, 600);
   
    //sched_task_pending(&task1);
    //sched_task_pending(&task2);

    while (1) {
	// __NOP();
	__WFI();
    }
}
