
#include "scheduler.h"
#include "console.h"

#include <stm32f0xx_ll_gpio.h>

#include <assert.h>
#include <string.h>
#include <stdio.h>

static tim_task_t timer1, timer2;
static task_t task1, task2;

static void set_pin1(tim_task_t *task, state_t state, uint32_t expiry)
{
    GPIOA->ODR |= GPIO_ODR_1;
}

static void unset_pin1(tim_task_t *task, state_t state, uint32_t expiry)
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

static void toggle_pin2(tim_task_t *task, state_t state, uint32_t expiry)
{
    GPIOA->ODR ^= GPIO_ODR_2;
}

static void write_hello(tim_task_t *task, state_t state, uint32_t expiry)
{
    static double f = 0;
    f += 0.5;
    printf("hello: %.2f\n", f);
}

int main()
{
    console_init();

    gpio_set_mode(GPIOA, 1, LL_GPIO_MODE_OUTPUT);
    gpio_set_mode(GPIOA, 2, LL_GPIO_MODE_OUTPUT);
    // gpio_set_mode(GPIOA, 3, LL_GPIO_MODE_OUTPUT);

    
    sched_init();
    sched_timer_init(&timer1, &write_hello, NULL);
    /*
    sched_timer_init(&timer2, &unset_pin1, NULL);
    sched_task_init(&task1, &set_pin1_task, NULL);
    sched_task_init(&task2, &unset_pin1_task, NULL);
    */

    sched_timer_schedule(&timer1, 5000, 1000000);
    // sched_timer_schedule(&timer2, 5300, 600);
   
    //sched_task_pending(&task1);
    //sched_task_pending(&task2);

    while (1) {
	// __NOP();
	__WFI();
    }
}
