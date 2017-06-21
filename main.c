
#include "scheduler.h"
#include "console.h"
#include "exti.h"
#include "buttons.h"

#include <stm32f0xx_ll_gpio.h>

#include <assert.h>
#include <string.h>
#include <stdio.h>

static tim_task_t timer1, timer2;
static task_t task1, task2;

/*
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
*/

static void button_changed(uint16_t changed, uint16_t state, void *client_data)
{
    const char *text = (const char *) client_data;
    printf("button cb, changed: %u, state: %u, text: %s\n", changed, state, text);
}

int main()
{
    console_init();
    sched_init();
    exti_init();
    buttons_init(&button_changed, "callback");

    while (1) {
	__WFI();
    }
}
