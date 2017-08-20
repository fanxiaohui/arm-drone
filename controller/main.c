
#include <os/gpio.h>
#include <os/scheduler.h>
#include <os/console.h>
#include <os/exti.h>
#include <os/spi.h>
#include <os/nrf24l01p.h>

#include <assert.h>
#include <string.h>
#include <stdio.h>

static void set_pin1(tim_task_t *task, state_t state, uint32_t expiry)
{
    GPIOA->ODR |= 1 << 1;
}

static void unset_pin1(tim_task_t *task, state_t state, uint32_t expiry)
{
    GPIOA->ODR &= ~(1 << 1);
}

int main()
{
    // console_init();
    sched_init();
    // exti_init();
    // spi_init();
    // buttons_init(&button_changed, "callback");

    gpio_set_mode(GPIOA, 1, GPIO_MODE_OUTPUT);
    
    tim_task_t timer1, timer2;
    sched_timer_init(&timer1, &set_pin1, NULL);
    sched_timer_schedule_at(&timer1, 10000, 1000000);

    sched_timer_init(&timer2, &unset_pin1, NULL);
    sched_timer_schedule_at(&timer2, 510000, 1000000);
    
    while (1) {
	__WFI();
    }
}
