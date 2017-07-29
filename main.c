
#include "scheduler.h"
#include "console.h"
#include "exti.h"
#include "buttons.h"
#include "spi.h"
#include "nrf24l01p.h"

#include <stm32f0xx_ll_gpio.h>

#include <assert.h>
#include <string.h>
#include <stdio.h>

static tim_task_t timer1, timer2;
static task_t task1, task2;

static nrf24l_t nrf24l;

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

static void send_payload(tim_task_t *task, state_t prev_state, uint32_t expiry_time)
{
    static unsigned int pdu = 0;

    ++pdu;
    printf("sending payload: %u\n", pdu);
    nrf24l_send(&nrf24l, &pdu);
}

int main()
{
    console_init();
    sched_init();
    exti_init();
    spi_init();
    // buttons_init(&button_changed, "callback");

    // nRF24L01+ pin assignments:
    // PA0 - IRQ
    // PA1 - CE
    // PA5 - SCK
    // PA6 - MISO
    // PA7 - MOSI
    // PB1 - CSN, software NSS

    printf("starting...\n");
    
    // initialise NRF24L01 specific pins
    nrf24l.irq_port = GPIOA;
    nrf24l.irq_pin = 0;
    nrf24l.ce_port = GPIOA;
    nrf24l.ce_pin = 1;
    nrf24l.nss.port = GPIOB;
    nrf24l.nss.pin = 1;
    nrf24l.payload_size = 4;
    nrf24l_init_ptx(&nrf24l);
    nrf24l_set_tx_address(&nrf24l, 0xAE058CB7);
    uint32_t addr = nrf24l_get_tx_address(&nrf24l);

    tim_task_t sender;
    sched_timer_init(&sender, &send_payload, NULL);
    sched_timer_schedule_rel(&sender, 1000, 1000000);
    
    while (1) {
	__WFI();
    }
}
