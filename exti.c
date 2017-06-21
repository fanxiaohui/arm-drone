
#include "exti.h"
#include "scheduler.h"

#include <assert.h>
#include <string.h>

#define EXTI_LINES_NUM	16

// list of irq handler registrations for each IRQ line
static exti_irq_handler_t *volatile exti_irq_handlers[EXTI_LINES_NUM];

static void exti_call_handlers(unsigned int irqn);

void exti_init()
{
    // enable all interrupt handlers, individual lines will be enabled
    // by the user
    NVIC_SetPriority(EXTI0_1_IRQn, SCHED_TASK_PRIORITY);
    NVIC_SetPriority(EXTI2_3_IRQn, SCHED_TASK_PRIORITY);
    NVIC_SetPriority(EXTI4_15_IRQn, SCHED_TASK_PRIORITY);
    NVIC_EnableIRQ(EXTI0_1_IRQn);
    NVIC_EnableIRQ(EXTI2_3_IRQn);
    NVIC_EnableIRQ(EXTI4_15_IRQn);
}

void exti_irq_handler_init(exti_irq_handler_t *handler,
			   exti_irq_handler_fn fn, void *client_data)
{
    handler->next = NULL;
    handler->irq_handler_fn = fn;
    handler->client_data = client_data;
}

void exti_register(exti_irq_handler_t *handler, unsigned int irqn)
{
    assert(irqn < EXTI_LINES_NUM);
    assert(!handler->next);
    
    // add new callback to the end of the queue
    exti_irq_handler_t * volatile *prev = &exti_irq_handlers[irqn];
    while (*prev) {
	prev = &(*prev)->next;
    }
    *prev = handler;
}

void exti_call_handlers(unsigned int irqn)
{
    for (exti_irq_handler_t *handler = exti_irq_handlers[irqn];
	 handler;
	 handler = handler->next) {
	(*handler->irq_handler_fn)(irqn, handler->client_data);
    }
}

void EXTI0_1_IRQHandler()
{
    if (EXTI->PR & EXTI_PR_PR0) {
	EXTI->PR = EXTI_PR_PR0;
	exti_call_handlers(0);
    } else if (EXTI->PR & EXTI_PR_PR1) {
	EXTI->PR = EXTI_PR_PR1;
	exti_call_handlers(1);
    }
}

void EXTI2_3_IRQHandler()
{
    if (EXTI->PR & EXTI_PR_PR2) {
	EXTI->PR = EXTI_PR_PR2;
	exti_call_handlers(2);
    } else if (EXTI->PR & EXTI_PR_PR3) {
	EXTI->PR = EXTI_PR_PR3;
	exti_call_handlers(3);
    }
}

void EXTI4_15_IRQHandler()
{
    for (unsigned int irq = 4, irq_bit = 1 << 4; irq <= 15; ++irq, irq_bit <<= 1) {
	if (EXTI->PR & irq_bit) {
	    EXTI->PR = irq_bit;
	    exti_call_handlers(irq);
	}
    }
}
