
#include "exti.h"
#include "scheduler.h"

#include <assert.h>
#include <string.h>

#define EXTI_LINES_NUM	16

// list of irq handler registrations for each IRQ line
static exti_irq_handler_t *volatile exti_irq_handlers[EXTI_LINES_NUM];

INLINE void exti_check_handlers(unsigned int irqn);

void exti_init()
{
    // enable all interrupt handlers, individual lines will be enabled
    // by the user
    NVIC_SetPriority(EXTI0_IRQn, SCHED_TASK_PRIORITY);
    NVIC_SetPriority(EXTI1_IRQn, SCHED_TASK_PRIORITY);
    NVIC_SetPriority(EXTI2_IRQn, SCHED_TASK_PRIORITY);
    NVIC_SetPriority(EXTI3_IRQn, SCHED_TASK_PRIORITY);
    NVIC_SetPriority(EXTI4_IRQn, SCHED_TASK_PRIORITY);
    NVIC_SetPriority(EXTI9_5_IRQn, SCHED_TASK_PRIORITY);
    NVIC_SetPriority(EXTI15_10_IRQn, SCHED_TASK_PRIORITY);

    NVIC_EnableIRQ(EXTI0_IRQn);
    NVIC_EnableIRQ(EXTI1_IRQn);
    NVIC_EnableIRQ(EXTI2_IRQn);
    NVIC_EnableIRQ(EXTI3_IRQn);
    NVIC_EnableIRQ(EXTI4_IRQn);
    NVIC_EnableIRQ(EXTI9_5_IRQn);
    NVIC_EnableIRQ(EXTI15_10_IRQn);
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

INLINE void exti_check_handlers(unsigned int irqn)
{
    unsigned int pending_flag = 1 << irqn;
    if (!(EXTI->PR & pending_flag))
        return;
    
    EXTI->PR = pending_flag;
    
    for (exti_irq_handler_t *handler = exti_irq_handlers[irqn];
	 handler;
	 handler = handler->next) {
	(*handler->irq_handler_fn)(irqn, handler->client_data);
    }
}

void EXTI0_IRQHandler()
{
    exti_check_handlers(0);
}

void EXTI1_IRQHandler()
{
    exti_check_handlers(1);
}

void EXTI2_IRQHandler()
{
    exti_check_handlers(2);
}

void EXTI3_IRQHandler()
{
    exti_check_handlers(3);
}

void EXTI4_IRQHandler()
{
    exti_check_handlers(4);
}

void EXTI9_5_IRQHandler()
{
    for (unsigned int irq = 5; irq <= 9; ++irq) {
        exti_check_handlers(irq);
    }
}

void EXTI15_10_IRQHandler()
{
    for (unsigned int irq = 10; irq <= 15; ++irq) {
        exti_check_handlers(irq);
    }
}
