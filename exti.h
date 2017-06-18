#pragma once

typedef void (*exti_irq_handler_fn)(unsigned int irqn, void *client_data);

struct exti_irq_handler_st;
typedef volatile struct exti_irq_handler_st exti_irq_handler_t;

// single-linked non-circular list of EXTI interrupt callbacks
struct exti_irq_handler_st {
    exti_irq_handler_t	*next;
    exti_irq_handler_fn irq_handler_fn;
    void		*client_data;
};

extern void exti_init();
extern void exti_irq_handler_init(exti_irq_handler_t *handler,
				  exti_irq_handler_fn fn, void *client_data);
extern void exti_register(exti_irq_handler_t *handler, unsigned int irqn);

