
#include "buttons.h"
#include "utils.h"
#include "scheduler.h"

#include <stm32f0xx_ll_gpio.h>
#include <stdbool.h>

static buttons_t buttons;
static exti_irq_handler_t button_irq_handlers[NUM_BUTTONS];

static void button_irq_handler(unsigned int irqn, void *button_pin);

void buttons_init(button_cb_fn cb, void *client_data)
{
    buttons.pins[0].pin = BUTTON_A_PIN;
    buttons.pins[1].pin = BUTTON_B_PIN;
    buttons.pins[2].pin = BUTTON_C_PIN;
    buttons.pins[3].pin = BUTTON_D_PIN;

    for (int i = 0; i < NUM_BUTTONS; ++i) {
	// all buttons are connected to GPIOA
	gpio_set_mode(GPIOA, buttons.pins[i].pin, LL_GPIO_MODE_INPUT);

	// enable external interrupt for the current pin, client data is pin index
	exti_irq_handler_init(&buttons.pins[i].irq_handler, &button_irq_handler, (void *) i);

	// register IRQ callbacks for the interrupt line,
	// note that interrupt lines match PIN numbers
	exti_register(&buttons.pins[i].irq_handler, buttons.pins[i].pin);

	// enable interrupt line in EXTI, both rising and falling triggers
	EXTI->IMR |= 1 << buttons.pins[i].pin;
	EXTI->RTSR |= 1 << buttons.pins[i].pin;
	EXTI->FTSR |= 1 << buttons.pins[i].pin;
    }
}

static void button_irq_handler(unsigned int irqn, void *client_data)
{
    unsigned int pin = (unsigned int) client_data;
    unsigned int pin_mask = 1 << pin;
    
    uint32_t now = sched_now();
    bool current_state = GPIOA->ODR & pin_mask;
    if (current_state) {
	buttons.pending_state |= pin_mask;
    } else {
	buttons.pending_state &= ~pin_mask;
    }

    
    
    // TBD
}
