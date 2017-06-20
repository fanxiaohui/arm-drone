
#include "buttons.h"
#include "utils.h"
#include "scheduler.h"

#include <stm32f0xx_ll_gpio.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>

// microseconds for which the button state must be stable before change is accepted
#define BUTTON_DEBOUNCE_DELAY	20000

#define	BUTTON_A_PIN	14
#define	BUTTON_B_PIN	10
#define	BUTTON_C_PIN	13
#define	BUTTON_D_PIN	4

static buttons_t buttons = {
    .pins = { { .pin = BUTTON_A_PIN },
	      { .pin = BUTTON_B_PIN },
	      { .pin = BUTTON_C_PIN },
	      { .pin = BUTTON_D_PIN }
    }
};
	      
static exti_irq_handler_t button_irq_handlers[NUM_BUTTONS];

static void button_irq_handler(unsigned int irqn, void *button_pin);

void buttons_init(button_cb_fn cb, void *client_data)
{
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
    unsigned int pin_state_mask = 1 << pin;		   // pin position in state fields
    unsigned int pin_io_mask = 1 << buttons.pins[pin].pin; // pin position in GPIO register
    
    // button is pressed when we read low
    unsigned int pin_state = GPIOA->ODR & pin_io_mask ? 0 : pin_state_mask;
    buttons.pending_state |= pin_state;
    buttons.pending_state &= ~pin_state;

    uint32_t now = sched_now();
    // handle the case when the button was not pressed for a long time and the timer
    // wrapper around
    if (!sched_time_lte(buttons.pins[pin].irq_time, now)
	|| sched_time_lte(buttons.pins[pin].irq_time + BUTTON_DEBOUNCE_DELAY, now)) {
	if ((buttons.state & pin_state_mask) != pin_state) {
	    buttons.state |= pin_state;
	    buttons.state &= ~pin_state;
	    printf("button: %d, new state: %d\n", pin, !!pin_state);
	}
    } else {
	printf("button: %d, new state: %d, ignored as too quick: %lu micros\n",
	       pin, !!pin_state, now - buttons.pins[pin].irq_time);
    }
    
    buttons.pins[pin].irq_time = now;
}
