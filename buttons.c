
#include "buttons.h"
#include "exti.h"
#include "scheduler.h"

#include <stm32f0xx_ll_gpio.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>

// microseconds for which the button state must be stable before change is accepted
#define BUTTON_DEBOUNCE_DELAY	20000

#define NUM_BUTTONS	4

#define	BUTTON_A_PIN	3
#define	BUTTON_B_PIN	10
#define	BUTTON_C_PIN	2
#define	BUTTON_D_PIN	4

struct button_st {
    unsigned int       pin;
    exti_irq_handler_t irq_handler;	// runs when interrupt is raised on button pin
    tim_task_t	       commit_state;    // runs after a period of no interrupt
};

typedef struct button_st button_t;

struct buttons_st {
    button_t	 pins[NUM_BUTTONS];
    uint16_t	 state;		   // 1 - button is pressed, 0 - button is released
    uint16_t	 pending_state;	   // last read state, which may not be stable
    button_cb_fn cb;
    void	 *client_data;
};

typedef struct buttons_st buttons_t;

static volatile buttons_t buttons = {
    .pins = { { .pin = BUTTON_A_PIN },
              { .pin = BUTTON_B_PIN },
	      { .pin = BUTTON_C_PIN },
	      { .pin = BUTTON_D_PIN }
    }
};
	      
static void button_irq_handler(unsigned int irqn, void *button_pin);
static void button_commit_state(tim_task_t *task, state_t prev_state, uint32_t expiry_time);

void buttons_init(button_cb_fn cb, void *client_data)
{
    buttons.cb = cb;
    buttons.client_data = client_data;
    
    for (int i = 0; i < NUM_BUTTONS; ++i) {
	// all buttons are connected to GPIOA
	gpio_set_mode(GPIOA, buttons.pins[i].pin, LL_GPIO_MODE_INPUT);

	// enable external interrupt for the current pin, client data is pin index
	exti_irq_handler_init(&buttons.pins[i].irq_handler, &button_irq_handler, (void *) i);

	// register IRQ callbacks for the interrupt line,
	// note that interrupt lines match PIN numbers
	exti_register(&buttons.pins[i].irq_handler, buttons.pins[i].pin);

        // initialise timer task to check if button state change is stable
        sched_timer_init(&buttons.pins[i].commit_state, &button_commit_state, (void *) i);
        
	// enable interrupt line in EXTI, both rising and falling triggers
	EXTI->IMR |= 1 << buttons.pins[i].pin;
	EXTI->RTSR |= 1 << buttons.pins[i].pin;
	EXTI->FTSR |= 1 << buttons.pins[i].pin;
    }
}

static void button_irq_handler(unsigned int irqn, void *client_data)
{
    unsigned int pin = (unsigned int) client_data;
    uint16_t pin_state_mask = 1 << pin;		   // pin position in state fields
    uint32_t pin_io_mask = 1 << buttons.pins[pin].pin; // pin position in GPIO register
    
    // note that button is pressed when we read low
    if (GPIOA->IDR & pin_io_mask) {
        buttons.pending_state &= ~pin_state_mask;
    } else {
        buttons.pending_state |= pin_state_mask;
    }
    // (re-)schedule timer to see if the state change is stable only if the pending state
    // is different from the current state
    if ((buttons.pending_state & pin_state_mask) != (buttons.state & pin_state_mask)) {
        sched_timer_schedule_rel(&buttons.pins[pin].commit_state, BUTTON_DEBOUNCE_DELAY, 0);
    } else {
        // we may have a timer pending
        sched_timer_cancel(&buttons.pins[pin].commit_state);
    }

    printf("button: %d, pending state: %d\n", pin, !!(buttons.pending_state & pin_state_mask));
}

static void button_commit_state(tim_task_t *task, state_t prev_state, uint32_t expiry_time)
{
    // task may have been cancelled
    if (prev_state != TASK_SCHEDULED)
        return;
    
    unsigned int pin = (unsigned int) task->client_data;
    uint16_t pin_state_mask = 1 << pin;  // pin position in state fields

    // we have not had an interrupt for a while, commit the state change
    if (buttons.pending_state & pin_state_mask) {
        buttons.state |= pin_state_mask;
    } else {
        buttons.state &= ~pin_state_mask;
    }
    printf("button: %d, new state: %d\n", pin, !!(buttons.state & pin_state_mask));
    (*buttons.cb)(pin_state_mask, buttons.state, buttons.client_data);
}
