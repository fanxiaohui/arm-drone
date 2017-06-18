#pragma once

#include "exti.h"
#include <stm32f0xx.h>

/*----------------------------------------------------------------------
 * Public type declarations
 *----------------------------------------------------------------------*/

struct button_st {
    unsigned int       pin;
    uint32_t	       irq_time;	// last time the state of the pin has changed
    exti_irq_handler_t irq_handler;
};

typedef struct button_st button_t;

#define NUM_BUTTONS	4

// button positions in the state field
#define	BUTTON_A	(1 << 0)
#define	BUTTON_B	(1 << 1)
#define	BUTTON_C	(1 << 2)
#define	BUTTON_D	(1 << 3)

#define	BUTTON_A_PIN	14
#define	BUTTON_B_PIN	10
#define	BUTTON_C_PIN	13
#define	BUTTON_D_PIN	4

struct buttons_st;

typedef struct buttons_st buttons_t;

typedef void (*button_cb_fn)(uint32_t changed, uint32_t state, void *client_data);

struct buttons_st {
    button_t	 pins[NUM_BUTTONS];
    uint32_t	 state;		   // 1 - button is pressed, 0 - button is released
    uint32_t	 pending_state;	   // last read state, which may not be stable
    button_cb_fn cb;
};

/*----------------------------------------------------------------------
 * Public function declarations
 *----------------------------------------------------------------------*/

extern void buttons_init(button_cb_fn cb, void *client_data);
