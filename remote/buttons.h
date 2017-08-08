#pragma once

#include <stdint.h>

/*----------------------------------------------------------------------
 * Public type declarations
 *----------------------------------------------------------------------*/

// button positions in the state field
#define	BUTTON_A	(1 << 0)
#define	BUTTON_B	(1 << 1)
#define	BUTTON_C	(1 << 2)
#define	BUTTON_D	(1 << 3)

typedef void (*button_cb_fn)(uint16_t changed, uint16_t state, void *client_data);

/*----------------------------------------------------------------------
 * Public function declarations
 *----------------------------------------------------------------------*/

extern void buttons_init(button_cb_fn cb, void *client_data);
