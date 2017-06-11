#pragma once

#include <stm32f0xx.h>

#include "utils.h"

extern void console_init();

/** Return the number of characters written, might be less than requested. */
extern int console_write(const char *buf, int size);
