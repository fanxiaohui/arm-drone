#pragma once

extern void console_init();

/* Non-blocking console write. */
extern int console_write_nb(const char *buf, int size);

/* Wait until buffer space will be available. */
extern void console_write(const char *buf, int size);
