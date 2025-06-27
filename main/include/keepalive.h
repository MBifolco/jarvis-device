#ifndef KEEPALIVE_H
#define KEEPALIVE_H

#include <stdbool.h>

// Initialize the keep-alive flag (call once in app_main)
void keepalive_init(void);

// Query the flag
bool keepalive_is_enabled(void);

// Enable (sets flag = true)
void keepalive_enable(void);

// Disable (sets flag = false)
void keepalive_disable(void);

#endif // KEEPALIVE_H
