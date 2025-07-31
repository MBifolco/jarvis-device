#include "keepalive.h"

static bool s_enabled = false;

void keepalive_init(void) {
    s_enabled = false;
}

bool keepalive_is_enabled(void) {
    return s_enabled;
}

void keepalive_enable(void) {
    s_enabled = true;
}

void keepalive_disable(void) {
    s_enabled = false;
}
