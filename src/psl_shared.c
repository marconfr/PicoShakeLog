/**
 * @file psl_shared.c
 * @brief Shared state implementation for inter-core synchronization
 *
 * This file implements the shared state management functions used for
 * coordination between Core 0 and Core 1. All state variables are protected
 * by hardware spin locks to ensure atomic access across cores.
 *
 * Shared state includes:
 * - Acquisition control flags (start/stop commands)
 * - Client connection status
 * - Heartbeat/watchdog timing
 * - Thread-safe getters and setters
 *
 * @author Frédéric Marçon
 * @date 2026
 * @copyright Academic Free License v3.0
 */

#include "psl_shared.h"
#include "psl_log.h"
#include "pico/time.h"
#include <stdio.h>

/* Global instance of shared state */
psl_shared_state_t g_shared_state = {0};

bool shared_state_init(psl_shared_state_t* state) {
    if (state == NULL) {
        return false;
    }

    state->acquisition_active = false;
    state->client_connected   = false;
    state->last_heartbeat_ms  = 0;
    state->state_lock         = spin_lock_instance(next_striped_spin_lock_num());

    if (state->state_lock == NULL) {
        LOG_E(INIT, "Shared state: failed to get spinlock");
        return false;
    }

    LOG_OK(INIT, "Shared state: lock ready");
    return true;
}

void shared_state_set_acquisition(psl_shared_state_t* state, bool active) {
    if (state == NULL || state->state_lock == NULL)
        return;

    uint32_t saved_irq        = spin_lock_blocking(state->state_lock);
    state->acquisition_active = active;
    spin_unlock(state->state_lock, saved_irq);
}

bool shared_state_get_acquisition(psl_shared_state_t* state) {
    if (state == NULL || state->state_lock == NULL)
        return false;

    uint32_t saved_irq = spin_lock_blocking(state->state_lock);
    bool     active    = state->acquisition_active;
    spin_unlock(state->state_lock, saved_irq);

    return active;
}

void shared_state_update_heartbeat(psl_shared_state_t* state) {
    if (state == NULL || state->state_lock == NULL)
        return;

    uint32_t saved_irq       = spin_lock_blocking(state->state_lock);
    state->last_heartbeat_ms = to_ms_since_boot(get_absolute_time());
    spin_unlock(state->state_lock, saved_irq);
}

bool shared_state_heartbeat_expired(psl_shared_state_t* state, uint32_t timeout_ms) {
    if (state == NULL || state->state_lock == NULL)
        return false;

    uint32_t saved_irq = spin_lock_blocking(state->state_lock);
    uint32_t now_ms    = to_ms_since_boot(get_absolute_time());
    uint32_t last_ms   = state->last_heartbeat_ms;
    spin_unlock(state->state_lock, saved_irq);

    return (now_ms - last_ms) > timeout_ms;
}
