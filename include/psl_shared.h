/**
 * @file psl_shared.h
 * @brief Shared state management between Core 0 and Core 1
 *
 * This header defines the shared state structure used for synchronization
 * and coordination between the two cores of the RP2040. All shared variables
 * are volatile and protected by hardware spin locks to ensure thread-safe
 * access across cores.
 *
 * The shared state manages:
 * - Acquisition start/stop commands (Core 0 → Core 1)
 * - Client connection status (Core 0 → Core 1)
 * - Heartbeat/watchdog timing
 * - Thread-safe state transitions
 *
 * @author Frédéric Marçon
 * @date 2026
 * @copyright Academic Free License v3.0
 *
 * @see psl_core0.h for Core 0 control operations
 * @see psl_core1.h for Core 1 acquisition operations
 */

#ifndef PSL_SHARED_H
#define PSL_SHARED_H

#include "hardware/sync.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*=============================================================================
 * SHARED STATE BETWEEN CORES
 *===========================================================================*/

/**
 * @brief Shared state between Core 0 (communication) and Core 1 (acquisition)
 *
 * All variables are volatile and protected by a spin_lock.
 * Used to synchronize commands and monitor client connection.
 */
typedef struct {
    volatile bool     acquisition_active; /* Core0: Write, Core1: Read */
    volatile bool     client_connected;   /* Core0: Write, Core1: Read */
    volatile uint32_t last_heartbeat_ms;  /* Core0: Write, Watchdog: Read */

    spin_lock_t* state_lock; /* Protects all fields above */
} psl_shared_state_t;

/* Global instance of shared state */
extern psl_shared_state_t g_shared_state;

/*=============================================================================
 * SHARED STATE FUNCTIONS
 *===========================================================================*/

/**
 * @brief Initialize shared state between cores
 * @return true on success, false otherwise
 */
bool shared_state_init(psl_shared_state_t* state);

/**
 * @brief Set acquisition_active flag (thread-safe)
 * @param state Pointer to shared state
 * @param active true to start, false to stop
 */
void shared_state_set_acquisition(psl_shared_state_t* state, bool active);

/**
 * @brief Read acquisition_active flag (thread-safe)
 * @param state Pointer to shared state
 * @return true if acquisition active, false otherwise
 */
bool shared_state_get_acquisition(psl_shared_state_t* state);

/**
 * @brief Update heartbeat timestamp (called on each tcp_write)
 * @param state Pointer to shared state
 */
void shared_state_update_heartbeat(psl_shared_state_t* state);

/**
 * @brief Check if heartbeat has expired (> timeout_ms)
 * @param state Pointer to shared state
 * @param timeout_ms Timeout in milliseconds
 * @return true if expired, false otherwise
 */
bool shared_state_heartbeat_expired(psl_shared_state_t* state, uint32_t timeout_ms);

#ifdef __cplusplus
}
#endif

#endif /* PSL_SHARED_H */
