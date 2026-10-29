/**
 * @file psl_api.h
 * @brief Main API and state management for PicoShakeLog system
 *
 * This header defines the core data structures, state machine, and protocol
 * for the PicoShakeLog IMU acquisition system. It includes TCP protocol definitions,
 * error handling, state management, and the main initialization functions.
 *
 * The state machine coordinates between:
 * - System initialization (WiFi, IMU, DMA)
 * - Client connection waiting
 * - Data acquisition and streaming
 * - Error handling and recovery
 *
 * @author Frédéric Marçon
 * @date 2026
 * @copyright Academic Free License v3.0
 *
 * @see psl_core0.h for Core 0 operations (WiFi/TCP)
 * @see psl_core1.h for Core 1 operations (IMU acquisition)
 */

#ifndef PSL_API_H
#define PSL_API_H

#include "hardware/dma.h"
#include "hardware/spi.h"
#include "hardware/sync.h"
#include "icm42688_driver.h"
#include "psl_config.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*=============================================================================
 * TCP PROTOCOL (single-byte ASCII)
 *===========================================================================*/

/* Incoming commands */
#define CMD_START  'S' /* 0x53: Start acquisition */
#define CMD_STOP   'Q' /* 0x54: Stop acquisition */
#define CMD_STATUS '?' /* 0x3F: Request status */

/* Outgoing responses */
#define RESP_ACK     'A' /* 0x41: Command accepted */
#define RESP_NAK     'N' /* 0x4E: Command rejected */
#define RESP_RUNNING 'R' /* 0x52: RUNNING state */
#define RESP_IDLE    'I' /* 0x49: IDLE state */

/*=============================================================================
 * STATE MACHINE
 *===========================================================================*/

typedef enum {
    ERROR_NONE           = 1 << 0,
    ERROR_ICM42688_INIT  = 1 << 1,
    ERROR_ICM42688_COMM  = 1 << 2,
    ERROR_DMA_CONFIG     = 1 << 3,
    ERROR_BUFFER_ALLOC   = 1 << 4,
    ERROR_WIFI_INIT      = 1 << 5,
    ERROR_CLIENT_LOST    = 1 << 6,
    ERROR_BUFFER_OVERRUN = 1 << 7
} psl_error_flags_t;

typedef enum {
    STATE_START = 0,
    STATE_INIT,
    STATE_WAIT_FOR_CLIENT,
    STATE_IDLE,
    STATE_RUNNING,
    STATE_ERROR
} psl_state_t;

/**
 * @brief Handler structure for managing the state of the PicoShakeLog system.
 *
 * This structure tracks the current and previous states of the system,
 * as well as the readiness and configuration status of various modules.
 * It also maintains cumulative error flags and provides an optional callback
 * for state change events.
 *
 * @typedef psl_state_handler_t
 *
 * @param current_state        Current operational state of the PicoShakeLog system.
 * @param previous_state       Previous operational state of the PicoShakeLog system.
 * @param modules_states       Structure containing readiness/configuration flags for system
 * modules:
 *                            - icm42688_ready: IMU initialized and responding correctly.
 *                            - dma_configured: DMA channel allocated and configured.
 *                            - buffers_allocated: Ping-pong buffers allocated.
 *                            - client_connected: WiFi client connected and ready.
 * @param error_flags          Cumulative error flags for the system.
 * @param on_state_change      Optional callback function invoked on state change.
 */
typedef struct {
    psl_state_t current_state;
    psl_state_t previous_state;

    struct {
        bool icm42688_ready;    /* IMU initialized and responding correctly */
        bool dma_configured;    /* DMA channel allocated and configured */
        bool buffers_allocated; /* Ping-pong buffers allocated */
        bool client_connected;  /* WiFi client connected and ready */
    } modules_states;

    uint32_t error_flags; /* Cumulative error flags */

    void (*on_state_change)(psl_state_t old_state, psl_state_t new_state);
} psl_state_handler_t;

/**
 * @struct psl_dma_buffers_t
 * @brief Structure for managing DMA buffers in a ping-pong configuration.
 *
 * This structure holds pointers to two DMA buffers (ping and pong), their size,
 * and the currently active buffer. It also includes a spin lock to ensure
 * thread-safe access to the buffers, suitable for use in interrupt contexts.
 *
 * @var buffer_ping Pointer to the ping DMA buffer.
 * @var buffer_pong Pointer to the pong DMA buffer.
 * @var buffer_size Size of each DMA buffer in bytes.
 * @var current_buffer Pointer to the buffer currently in use.
 * @var buffer_lock Spin lock to protect buffer access (IRQ safe).
 */
typedef struct {
    uint8_t*     buffer_ping;
    uint8_t*     buffer_pong;
    size_t       buffer_size;
    uint8_t*     current_buffer; /* pointer to the buffer currently in use */
    spin_lock_t* buffer_lock;    /* lock to protect the buffer (IRQ safe) */
} psl_dma_buffers_t;

/*=============================================================================
 * GLOBAL VARIABLES
 *===========================================================================*/

extern psl_state_handler_t      pv_state_handler;
extern icm42688_connection_handler_t icm42688_device;
extern icm42688_config_t             icm42688_cfg;
extern psl_dma_buffers_t        dma_buffers;

/*=============================================================================
 * PUBLIC API
 *===========================================================================*/

/**
 * @brief Transition to a new state in the state machine.
 *
 * Checks that the transition is valid (entry conditions), then
 * performs the change if allowed. Calls on_state_change callback if defined.
 *
 * @param handler State machine context
 * @param new_state Target new state
 * @return true if transition succeeded, false otherwise
 */
bool psl_set_state(psl_state_handler_t* handler, psl_state_t new_state);

/**
 * @brief Returns the printable name of a state (e.g. "WAIT_FOR_CLIENT") for logging.
 */
const char* psl_state_name(psl_state_t state);

/**
 * @brief Initialize all subsystems (SPI, IMU, DMA, WiFi, buffers).
 *
 * Complete initialization sequence:
 * - SPI0 configuration
 * - ICM42688 detection and configuration
 * - DMA ping-pong buffers allocation
 * - Inter-core FIFO initialization
 * - Shared state initialization
 * - WiFi SoftAP and TCP server startup
 *
 * @param handler State machine context
 * @return true if all subsystems are OK, false on error
 */
bool init_system_dev(psl_state_handler_t* handler);

#ifdef __cplusplus
}
#endif

#endif /* PSL_API_H */
