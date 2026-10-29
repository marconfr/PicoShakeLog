/**
 * @file psl_core0.h
 * @brief Core 0 WiFi/TCP communication and control interface for PicoShakeLog
 *
 * This header file defines the public API for Core 0 operations in the PicoShakeLog
 * dual-core architecture. Core 0 is responsible for handling network communication
 * via WiFi SoftAP and TCP server, managing client connections, and coordinating
 * with Core 1 through a FIFO-based inter-core communication system.
 *
 * Core 0 manages:
 * - WiFi SoftAP initialization and management
 * - TCP server setup and client connection handling
 * - Network data reception and transmission
 * - Main control loop and system coordination
 *
 * @author Frédéric Marçon
 * @date 2026
 * @copyright Academic Free License v3.0
 *
 * @see psl_core1.h for Core 1 operations (IMU data acquisition)
 * @see psl_fifo.h for inter-core communication
 */

#ifndef PSL_CORE0_H
#define PSL_CORE0_H

#include "psl_api.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/*=============================================================================
 * CORE 0 - COMMUNICATION & CONTROL
 *===========================================================================*/

/**
 * @brief Initialize WiFi SoftAP and TCP server on Core 0
 *
 * Sets up a WiFi Software Access Point (SoftAP) and initializes a TCP server
 * for network communication. This function configures the necessary WiFi parameters
 * and establishes the TCP listening socket for incoming client connections.
 *
 * @param[in,out] state_handler Pointer to the global state handler structure
 *                              that contains WiFi, TCP, and system state information.
 *                              Must not be NULL.
 *
 * @return true if WiFi SoftAP and TCP server initialization succeeds
 * @return false if initialization fails (WiFi setup error, TCP socket creation error,
 *               or invalid state_handler pointer)
 *
 * @pre state_handler must be initialized and valid
 * @pre Core 0 must be available for network operations
 *
 * @post On success: WiFi SoftAP is active and TCP server is listening
 * @post On failure: state_handler error flags may be set
 *
 * @see psl_state_handler_t
 * @see core0_cleanup_wifi_tcp()
 *
 * @note This function should be called during system initialization
 * @note WiFi credentials should be configured before calling this function
 */
bool core0_init_wifi_tcp(psl_state_handler_t* state_handler);

/**
 * @brief Main loop function for core 0 of the Raspberry Pi Pico.
 *
 * This function implements the primary execution loop for core 0 of the dual-core
 * Raspberry Pi Pico microcontroller. It handles the main processing tasks and
 * control flow for core 0's operations.
 *
 * @note This function typically runs indefinitely and should not return during
 *       normal operation.
 *
 * @see core_launch_core1() for launching the core 1 main loop
 */
void core0_main_loop(void);

/**
 * @brief Send data to TCP client (if connected)
 *
 * Transmits a buffer of data to the currently connected TCP client.
 * This function checks if a client is connected before attempting to send.
 * Useful for streaming IMU data or sending system status information.
 *
 * @param[in] data Pointer to the data buffer to transmit.
 *                 Must not be NULL if length > 0.
 * @param[in] length Size of the data buffer in bytes.
 *                   Must be greater than 0.
 *
 * @return true if data was successfully sent to the TCP client
 * @return false if no client is connected, data pointer is NULL,
 *               length is 0, or transmission failed
 *
 * @pre A TCP client must be connected for successful transmission
 * @pre data must point to a valid buffer of at least length bytes
 *
 * @warning This function does not guarantee delivery; use appropriate
 *          error handling and retry logic in higher-level protocols
 *
 * @see core0_init_wifi_tcp() for TCP connection setup
 * @see core0_cleanup() for connection teardown
 */
bool core0_tcp_send(const uint8_t* data, size_t length);

/**
 * @brief Close TCP connection and cleanup WiFi resources
 *
 * Performs a graceful shutdown of the TCP server and WiFi SoftAP.
 * This function closes any active client connections, deallocates network
 * buffers, and disables the WiFi interface. Should be called during
 * system shutdown or when reconfiguring network settings.
 *
 * @post TCP connections are closed
 * @post WiFi SoftAP is disabled
 * @post Network resources are freed
 *
 * @note This function should be called before system reset or power-down
 * @note After calling this function, call core0_init_wifi_tcp() to
 *       re-establish network connectivity
 *
 * @see core0_init_wifi_tcp() for re-initialization
 */
void core0_cleanup(void);

#ifdef __cplusplus
}
#endif

#endif /* PSL_CORE0_H */
