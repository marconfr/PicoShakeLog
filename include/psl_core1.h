/**
 * @file psl_core1.h
 * @brief Core 1 real-time IMU data acquisition interface for PicoShakeLog
 *
 * This header defines the API for Core 1 operations in the PicoShakeLog dual-core
 * architecture. Core 1 is dedicated to real-time IMU data acquisition using
 * DMA transfers triggered by hardware interrupts. It operates independently
 * from Core 0, communicating via FIFO for high-throughput data streaming.
 *
 * Core 1 responsibilities:
 * - Monitoring IMU interrupt pin (data ready signal)
 * - Triggering DMA transfers to read IMU burst data
 * - Processing DMA completion interrupts
 * - Pushing acquired data to inter-core FIFO
 * - Real-time operation without blocking
 *
 * @author Frédéric Marçon
 * @date 2026
 * @copyright Academic Free License v3.0
 *
 * @see psl_core0.h for Core 0 operations (network communication)
 * @see psl_fifo.h for inter-core communication
 * @see icm42688_driver.h for IMU hardware interface
 */

#ifndef PSL_CORE1_H
#define PSL_CORE1_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*=============================================================================
 * CORE 1 - REAL-TIME ACQUISITION
 *===========================================================================*/

/**
 * @brief Core 1 entry point
 *
 * This function is launched via multicore_launch_core1().
 * Infinite loop that:
 * - Waits for acquisition_active flag
 * - Processes DMA IRQs
 * - Pushes data to FIFO
 */
void core1_entry(void);

/**
 * @brief Start DMA to read an IMU burst
 *
 * Called from GPIO IRQ (IMU INT pin).
 */
void core1_start_dma_transfer(void);

#ifdef __cplusplus
}
#endif

#endif /* PSL_CORE1_H */
