/**
 * @file psl_fifo.h
 * @brief Thread-safe circular FIFO for inter-core communication
 *
 * This header defines a lock-free circular FIFO buffer used for high-performance
 * data exchange between Core 1 (producer - IMU acquisition) and Core 0 (consumer -
 * TCP transmission). The FIFO uses spin locks for IRQ-safe synchronization and
 * provides zero-copy access to data slots.
 *
 * Features:
 * - Fixed-size circular buffer with configurable slots
 * - IRQ-safe operations using hardware spin locks
 * - Zero-copy write/read interface
 * - Thread-safe between cores
 * - Overflow detection and handling
 *
 * @author Frédéric Marçon
 * @date 2026
 * @copyright Academic Free License v3.0
 *
 * @see psl_core0.h for consumer operations
 * @see psl_core1.h for producer operations
 */

#ifndef PSL_FIFO_H
#define PSL_FIFO_H

#include "hardware/sync.h"
#include "psl_config.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/*=============================================================================
 * INTER-CORE CIRCULAR FIFO
 *===========================================================================*/

/**
 * @brief Thread-safe circular FIFO structure for inter-core exchange
 *
 * Contiguous memory pool divided into FIFO_SLOTS buffers.
 * Used to decouple acquisition (Core 1) from TCP transmission (Core 0).
 */
typedef struct {
    uint8_t* fifo_pool; /* Contiguous pool of FIFO_SLOTS buffers */
    size_t   slot_size; /* Slot size (bytes) */

    volatile uint32_t write_idx; /* Write index (0 to FIFO_SLOTS-1) */
    volatile uint32_t read_idx;  /* Read index (0 to FIFO_SLOTS-1) */
    volatile uint32_t count;     /* Number of full slots (0 to FIFO_SLOTS) */

    spin_lock_t* lock; /* IRQ-safe protection */
} psl_fifo_t;

/* Global instance of FIFO */
extern psl_fifo_t g_fifo;

/*=============================================================================
 * FIFO FUNCTIONS
 *===========================================================================*/

/**
 * @brief Initialize FIFO with specified slot size
 * @param fifo Pointer to FIFO structure
 * @param slot_size Slot size in bytes
 * @return true on success, false otherwise
 */
bool fifo_init(psl_fifo_t* fifo, size_t slot_size);

/**
 * @brief Free FIFO memory
 * @param fifo Pointer to FIFO structure
 */
void fifo_free(psl_fifo_t* fifo);

/**
 * @brief Write data to FIFO (thread-safe)
 * @param fifo Pointer to FIFO structure
 * @param data Pointer to data to write
 * @return true on success, false if FIFO full
 * @note Called from Core 1 (DMA IRQ)
 */
bool fifo_push(psl_fifo_t* fifo, const uint8_t* data);

/**
 * @brief Read and remove data from FIFO (thread-safe)
 * @param fifo Pointer to FIFO structure
 * @param dest Destination buffer (must have at least slot_size bytes)
 * @return true on success, false if FIFO empty
 * @note Called from Core 0 (TCP loop)
 */
bool fifo_pop(psl_fifo_t* fifo, uint8_t* dest);

/**
 * @brief Read data from FIFO without removing (thread-safe)
 * @param fifo Pointer to FIFO structure
 * @param dest Destination buffer
 * @return true on success, false if FIFO empty
 */
bool fifo_peek(psl_fifo_t* fifo, uint8_t* dest);

/**
 * @brief Remove the oldest slot without copying it (thread-safe)
 *
 * Use after fifo_peek() once the slot has been handed over successfully (e.g. tcp_write OK).
 * @param fifo Pointer to FIFO structure
 * @return true if a slot was removed, false if FIFO empty
 */
bool fifo_drop(psl_fifo_t* fifo);

/**
 * @brief Check if FIFO is full (thread-safe)
 * @param fifo Pointer to FIFO structure
 * @return true if full, false otherwise
 */
bool fifo_is_full(psl_fifo_t* fifo);

/**
 * @brief Check if FIFO is empty (thread-safe)
 * @param fifo Pointer to FIFO structure
 * @return true if empty, false otherwise
 */
bool fifo_is_empty(psl_fifo_t* fifo);

/**
 * @brief Return number of available slots (thread-safe)
 * @param fifo Pointer to FIFO structure
 * @return Number of free slots
 */
uint32_t fifo_available_slots(psl_fifo_t* fifo);

/**
 * @brief Completely empty FIFO (thread-safe)
 * @param fifo Pointer to FIFO structure
 */
void fifo_reset(psl_fifo_t* fifo);

#ifdef __cplusplus
}
#endif

#endif /* PSL_FIFO_H */
