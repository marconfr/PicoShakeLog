/**
 * @file psl_api.c
 * @brief Implementation of core API and state management functions
 *
 * This file implements the main initialization routines, state machine logic,
 * and global variable definitions for the PicoShakeLog system. It coordinates
 * between all subsystems including IMU, DMA, WiFi, and inter-core communication.
 *
 * Key responsibilities:
 * - System-wide initialization (init_system_dev)
 * - IMU and DMA configuration
 * - State handler management
 * - Global resource allocation
 *
 * @author Frédéric Marçon
 * @date 2026
 * @copyright Academic Free License v3.0
 */

#include "psl_api.h"
#include "psl_core0.h"
#include "psl_fifo.h"
#include "psl_log.h"
#include "psl_shared.h"

#include "hardware/dma.h"
#include "hardware/spi.h"
#include "icm42688_driver.h"

#include <stdio.h>
#include <stdlib.h>

/*=============================================================================
 * GLOBAL VARIABLES
 *===========================================================================*/

icm42688_connection_handler_t icm42688_device;

icm42688_config_t icm42688_cfg = ICM42688_IMU_CONFIG_TRANSPORT;

psl_dma_buffers_t dma_buffers = {0};

psl_state_handler_t pv_state_handler = {
    .previous_state = STATE_START,
    .current_state  = STATE_INIT,
    .modules_states =
        {
            .icm42688_ready    = false,
            .dma_configured    = false,
            .buffers_allocated = false,
            .client_connected  = false,
        },
    .error_flags = ERROR_NONE,
};

/*=============================================================================
 * STATE MACHINE METHODS
 *===========================================================================*/

/**
 * @brief Checks if the system can enter idle state.
 *
 * This function evaluates whether all required modules are ready for the system to safely enter an
 * idle state. It checks the readiness of the ICM42688 sensor, DMA configuration, and buffer
 * allocation. This is an awesome function, isn't it ?
 *
 * @param handler Pointer to the psl_state_handler_t structure containing the current module
 * states.
 * @return true if all required modules are ready and the system can enter idle; false otherwise.
 */
static bool can_enter_idle(psl_state_handler_t* handler) {
    return handler->modules_states.icm42688_ready && handler->modules_states.dma_configured &&
           handler->modules_states.buffers_allocated;
}

/**
 * @brief Determines if the system can transition to the running state.
 *
 * This function checks whether the system is eligible to enter the running state
 * by verifying two conditions:
 *   1. The system can enter the idle state (via can_enter_idle()).
 *   2. The client is currently connected (handler->modules_states.client_connected).
 * This is also an awesome function, isn't it ?
 * @param handler Pointer to the PSL state handler structure.
 * @return true if the system can enter the running state, false otherwise.
 */
static bool can_enter_running(psl_state_handler_t* handler) {
    return can_enter_idle(handler) && handler->modules_states.client_connected;
}

/**
 * @brief Resets the state handler structure to its initial state.
 *
 * This function initializes the given psl_state_handler_t structure by resetting
 * all error flags, state variables, and module state flags to their default values.
 * It ensures the handler is ready for a fresh start or reinitialization.
 *
 * @param handler Pointer to the psl_state_handler_t structure to reset.
 * @return true if the handler was successfully reset; false if the handler pointer is NULL.
 */
static bool reset_state_handlers(psl_state_handler_t* handler) {
    if (handler == NULL) {
        LOG_E(STATE, "State handler is NULL");
        return false;
    }

    handler->error_flags                      = ERROR_NONE;
    handler->previous_state                   = STATE_START;
    handler->current_state                    = STATE_START;
    handler->modules_states.icm42688_ready    = false;
    handler->modules_states.dma_configured    = false;
    handler->modules_states.buffers_allocated = false;
    handler->modules_states.client_connected  = false;

    return true;
}

const char* psl_state_name(psl_state_t state) {
    switch (state) {
    case STATE_START:
        return "START";
    case STATE_INIT:
        return "INIT";
    case STATE_WAIT_FOR_CLIENT:
        return "WAIT_FOR_CLIENT";
    case STATE_IDLE:
        return "IDLE";
    case STATE_RUNNING:
        return "RUNNING";
    case STATE_ERROR:
        return "ERROR";
    }
    return "UNKNOWN";
}

bool psl_set_state(psl_state_handler_t* handler, psl_state_t new_state) {
    if (handler == NULL) {
        LOG_E(STATE, "State handler is NULL");
        return false;
    }

    psl_state_t old_state = handler->current_state;

    switch (new_state) {
    case STATE_START:
        break;

    case STATE_IDLE:
        if (!can_enter_idle(handler)) {
            LOG_W(STATE, "Cannot enter IDLE: modules not ready");
            psl_set_state(handler, STATE_ERROR);
            return false;
        }
        break;

    case STATE_RUNNING:
        if (!can_enter_running(handler)) {
            LOG_W(STATE, "Cannot enter RUNNING: client not connected");
            return false;
        }
        break;

    case STATE_WAIT_FOR_CLIENT:
        if (!can_enter_idle(handler)) {
            LOG_W(STATE, "Cannot wait for client: system not ready");
            psl_set_state(handler, STATE_ERROR);
            return false;
        }
        break;

    case STATE_ERROR:
        break;

    case STATE_INIT:
        if (old_state != STATE_ERROR && old_state != STATE_INIT) {
            LOG_W(STATE, "Cannot reinit from state %s", psl_state_name(old_state));
            return false;
        }
        break;
    }

    handler->previous_state = old_state;
    handler->current_state  = new_state;

    LOG_I(STATE, "%s -> %s", psl_state_name(old_state), psl_state_name(new_state));

    if (handler->on_state_change != NULL) {
        handler->on_state_change(old_state, new_state);
    }

    return true;
}

/*=============================================================================
 * SPI & DMA INITIALIZATION
 *===========================================================================*/

/**
 * @brief Initializes the SPI0 peripheral and associated GPIOs for communication with the ICM42688
 * sensor.
 *
 * This function configures the SPI0 interface with the appropriate baud rate, data format,
 * and pin multiplexing required to communicate with the ICM42688 IMU sensor. It also sets up
 * the chip select (CS) and interrupt (INT1) pins. By abstracting the initialization into a single
 * function, it simplifies sensor setup and ensures consistent configuration, making your code
 * cleaner and less error-prone.
 *
 * @note This function must be called before attempting to communicate with the ICM42688 sensor.
 * @see psl_config.h for pin definitions and SPI settings.
 */
static void init_spi0_for_icm42688(void) {
    spi_init(SPI_PORT, SPI_BAUDRATE);
    gpio_set_function(SPI_MISO, GPIO_FUNC_SPI);
    gpio_set_function(SPI_MOSI, GPIO_FUNC_SPI);
    gpio_set_function(SPI_SCK, GPIO_FUNC_SPI);
    gpio_init(SPI_CS);
    gpio_set_dir(SPI_CS, GPIO_OUT);

    spi_set_format(SPI_PORT, 8, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);

    gpio_init(SPI_INT1);
    gpio_set_dir(SPI_INT1, GPIO_IN);
}

/* Single dummy byte sent repeatedly by TX DMA to generate SPI clock */
/* Dummy byte for SPI TX DMA (must be in SRAM, not flash/const) */
static uint8_t spi_dummy_byte = 0x00;

/**
 * @brief Initializes and claims the SPI DMA channels for exclusive use.
 *
 * This function claims the RX and TX DMA channels to prevent interference from other peripherals
 * (such as the CYW43 WiFi DMA). It ensures both channels are fully stopped, clears any pending
 * IRQs, and disables IRQs for both channels. Note that DREQ configuration is intentionally omitted
 * here to avoid unwanted IRQ storms; full DMA configuration is performed later in
 * `core1_start_dma_transfer()`.
 *
 * @note This function should be called during system initialization before starting any DMA
 * transfers.
 */
static void init_spi_dma(void) {
    /* Claim channels exclusively to prevent interference (e.g. from CYW43 WiFi DMA) */
    dma_channel_claim(SPI_DMA_CHANNEL);
    dma_channel_claim(SPI_DMA_TX_CHANNEL);

    /* Ensure both channels are fully stopped and IRQs cleared.
     * Do NOT configure DREQ here: an armed DREQ with count=0 causes the channel
     * to complete instantly on first DREQ signal → IRQ storm in free-run.
     * Full configuration is done in core1_start_dma_transfer() at each burst. */
    dma_channel_abort(SPI_DMA_CHANNEL);
    dma_channel_abort(SPI_DMA_TX_CHANNEL);
    dma_channel_acknowledge_irq0(SPI_DMA_CHANNEL);
    dma_channel_acknowledge_irq0(SPI_DMA_TX_CHANNEL);
    dma_channel_set_irq0_enabled(SPI_DMA_CHANNEL, false);
    dma_channel_set_irq0_enabled(SPI_DMA_TX_CHANNEL, false);

    LOG_I(DMA, "Channels %d (RX) and %d (TX) claimed", SPI_DMA_CHANNEL, SPI_DMA_TX_CHANNEL);
}

/*=============================================================================
 * DMA BUFFERS MANAGEMENT
 *===========================================================================*/

/**
 * @brief Initializes DMA buffers and associated spinlock for the PicoShakeLog device.
 *
 * This function allocates two aligned DMA buffers (ping and pong) of the required size
 * for the ICM42688 device, and initializes a spinlock for buffer access synchronization.
 * The buffer size is determined by querying the ICM42688 device.
 *
 * @param[out] buffers Pointer to a psl_dma_buffers_t structure that will be initialized.
 * @return true if buffers and spinlock were successfully allocated and initialized, false
 * otherwise.
 */
static bool init_dma_buffers(psl_dma_buffers_t* buffers) {
    size_t buffer_size = icm42688_get_required_buffer_size(&icm42688_device);
    LOG_I(INIT, "DMA buffer size needed: %zu bytes", buffer_size);

    buffers->buffer_ping    = (uint8_t*)aligned_alloc(64, buffer_size);
    buffers->buffer_pong    = (uint8_t*)aligned_alloc(64, buffer_size);
    buffers->buffer_size    = buffer_size;
    buffers->current_buffer = buffers->buffer_ping;
    buffers->buffer_lock    = spin_lock_instance(next_striped_spin_lock_num());

    if (!buffers->buffer_ping || !buffers->buffer_pong || !buffers->buffer_lock) {
        LOG_E(INIT, "Failed to allocate DMA buffers or spinlock");
        if (buffers->buffer_ping)
            free(buffers->buffer_ping);
        if (buffers->buffer_pong)
            free(buffers->buffer_pong);
        return false;
    }

    LOG_OK(INIT, "DMA buffers allocated and spinlock initialized");
    return true;
}

/*=============================================================================
 * SYSTEM INITIALIZATION
 *===========================================================================*/

bool init_system_dev(psl_state_handler_t* handler) {
    if (!reset_state_handlers(handler)) {
        return false; /*FIXME: this is a pv_state_handler error how to fix it ?*/
    }

    /* == SPI =================================================================== */
    LOG_I(INIT, "SPI0 for ICM42688");
    init_spi0_for_icm42688();

    /* == ICM42688 ============================================================== */
    LOG_I(IMU, "Initializing ICM42688");
    if (icm42688_init_spi(&icm42688_device, SPI_PORT, SPI_CS, &icm42688_cfg) != 0) {
        handler->error_flags |= ERROR_ICM42688_INIT;
        psl_set_state(handler, STATE_ERROR);
        return false;
    }

    if (!icm42688_is_connected(&icm42688_device)) {
        LOG_E(IMU, "ICM42688 not detected");
        handler->error_flags |= ERROR_ICM42688_INIT;
        psl_set_state(handler, STATE_ERROR);
        return false;
    }

    /* Configure accel/gyro ODR, ranges, filters */
    if (icm42688_configure_accel_gyro(&icm42688_device) != ICM42688_OK) {
        LOG_E(IMU, "Accel/gyro configuration failed");
        handler->error_flags |= ERROR_ICM42688_INIT;
        psl_set_state(handler, STATE_ERROR);
        return false;
    }

    /* Configure FIFO (mode, watermark, packet format) */
    if (icm42688_configure_fifo(&icm42688_device) != ICM42688_OK) {
        LOG_E(IMU, "FIFO configuration failed");
        handler->error_flags |= ERROR_ICM42688_INIT;
        psl_set_state(handler, STATE_ERROR);
        return false;
    }

    /* Configure interrupt (INT1 = FIFO watermark, PULSE, active-high) */
    if (icm42688_configure_interrupt(&icm42688_device) != ICM42688_OK) {
        LOG_E(IMU, "Interrupt configuration failed");
        handler->error_flags |= ERROR_ICM42688_INIT;
        psl_set_state(handler, STATE_ERROR);
        return false;
    }

    handler->modules_states.icm42688_ready = true;
    LOG_OK(IMU, "ICM42688 ready (accel/gyro/FIFO/INT configured)");

    /* == DMA =================================================================== */
    LOG_I(DMA, "Configuring DMA");
    init_spi_dma();
    handler->modules_states.dma_configured = true;
    LOG_OK(DMA, "DMA configured");

    /* == DMA Buffers =========================================================== */
    LOG_I(INIT, "Allocating DMA buffers");
    if (!init_dma_buffers(&dma_buffers)) {
        handler->error_flags |= ERROR_BUFFER_ALLOC;
        psl_set_state(handler, STATE_ERROR);
        return false;
    }
    handler->modules_states.buffers_allocated = true;
    LOG_OK(INIT, "DMA buffers ready");

    /* == FIFO ================================================================== */
    LOG_I(FIFO, "Allocating inter-core FIFO");
    if (!fifo_init(&g_fifo, dma_buffers.buffer_size)) {
        handler->error_flags |= ERROR_BUFFER_ALLOC;
        psl_set_state(handler, STATE_ERROR);
        return false;
    }
    LOG_OK(FIFO, "Inter-core FIFO ready");

    /* == Shared State ========================================================== */
    LOG_I(INIT, "Initializing shared state (inter-core)");
    if (!shared_state_init(&g_shared_state)) {
        handler->error_flags |= ERROR_BUFFER_ALLOC;
        psl_set_state(handler, STATE_ERROR);
        return false;
    }
    LOG_OK(INIT, "Shared state initialized");

    /* == WiFi & TCP (Core 0) =================================================== */
    if (!core0_init_wifi_tcp(handler)) {
        return false;
    }

    /* == Transition to WAIT_FOR_CLIENT ========================================= */
    if (can_enter_idle(handler)) {
        psl_set_state(handler, STATE_WAIT_FOR_CLIENT);
        LOG_OK(INIT, "System ready, waiting for client");
        return true;
    } else {
        psl_set_state(handler, STATE_ERROR);
        return false;
    }
}
