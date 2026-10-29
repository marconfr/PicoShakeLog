/**
 * @file psl_core1.c
 * @brief Core 1 implementation: Real-time IMU data acquisition
 *
 * This file implements all Core 1 operations for the PicoShakeLog system,
 * dedicated to real-time IMU data acquisition. Core 1 operates independently,
 * monitoring the IMU interrupt pin and using DMA for high-speed data transfers.
 * Acquired data is pushed to the inter-core FIFO for consumption by Core 0.
 *
 * Operation:
 * - GPIO interrupt on IMU INT pin (data ready)
 * - DMA transfer initiation for SPI burst read
 * - DMA completion interrupt handling
 * - Data validation and FIFO push
 * - Zero-copy, non-blocking architecture
 *
 * @author Frédéric Marçon
 * @date 2026
 * @copyright Academic Free License v3.0
 */

#include "psl_core1.h"
#include "psl_api.h"
#include "psl_config.h"
#include "psl_fifo.h"
#include "psl_log.h"
#include "psl_shared.h"

#include "hardware/dma.h"
#include "hardware/gpio.h"
#include "hardware/irq.h"
#include "hardware/spi.h"
#include "icm42688_driver.h"
#include "pico/time.h"

#include "pico/stdlib.h"
#include <stdio.h>
#include <string.h>

/*=============================================================================
 * EXTERNAL VARIABLES (defined in psl_api.c)
 *===========================================================================*/

extern psl_dma_buffers_t        dma_buffers;
extern icm42688_connection_handler_t icm42688_device;

/* Dummy byte sent by TX DMA to generate SPI clock during burst read.
 * Must be in SRAM (not flash/const) for DMA to read reliably. */
/* There is a lot of dummy things...*/
static uint8_t spi_dummy_byte_core1 = 0x00;

/* ---- DEBUG COUNTERS (retirer après diagnostic) ---- */
static volatile uint32_t dbg_gpio_irq_count  = 0; /* GPIO INT1 edges reçus         */
static volatile uint32_t dbg_dma_start_count = 0; /* core1_start_dma_transfer()    */
static volatile uint32_t dbg_dma_isr_count   = 0; /* DMA RX ISR (transferts finis) */
static uint32_t          dbg_last_print_ms   = 0;
/* --------------------------------------------------- */

/*=============================================================================
 * DEBUG HELPERS
 *===========================================================================*/

#if LOG_LEVEL <= LOG_LEVEL_DEBUG
/** Formate un octet sous forme binaire "0b11111111" dans buf (>= 11 octets) */
static const char* bin8_str(uint8_t v, char* buf) {
    buf[0] = '0';
    buf[1] = 'b';
    for (int i = 0; i < 8; i++) {
        buf[2 + i] = ((v >> (7 - i)) & 1) ? '1' : '0';
    }
    buf[10] = '\0';
    return buf;
}

/** Lit un registre Bank0 de l'ICM42688 et affiche son adresse + valeur binaire */
static void dbg_read_reg(const char* name, uint8_t addr) {
    uint8_t val = 0;
    char    bin[11];
    icm42688_device.read_regs(&icm42688_device, addr, &val, 1);
    LOG_D(IMU, "  %-20s [0x%02X] = %s (0x%02X)", name, addr, bin8_str(val, bin), val);
}

/** Dump complet des registres de configuration ICM42688 (Bank 0) */
static void dbg_dump_icm42688_regs(void) {
    /* Lire INT_STATUS EN PREMIER, avant tout accès FIFO qui pourrait le modifier */
    uint8_t int_status = 0;
    icm42688_device.read_regs(&icm42688_device, 0x2D, &int_status, 1);

    char bin[11];
    LOG_D(IMU, "=== ICM42688 Register Dump (Bank 0) ===");
    LOG_D(IMU, "  %-20s [0x2D] = %s (0x%02X)  FIFO_THS=%d  FIFO_FULL=%d  DRDY=%d",
          "INT_STATUS (1st)", bin8_str(int_status, bin), int_status, (int_status >> 2) & 1,
          (int_status >> 1) & 1, (int_status >> 3) & 1);

    dbg_read_reg("PWR_MGMT0", 0x4E);
    dbg_read_reg("GYRO_CONFIG0", 0x4F);
    dbg_read_reg("ACCEL_CONFIG0", 0x50);
    dbg_read_reg("INTF_CONFIG1", 0x4D);
    dbg_read_reg("INT_CONFIG", 0x14);
    dbg_read_reg("INT_CONFIG0", 0x63);
    dbg_read_reg("INT_CONFIG1", 0x64);
    dbg_read_reg("INT_SOURCE0", 0x65);
    dbg_read_reg("INTF_CONFIG0",
                 0x4C); /* bit6=FIFO_COUNT_IN_REC bit5=COUNT_ENDIAN bit4=DATA_ENDIAN */
    dbg_read_reg("FIFO_CONFIG", 0x16);
    dbg_read_reg("FIFO_CONFIG1", 0x5F);
    dbg_read_reg("FIFO_CONFIG2", 0x60); /* WM [7:0]  */
    dbg_read_reg("FIFO_CONFIG3", 0x61); /* WM [11:8] */
    /* Lire FIFO_COUNT après INT_STATUS pour ne pas interférer avec le flag */
    LOG_D(IMU, "  %-20s        = %u records", "FIFO_COUNT",
          (unsigned)icm42688_get_fifo_count(&icm42688_device));
    /* Re-lire INT_STATUS après FIFO_COUNT pour voir si ça change */
    uint8_t int_status2 = 0;
    icm42688_device.read_regs(&icm42688_device, 0x2D, &int_status2, 1);
    LOG_D(IMU, "  %-20s [0x2D] = %s (0x%02X)", "INT_STATUS (after)", bin8_str(int_status2, bin),
          int_status2);
    LOG_D(IMU, "==========================================");
}
#endif /* LOG_LEVEL <= LOG_LEVEL_DEBUG */

/*=============================================================================
 * ISR HANDLERS (CORE 1)
 *===========================================================================*/

/**
 * @brief Interrupt Service Routine (ISR) handler for GPIO events on core 1.
 *
 * This ISR is triggered when a GPIO event occurs. It checks if the event is for the
 * specified SPI interrupt pin (SPI_INT1). If not, the handler returns immediately.
 * If the acquisition is not active (checked via a volatile flag), the handler also returns.
 * Otherwise, it increments a debug counter for GPIO interrupt edges and initiates a DMA transfer.
 *
 * @param gpio The GPIO number that triggered the interrupt.
 * @param events The event flags associated with the interrupt.
 */
static void __isr core1_gpio_isr_handler(uint gpio, uint32_t events) {
    if (gpio != SPI_INT1)
        return;

    /* Read volatile flag directly (no spinlock) : spinlock_blocking is forbidden in ISR */
    if (!g_shared_state.acquisition_active) {
        return;
    }

    dbg_gpio_irq_count++; /* DEBUG: GPIO INT1 edges */
    core1_start_dma_transfer();
}

/**
 * @brief DMA interrupt service routine (ISR) handler for Core 1.
 *
 * This ISR is triggered when a DMA transfer on the specified SPI DMA channel completes.
 * It performs the following actions:
 *   - Acknowledges the DMA interrupt.
 *   - Terminates the SPI transaction by setting chip select (CS) high.
 *   - Retrieves the current DMA buffer pointer.
 *   - Increments a debug counter for DMA ISR invocations.
 *   - Swaps the ping/pong DMA buffers for the next transfer.
 *   - If data acquisition is active, pushes the completed buffer to the FIFO.
 *     Handles FIFO overflow gracefully.
 *
 * @note This function runs in interrupt context on Core 1. Spinlocks or blocking operations
 *       are forbidden in this context. The buffer pointer is only modified here, so no
 *       synchronization is required.
 */
static void __isr core1_dma_isr_handler(void) {
    if (dma_channel_get_irq0_status(SPI_DMA_CHANNEL)) {
        dma_channel_acknowledge_irq0(SPI_DMA_CHANNEL);

        /* Terminate SPI transaction (CS high) of core1_start_dma_transfer*/
        icm42688_dma_end_spi_fifo_read(&icm42688_device);

        /* Get current DMA buffer.
         * The buffer pointer is only modified here (in this ISR, Core 1 only).
         * No spinlock needed: spinlock_blocking is forbidden in ISR context. */
        uint8_t* current_buf = dma_buffers.current_buffer;

        dbg_dma_isr_count++; /* DEBUG */

        /* Swap ping/pong for next burst */
        dma_buffers.current_buffer = (dma_buffers.current_buffer == dma_buffers.buffer_ping)
                                         ? dma_buffers.buffer_pong
                                         : dma_buffers.buffer_ping;

        /* Push to FIFO only if acquisition is active (guards residual DMA on stop).
         * Read volatile flag directly - spinlock_blocking forbidden in ISR. */
        if (g_shared_state.acquisition_active) {
            if (!fifo_push(&g_fifo, current_buf)) {
            }
        }
    }
}

/*=============================================================================
 * CORE 1 FUNCTIONS
 *===========================================================================*/

void core1_start_dma_transfer(void) {
    /* Read current buffer pointer: volatile read, no spinlock needed in ISR.
     * The buffer pointer is only swapped in core1_dma_isr_handler (same core). */
    uint8_t* dest_buffer = dma_buffers.current_buffer;
    uint32_t count       = dma_buffers.buffer_size;

    dbg_dma_start_count++; /* DEBUG: DMA starts */

    /* Send SPI read command for FIFO_DATA register (CS low + command byte) */
    icm42688_dma_start_spi_fifo_read(&icm42688_device);

    /* Fully reconfigure both channels at each burst.
     * Using only set_write_addr/set_trans_count does NOT re-arm the DREQ
     * when the channel has previously finished with count=0 (initial state).
     * dma_channel_configure() rewrites the full control register including
     * the DREQ, which guarantees rate-limiting by SPI hardware. */

    /* RX channel: SPI RX FIFO → dest_buffer, throttled by DREQ_RX */
    dma_channel_config cfg_rx = dma_channel_get_default_config(SPI_DMA_CHANNEL);
    channel_config_set_transfer_data_size(&cfg_rx, DMA_SIZE_8);
    channel_config_set_dreq(&cfg_rx, spi_get_dreq(SPI_PORT, false)); /* DREQ RX */
    channel_config_set_read_increment(&cfg_rx, false);               /* fixed: SPI RX reg */
    channel_config_set_write_increment(&cfg_rx, true);               /* increment dest buf */
    dma_channel_configure(SPI_DMA_CHANNEL, &cfg_rx, dest_buffer,     /* write: ping or pong */
                          &spi_get_hw(SPI_PORT)->dr,                 /* read:  SPI RX FIFO */
                          count, false);                             /* no start yet */

    /* TX channel: dummy byte → SPI TX FIFO, throttled by DREQ_TX (generates SCK) */
    dma_channel_config cfg_tx = dma_channel_get_default_config(SPI_DMA_TX_CHANNEL);
    channel_config_set_transfer_data_size(&cfg_tx, DMA_SIZE_8);
    channel_config_set_dreq(&cfg_tx, spi_get_dreq(SPI_PORT, true)); /* DREQ TX */
    channel_config_set_read_increment(&cfg_tx, false);              /* fixed: same dummy byte */
    channel_config_set_write_increment(&cfg_tx, false);             /* fixed: SPI TX reg */
    dma_channel_configure(SPI_DMA_TX_CHANNEL, &cfg_tx,
                          &spi_get_hw(SPI_PORT)->dr, /* write: SPI TX FIFO */
                          &spi_dummy_byte_core1,     /* read:  dummy byte  */
                          count, false);             /* no start yet */

    /* Start TX and RX simultaneously (atomic) to avoid losing the first byte (DMA API)*/
    dma_start_channel_mask((1u << SPI_DMA_CHANNEL) | (1u << SPI_DMA_TX_CHANNEL));
}

void core1_entry(void) {
    LOG_I(CORE1, "Started");

    /* Register IRQ handlers (Core 1 only) */
    irq_set_exclusive_handler(DMA_IRQ_0, core1_dma_isr_handler);
    irq_set_enabled(DMA_IRQ_0, true);
    dma_channel_set_irq0_enabled(SPI_DMA_CHANNEL, true);

    /* Register GPIO IRQ.
     * Read current level BEFORE enabling IRQ: if INT1 is already HIGH (front déjà passé
     * pendant l'init), on le logue pour diagnostiquer un "missed edge". */
    bool int1_already_high = gpio_get(SPI_INT1);
    gpio_set_irq_enabled_with_callback(SPI_INT1, GPIO_IRQ_EDGE_RISE, true, &core1_gpio_isr_handler);

    LOG_D(CORE1, "IRQ handlers registered (INT1 level at register time = %d)", int1_already_high);

    /* Core 1 infinite loop */
    while (true) {
        /* Wait for acquisition to be enabled */
        if (shared_state_get_acquisition(&g_shared_state)) {
            /* Active mode: sleep court pour ne pas bloquer sur __wfi() pendant le debug */
            sleep_ms(1);

            /* Debugging: enable by building with -DLOG_LEVEL=LOG_LEVEL_DEBUG.
             * This avoids spamming logs in normal operation while keeping the
             * diagnostic code available for investigations. */
#if LOG_LEVEL <= LOG_LEVEL_DEBUG
            uint32_t now_ms = to_ms_since_boot(get_absolute_time());
            if (now_ms - dbg_last_print_ms >= 1000) {
                dbg_last_print_ms   = now_ms;
                uint8_t* buf        = dma_buffers.current_buffer;
                bool     int1_level = gpio_get(SPI_INT1);
                uint8_t  int_status = 0;
                uint16_t fifo_count = icm42688_get_fifo_count(&icm42688_device);
                icm42688_device.read_regs(&icm42688_device, 0x2D, &int_status, 1);
                LOG_D(CORE1,
                      "GPIO_IRQ=%lu  DMA_START=%lu  DMA_ISR=%lu"
                      "  INT1_now=%d  INT_STATUS=0x%02X  FIFO_COUNT=%u"
                      "  buf[0..3]=%02X %02X %02X %02X",
                      (unsigned long)dbg_gpio_irq_count, (unsigned long)dbg_dma_start_count,
                      (unsigned long)dbg_dma_isr_count, (int)int1_level, int_status,
                      (unsigned)fifo_count, buf[0], buf[1], buf[2], buf[3]);
                dbg_dump_icm42688_regs();
                dbg_gpio_irq_count  = 0;
                dbg_dma_start_count = 0;
                dbg_dma_isr_count   = 0;
            }
#endif
        } else {
            /* Idle mode */
            sleep_ms(50);
        }
    }
}
