/**
 * @file psl_config.h
 * @brief Hardware configuration and constants for PicoShakeLog system
 *
 * This header centralizes all hardware pin assignments, communication parameters,
 * and system-wide constants. Modify this file to adapt the system to different
 * hardware configurations or to tune performance parameters.
 *
 * Configuration includes:
 * - SPI pins and baudrate for IMU communication
 * - WiFi credentials and network settings
 * - DMA channel assignments
 * - FIFO buffer sizing
 * - Protocol command definitions
 *
 * @author Frédéric Marçon
 * @date 2026
 * @copyright Academic Free License v3.0
 *
 * @see psl_api.h for system initialization
 */

#ifndef PSL_CONFIG_H
#define PSL_CONFIG_H

#include "hardware/gpio.h"
#include "hardware/spi.h"

#ifdef __cplusplus
extern "C" {
#endif

/*=============================================================================
 * SPI Configuration
 *===========================================================================*/
#define SPI_PORT     spi0
#define SPI_BAUDRATE 10 * 1000 * 1000 /* 10 MHz */
#define SPI_CS       17
#define SPI_SCK      18
#define SPI_MOSI     19
#define SPI_MISO     16
#define SPI_INT1     13
#define SPI_INT2     14

/*=============================================================================
 * DMA Configuration
 *===========================================================================*/
#define SPI_DMA_CHANNEL    0 /* RX: SPI → buffer  */
#define SPI_DMA_TX_CHANNEL 1 /* TX: dummy bytes → SPI (generates SPI clock) */

/*=============================================================================
 * WIFI Configuration
 *===========================================================================*/
#define WIFI_SSID  "PicoShakeLog"
#define WIFI_PASS  "PicoShakeLog"
#define FIFO_SLOTS 32

#ifdef __cplusplus
}
#endif

#endif /* PSL_CONFIG_H */