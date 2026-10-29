/**
 * @file    ICM42688.h
 * @brief   Driver header for ICM42688 6-axis IMU sensor (C version)
 * @details Supports I2C and SPI communication, FIFO management, and filtering
 *          for the ICM42688 IMU sensor.
 * @note    We use dev for device don't be choked by if dev==NULL != developers==NULL (or not)
 * @author Frédéric Marçon
 * @date 2026
 * @copyright Academic Free License v3.0
 */

#ifndef ICM42688_DRIVER_H
#define ICM42688_DRIVER_H

#include <hardware/i2c.h>
#include <hardware/spi.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*============================================================================
 * CONSTANTS AND STATUS CODES
 *============================================================================*/

#define ICM42688_WHOAMI_VALUE 0x47

typedef enum {
    ICM42688_OK                 = 0,
    ICM42688_ERR_NULL_PTR       = -1,
    ICM42688_ERR_BUS            = -2,
    ICM42688_ERR_NOT_FOUND      = -3,
    ICM42688_ERR_INVALID_ARG    = -4,
    ICM42688_ERR_CLOCKIN_CONFIG = -5,
    ICM42688_ERR_MAP_FSYNC_INT2 = -6
} icm42688_status_t;

/*============================================================================
 * BUS TYPE AND HANDLER
 *============================================================================*/

typedef enum { ICM42688_BUS_NONE = 0, ICM42688_BUS_I2C, ICM42688_BUS_SPI } icm42688_bus_type_t;

/**
 * @struct icm42688_conversion_factors
 * @brief Conversion factors for ICM42688 sensor data
 *
 * This structure contains the scaling factors needed to convert raw
 * sensor readings to physical units. The factors depend on the configured
 * full-scale range settings of the sensor.
 *
 * @var icm42688_conversion_factors::fifo_accel_g_per_lsb
 * Accelerometer conversion factor for FIFO data in g per LSB
 *
 * @var icm42688_conversion_factors::fifo_gyro_g_per_lsb
 * Gyroscope conversion factor for FIFO data in degrees per second per LSB
 *
 * @var icm42688_conversion_factors::fs_accel_g_per_lsb
 * Accelerometer conversion factor for register data in g per LSB (full-scale)
 *
 * @var icm42688_conversion_factors::fs_gyro_g_per_lsb
 * Gyroscope conversion factor for register data in degrees per second per LSB (full-scale)
 */
typedef struct icm42688_conversion_factors {
    float fifo_accel_g_per_lsb; /* FIFO high-res mode */
    float fifo_gyro_g_per_lsb;  /* FIFO high-res mode */
    float fs_accel_g_per_lsb;   /* Register/Full-scale mode */
    float fs_gyro_g_per_lsb;    /* Register/Full-scale mode */
} icm42688_conversion_factors_t;

/*============================================================================
 * CONFIGURATION STRUCTURES
 *============================================================================*/

/*----------------------------------------------------------------------------
 * Predefined Configurations
 *---------------------------------------------------------------------------*/

/**
 * @brief ICM42688 sensor configuration structure
 * Must be defined before icm42688 struct as it's used as a member
 */
typedef struct icm42688_config {
    uint8_t  accel_range; /* Will be icm42688_accel_range_t */
    uint8_t  gyro_range;  /* Will be icm42688_gyro_range_t */
    uint8_t  accel_odr;   /* Will be icm42688_accel_odr_t */
    uint8_t  gyro_odr;    /* Will be icm42688_gyro_odr_t */
    uint8_t  fifo_mode;   /* Will be icm42688_fifo_mode_t */
    bool     fifo_hires;
    bool     temp_in_fifo;
    bool     fsync_in_fifo;
    uint16_t fifo_watermark;
    uint8_t  aaf_filter_bw;    /* Will be icm42688_filter_3dB_bw_t */
    uint8_t  ui_filter_order;  /* Will be icm42688_ui_filter_order_t */
    uint8_t  ui_filter_number; /* Will be icm42688_ui_filter_bw_t */
    bool     use_external_clkin;
    uint8_t  fifo_packet_size;
} icm42688_config_t;

/**
 * @brief ICM42688 configuration for hand tremor detection
 *
 * Hand tremor characteristics:
 * - Parkinsonian tremor (rest): 4-6 Hz
 * - Essential tremor: 4-12 Hz
 * - Physiological tremor: 8-12 Hz
 * - Action/postural tremor: 6-12 Hz
 *
 * Target frequency band: 3 - 15 Hz (covers all clinical tremor types)
 *
 * Signal chain:
 *   Sensor → AAF (213 Hz) → UI Filter 3rd order (48.9 Hz) → FIFO @ 500 Hz
 *                ↓                      ↓
 *          Anti-aliasing           Aggressive noise rejection
 *          (Nyquist: 250 Hz)       Preserves 3-15 Hz tremor band
 */
#define ICM42688_IMU_CONFIG_TREMOR                                                                 \
    {.accel_range        = ICM42688_ACCEL_RANGE_4G,                                                \
     .gyro_range         = ICM42688_GYRO_RANGE_250DPS,                                             \
     .accel_odr          = ICM42688_ACCEL_ODR_500HZ,                                               \
     .gyro_odr           = ICM42688_GYRO_ODR_500HZ,                                                \
     .fifo_mode          = ICM42688_FIFO_MODE_STREAM,                                              \
     .fifo_hires         = true,                                                                   \
     .temp_in_fifo       = false,                                                                  \
     .fsync_in_fifo      = false,                                                                  \
     .fifo_watermark     = 64,                                                                     \
     .aaf_filter_bw      = ICM42688_FILTER_3dB_BW_213HZ,                                           \
     .ui_filter_order    = ICM42688_UI_FILTER_ORDER_3RD,                                           \
     .ui_filter_number   = ICM42688_UI_FILT_BW_4, /*48.9 Hz*/                                      \
     .use_external_clkin = false,                                                                  \
     .fifo_packet_size   = 0}

/**
 * @brief ICM42688 configuration for transport vibration analysis
 *
 * Transport vibration characteristics:
 * - Road vehicles (car, bus): 1-25 Hz (suspension, engine), peaks at 10-15 Hz
 * - Rail (train, metro): 0.5-80 Hz (wheel/rail interaction, bogie dynamics)
 * - Aircraft: 10-500 Hz (engines, structural modes)
 *
 * Target frequency band: 1 - 100 Hz (covers most transport modes)
 *
 * Signal chain:
 *   Sensor → AAF (536 Hz) → UI Filter 2nd order (111 Hz) → FIFO @ 1 kHz
 *                ↓                      ↓
 *          Anti-aliasing           Noise reduction
 *          (Nyquist: 500 Hz)       Preserves 1-100 Hz band
 */
#define ICM42688_IMU_CONFIG_TRANSPORT                                                              \
    {.accel_range        = ICM42688_ACCEL_RANGE_8G,                                                \
     .gyro_range         = ICM42688_GYRO_RANGE_500DPS,                                             \
     .accel_odr          = ICM42688_ACCEL_ODR_1KHZ,                                                \
     .gyro_odr           = ICM42688_GYRO_ODR_1KHZ,                                                 \
     .fifo_mode          = ICM42688_FIFO_MODE_STREAM,                                              \
     .fifo_hires         = true,                                                                   \
     .temp_in_fifo       = true,                                                                   \
     .fsync_in_fifo      = false,                                                                  \
     .fifo_watermark     = 64,                                                                     \
     .aaf_filter_bw      = ICM42688_FILTER_3dB_BW_536HZ,                                           \
     .ui_filter_order    = ICM42688_UI_FILTER_ORDER_2ND,                                           \
     .ui_filter_number   = ICM42688_UI_FILT_BW_1,                                                  \
     .use_external_clkin = false,                                                                  \
     .fifo_packet_size   = 0}

typedef struct icm42688 icm42688_connection_handler_t;

/**
 * @brief ICM42688 device instance structure
 *
 * This structure contains all configuration, state, and interface information
 * for an ICM42688 IMU sensor instance. It supports both I2C and SPI communication
 * and stores calibration/conversion factors for sensor data processing.
 *
 * @warning You should not modify the fields of this structure directly. Use the
 *          provided initialization methods (icm42688_init_i2c or icm42688_init_spi)
 *
 * @warning When switching between I2C and SPI modes, ensure only the relevant
 *          bus configuration fields are accessed based on the bus_type setting.
 *
 * @par Example Usage (SPI):
 * @code
 * //Use predefined tremor detection configuration
 * icm42688_config_t imu_config = ICM42688_IMU_CONFIG_TREMOR;
 *
 * //Create connection handler
 * icm42688_connection_handler_t icm42688_connection;
 *
 * //Initialize sensor over SPI
 * if (icm42688_init_spi(&icm42688_connection, spi0, SPI_CS_PIN, &imu_config) != ICM42688_OK) {
 *     printf("Failed to initialize ICM42688 over SPI!\n");
 *     return -1;
 * }
 *
 * //Configure and start sensor
 * icm42688_configure_accel_gyro(&icm42688_connection);
 * icm42688_configure_fifo(&icm42688_connection);
 * icm42688_start_sensor(&icm42688_connection);
 * @endcode
 */
struct icm42688 {
    icm42688_bus_type_t bus_type;

    /* I2C config (used when bus_type == ICM42688_BUS_I2C) */
    i2c_inst_t* i2c;
    uint8_t     i2c_addr;

    /* SPI config (used when bus_type == ICM42688_BUS_SPI) */
    spi_inst_t* spi;
    uint8_t     spi_cs_pin;

    /* Internal function pointers (set by init) */
    bool (*read_regs)(icm42688_connection_handler_t* dev, uint8_t reg, uint8_t* buf, size_t len);
    bool (*write_reg)(icm42688_connection_handler_t* dev, uint8_t reg, uint8_t val);
    bool (*write_reg_mask)(icm42688_connection_handler_t* dev, uint8_t reg, uint8_t mask,
                           uint8_t val);

    icm42688_config_t config; /* Sensor configuration (stored after first configure call) */
    bool              sensor_data_endian_big; /* Endianness for sensor data registers */
    bool              fifo_count_endian_big;  /* Endianness for FIFO count */
    bool              fifo_count_in_rec;      /* FIFO count are in REC not in Bytes */

    /* Conversion factors (calculated when sensor starts) */
    icm42688_conversion_factors_t conversion_factors;
};

/*============================================================================
 * BUS INITIALIZATION FUNCTIONS
 * ============================================================================*/

/**
 * @brief Initialize icm42688 over I2C.
 *
 * @param dev   Pointer to icm42688_connection_handler_t handle to initialize
 * @param i2c   I2C instance (must be already initialized)
 * @param addr  I2C address (icm42688_I2C_ADDR_DEFAULT or ALTERNATE)
 * @param config Pointer to the configuration to store in the device
 * @return icm42688_status_t
 */
icm42688_status_t icm42688_init_i2c(icm42688_connection_handler_t* dev, i2c_inst_t* i2c,
                                    uint8_t addr, const icm42688_config_t* config);

/**
 * @brief Initialize icm42688 over SPI.
 *
 * @param dev     Pointer to icm42688_connection_handler_t handle to initialize
 * @param spi     SPI instance (must be already initialized)
 * @param cs_pin  Chip select GPIO pin (must be configured as output)
 * @param config  Pointer to the configuration to store in the device
 * @return icm42688_status_t
 */
icm42688_status_t icm42688_init_spi(icm42688_connection_handler_t* dev, spi_inst_t* spi,
                                    uint8_t cs_pin, const icm42688_config_t* config);

/*============================================================================
 * SENSOR CONFIGURATION ENUMS
 *============================================================================*/

/*----------------------------------------------------------------------------
 * Accelerometer Settings
 *---------------------------------------------------------------------------*/

typedef enum {
    ICM42688_ACCEL_RANGE_2G  = 0b01100000,
    ICM42688_ACCEL_RANGE_4G  = 0b01000000,
    ICM42688_ACCEL_RANGE_8G  = 0b00100000,
    ICM42688_ACCEL_RANGE_16G = 0b00000000
} icm42688_accel_range_t;

typedef enum {
    ICM42688_ACCEL_ODR_32KHZ    = 0b00000001,
    ICM42688_ACCEL_ODR_16KHZ    = 0b00000010,
    ICM42688_ACCEL_ODR_8KHZ     = 0b00000011,
    ICM42688_ACCEL_ODR_4KHZ     = 0b00000100,
    ICM42588_ACCEL_ODR_2KHZ     = 0b00000101,
    ICM42688_ACCEL_ODR_1KHZ     = 0b00000110,
    ICM42688_ACCEL_ODR_500HZ    = 0b00001111,
    ICM42688_ACCEL_ODR_200HZ    = 0b00000111,
    ICM42688_ACCEL_ODR_100HZ    = 0b00001000,
    ICM42688_ACCEL_ODR_50HZ     = 0b00001001,
    ICM42688_ACCEL_ODR_25HZ     = 0b00001010,
    ICM42688_ACCEL_ODR_12_5HZ   = 0b00001011,
    ICM42688_ACCEL_ODR_6_25HZ   = 0b00001100,
    ICM42688_ACCEL_ODR_3_125HZ  = 0b00001101,
    ICM42688_ACCEL_ODR_1_5625HZ = 0b00001110,
} icm42688_accel_odr_t;

/*----------------------------------------------------------------------------
 * Gyroscope Settings
 *---------------------------------------------------------------------------*/

typedef enum {
    ICM42688_GYRO_RANGE_2000DPS   = 0b00000000, /* GYRO_CONFIG0[7:5] = 000 */
    ICM42688_GYRO_RANGE_1000DPS   = 0b00100000, /* GYRO_CONFIG0[7:5] = 001 */
    ICM42688_GYRO_RANGE_500DPS    = 0b01000000, /* GYRO_CONFIG0[7:5] = 010 */
    ICM42688_GYRO_RANGE_250DPS    = 0b01100000, /* GYRO_CONFIG0[7:5] = 011 */
    ICM42688_GYRO_RANGE_125DPS    = 0b10000000, /* GYRO_CONFIG0[7:5] = 100 */
    ICM42688_GYRO_RANGE_62_5DPS   = 0b10100000, /* GYRO_CONFIG0[7:5] = 101 */
    ICM42688_GYRO_RANGE_31_25DPS  = 0b11000000, /* GYRO_CONFIG0[7:5] = 110 */
    ICM42688_GYRO_RANGE_15_625DPS = 0b11100000  /* GYRO_CONFIG0[7:5] = 111 */
} icm42688_gyro_range_t;

typedef enum {
    ICM42688_GYRO_ODR_32KHZ  = 0b00000001,
    ICM42688_GYRO_ODR_16KHZ  = 0b00000010,
    ICM42688_GYRO_ODR_8KHZ   = 0b00000011,
    ICM42688_GYRO_ODR_4KHZ   = 0b00000100,
    ICM42588_GYRO_ODR_2KHZ   = 0b00000101,
    ICM42688_GYRO_ODR_1KHZ   = 0b00000110,
    ICM42688_GYRO_ODR_500HZ  = 0b00001111,
    ICM42688_GYRO_ODR_200HZ  = 0b00000111,
    ICM42688_GYRO_ODR_100HZ  = 0b00001000,
    ICM42688_GYRO_ODR_50HZ   = 0b00001001,
    ICM42688_GYRO_ODR_25HZ   = 0b00001010,
    ICM42688_GYRO_ODR_12_5HZ = 0b00001011,
} icm42688_gyro_odr_t;

/*----------------------------------------------------------------------------
 * FIFO Mode
 * ---------------------------------------------------------------------------*/

typedef enum {
    ICM42688_FIFO_MODE_BYPASS     = 0b00000000, /* FIFO_CONFIG[7:6] = 00 */
    ICM42688_FIFO_MODE_STREAM     = 0b01000000, /* FIFO_CONFIG[7:6] = 01 */
    ICM42688_FIFO_MODE_STOPONFULL = 0b10000000  /* FIFO_CONFIG[7:6] = 10 */
} icm42688_fifo_mode_t;

/*----------------------------------------------------------------------------
 * Filter Configuration
 *---------------------------------------------------------------------------*/

/**
 * @brief 3dB anti-aliasing filter bandwidth options
 * @note see 5.3
 */
typedef enum {
    ICM42688_FILTER_3dB_BW_42HZ,
    ICM42688_FILTER_3dB_BW_84HZ,
    ICM42688_FILTER_3dB_BW_126HZ,
    ICM42688_FILTER_3dB_BW_170HZ,
    ICM42688_FILTER_3dB_BW_213HZ,
    ICM42688_FILTER_3dB_BW_258HZ,
    ICM42688_FILTER_3dB_BW_303HZ,
    ICM42688_FILTER_3dB_BW_348HZ,
    ICM42688_FILTER_3dB_BW_394HZ,
    ICM42688_FILTER_3dB_BW_441HZ,
    ICM42688_FILTER_3dB_BW_488HZ,
    ICM42688_FILTER_3dB_BW_536HZ,
    ICM42688_FILTER_3dB_BW_585HZ,
    ICM42688_FILTER_3dB_BW_634HZ,
    ICM42688_FILTER_3dB_BW_684HZ,
    ICM42688_FILTER_3dB_BW_734HZ,
    ICM42688_FILTER_3dB_BW_785HZ,
    ICM42688_FILTER_3dB_BW_837HZ,
    ICM42688_FILTER_3dB_BW_890HZ,
    ICM42688_FILTER_3dB_BW_943HZ,
    ICM42688_FILTER_3dB_BW_997HZ,
    ICM42688_FILTER_3dB_BW_1051HZ,
    ICM42688_FILTER_3dB_BW_1107HZ,
    ICM42688_FILTER_3dB_BW_1163HZ,
    ICM42688_FILTER_3dB_BW_1220HZ,
    ICM42688_FILTER_3dB_BW_1277HZ,
    ICM42688_FILTER_3dB_BW_1336HZ,
    ICM42688_FILTER_3dB_BW_1395HZ,
    ICM42688_FILTER_3dB_BW_1454HZ,
    ICM42688_FILTER_3dB_BW_1515HZ,
    ICM42688_FILTER_3dB_BW_1577HZ,
    ICM42688_FILTER_3dB_BW_1639HZ,
    ICM42688_FILTER_3dB_BW_1702HZ,
    ICM42688_FILTER_3dB_BW_1766HZ,
    ICM42688_FILTER_3dB_BW_1830HZ,
    ICM42688_FILTER_3dB_BW_1896HZ,
    ICM42688_FILTER_3dB_BW_1962HZ,
    ICM42688_FILTER_3dB_BW_2029HZ,
    ICM42688_FILTER_3dB_BW_2097HZ,
    ICM42688_FILTER_3dB_BW_2166HZ,
    ICM42688_FILTER_3dB_BW_2235HZ,
    ICM42688_FILTER_3dB_BW_2306HZ,
    ICM42688_FILTER_3dB_BW_2377HZ,
    ICM42688_FILTER_3dB_BW_2449HZ,
    ICM42688_FILTER_3dB_BW_2522HZ,
    ICM42688_FILTER_3dB_BW_2596HZ,
    ICM42688_FILTER_3dB_BW_2671HZ,
    ICM42688_FILTER_3dB_BW_2746HZ,
    ICM42688_FILTER_3dB_BW_2823HZ,
    ICM42688_FILTER_3dB_BW_2900HZ,
    ICM42688_FILTER_3dB_BW_2978HZ,
    ICM42688_FILTER_3dB_BW_3057HZ,
    ICM42688_FILTER_3dB_BW_3137HZ,
    ICM42688_FILTER_3dB_BW_3217HZ,
    ICM42688_FILTER_3dB_BW_3299HZ,
    ICM42688_FILTER_3dB_BW_3381HZ,
    ICM42688_FILTER_3dB_BW_3464HZ,
    ICM42688_FILTER_3dB_BW_3548HZ,
    ICM42688_FILTER_3dB_BW_3633HZ,
    ICM42688_FILTER_3dB_BW_3718HZ,
    ICM42688_FILTER_3dB_BW_3805HZ,
    ICM42688_FILTER_3dB_BW_3892HZ,
    ICM42688_FILTER_3dB_BW_3979HZ
} icm42688_filter_3dB_bw_t;

/**
 * @brief User interface filter order options
 * @note see 5.5
 */
typedef enum {
    ICM42688_UI_FILTER_ORDER_1ST = 0b00000000,
    ICM42688_UI_FILTER_ORDER_2ND = 0b00000100,
    ICM42688_UI_FILTER_ORDER_3RD = 0b00001000
} icm42688_ui_filter_order_t;

/**
 * @brief User interface filter bandwidth options
 * @note see 5.5 tables according to filter order
 */
typedef enum {
    ICM42688_UI_FILT_BW_0 = 0, // Low Latency (ODR/2)
    ICM42688_UI_FILT_BW_1 = 1, // Low Noise
    ICM42688_UI_FILT_BW_2 = 2, // Low Noise
    ICM42688_UI_FILT_BW_3 = 3, // Low Noise
    ICM42688_UI_FILT_BW_4 = 4, // Low Noise  ~49 Hz @ 500 Hz ODR (3rd order)
    ICM42688_UI_FILT_BW_5 = 5, // Low Noise  ~30 Hz @ 500 Hz ODR (3rd order)
    ICM42688_UI_FILT_BW_6 = 6, // Low Noise  ~23 Hz @ 500 Hz ODR (3rd order)
    ICM42688_UI_FILT_BW_7 = 7, // Low Noise  ~13 Hz @ 500 Hz ODR (3rd order)
    // 8-13 = Reserved
    ICM42688_UI_FILT_BW_14 = 14, // Low Latency
    ICM42688_UI_FILT_BW_15 = 15, // Low Latency (ODR/4)
} icm42688_ui_filter_bw_t;

/*============================================================================
 * DATA STRUCTURES
 *============================================================================*/

/**
 * @struct icm42688_raw_data
 * @brief Raw sensor data from ICM42688 IMU
 * This structure holds the raw 16-bit values read directly from the
 * ICM42688 sensor registers without any conversion or scaling applied.
 */
typedef struct icm42688_raw_data {
    uint16_t accelX;
    uint16_t accelY;
    uint16_t accelZ;
    uint16_t gyroX;
    uint16_t gyroY;
    uint16_t gyroZ;
    uint16_t temp;
} icm42688_raw_data_t;

/**
 * @struct icm42688_converted_data
 * @brief Converted sensor data from ICM42688 IMU
 * This structure holds the converted values read directly from the
 * ICM42688 sensor registers.
 */
typedef struct icm42688_converted_data {
    float accelX;
    float accelY;
    float accelZ;
    float gyroX;
    float gyroY;
    float gyroZ;
    float temp;
} icm42688_converted_data_t;

/*============================================================================
 * PUBLIC FUNCTION DECLARATIONS
 *============================================================================*/

/*----------------------------------------------------------------------------
 * Sensor Detection and Control
 *---------------------------------------------------------------------------*/

/**
 * @brief Check if the ICM42688 sensor is connected and responding
 *
 * This function verifies the connection to the ICM42688 IMU sensor by attempting
 * to communicate with the device through the provided connection handler.
 *
 * @param[in] dev Pointer to the ICM42688 connection handler structure
 *
 * @return true if the sensor is connected and responding correctly
 * @return false if the sensor is not connected or communication failed
 *
 * @note The connection handler must be properly initialized before calling this function
 * @warning A NULL pointer for dev parameter may cause undefined behavior
 */
bool icm42688_is_connected(icm42688_connection_handler_t* dev);

/*----------------------------------------------------------------------------
 * Sensor Configuration Functions
 *---------------------------------------------------------------------------*/

/**
 * @brief Configures the accelerometer and gyroscope settings for the ICM42688 sensor
 *
 * This function applies the configuration parameters stored in dev->config
 * to both the accelerometer and gyroscope of the ICM42688 device.
 *
 * @param[in] dev Pointer to the ICM42688 connection handler structure
 *
 * @return icm42688_status_t Status code indicating success or failure of the operation
 *         - ICM42688_OK: Configuration applied successfully
 *         - ICM42688_ERR_NULL_PTR: Invalid device handler pointer
 *         - ICM42688_ERR_BUS: Communication error with the device
 *
 * @note Uses dev->config which must be initialized via icm42688_init_i2c() or icm42688_init_spi()
 * @note This function may perform register bank switching internally
 * @note Accelerometer and gyroscope should be in standby mode before configuration
 * @note Configuration includes full-scale range, output data rate (ODR), and filter settings
 *
 * @warning Changing configuration while sensors are active may cause temporary data inconsistency
 *
 * @see icm42688_init_i2c()
 * @see icm42688_init_spi()
 */
icm42688_status_t icm42688_configure_accel_gyro(icm42688_connection_handler_t* dev);

/**
 * @brief Configure the FIFO buffer settings for the ICM42688 sensor
 *
 * This function configures the FIFO (First-In-First-Out) buffer parameters
 * for the ICM42688 IMU sensor according to the provided configuration. The FIFO
 * buffer stores sensor data samples internally, allowing burst reads and reducing
 * the need for continuous polling.
 *
 * Configuration includes:
 * - FIFO operating mode (bypass, stream, or stop-on-full)
 * - High-resolution mode enable/disable
 * - Temperature data inclusion in FIFO
 * - FSYNC data inclusion in FIFO (when external clock is used)
 * - FIFO watermark threshold for interrupt generation
 *
 * @param[in] dev Pointer to the ICM42688 connection handler structure.
 *                Must be initialized via icm42688_init_i2c() or icm42688_init_spi()
 *
 * @return icm42688_status_t Status code indicating the result:
 *         - ICM42688_OK: FIFO configuration successful
 *         - ICM42688_ERR_NULL_PTR: dev pointer is NULL
 *         - ICM42688_ERR_BUS: Communication error with the sensor
 *         - ICM42688_ERR_INVALID_ARG: Invalid FIFO configuration parameter
 * @note Uses dev->config for FIFO settings. Call icm42688_configure_accel_gyro() first.
 * @note Updates dev->config.fifo_packet_size according to configuration
 * @note The device must be properly initialized before calling this function
 * @note The sensor should be stopped before reconfiguring FIFO to avoid data corruption
 * @note FIFO watermark is specified in bytes (not samples)
 *
 * @see icm42688_clear_fifo()
 * @see icm42688_read_fifo()
 * @see icm42688_get_fifo_count()
 */
icm42688_status_t icm42688_configure_fifo(icm42688_connection_handler_t* dev);

/**
 * @brief Configure interrupt settings for the ICM42688 sensor
 *
 * This function configures the interrupt behavior of the ICM42688 IMU sensor
 * based on the provided configuration parameters.
 *
 * @param[in] dev Pointer to the ICM42688 connection handler structure
 *
 * @return icm42688_status_t Status code indicating success or failure of the operation
 *         - ICM42688_OK: Configuration successful
 *         - ICM42688_ERROR: Configuration failed
 *         - ICM42688_INVALID_PARAM: Invalid parameter provided
 * @note Uses dev->config for interrupt settings. Call icm42688_configure_accel_gyro() first.
 * @note The device must be initialized before calling this function
 * @note This function may modify internal registers of the ICM42688 sensor
 *
 * @see icm42688_config_t
 * @see icm42688_connection_handler_t
 */
icm42688_status_t icm42688_configure_interrupt(icm42688_connection_handler_t* dev);

/**
 * @brief Start accelerometer and gyroscope in low-noise mode
 * @param[in] dev Pointer to the ICM42688 connection handler
 * @return icm42688_status_t Status code
 * @note Uses dev->config to calculate conversion factors. Call icm42688_configure_accel_gyro()
 * first.
 * @note Conversion factors are automatically calculated and stored in dev->conversion_factors
 */
icm42688_status_t icm42688_start_sensor(icm42688_connection_handler_t* dev);

/**
 * @brief Stop accelerometer and gyroscope (power off)
 * @param[in] dev Pointer to the ICM42688 connection handler
 * @return icm42688_status_t Status code
 */
icm42688_status_t icm42688_stop_sensor(icm42688_connection_handler_t* dev);

/*----------------------------------------------------------------------------
 * Data Reading Functions
 *---------------------------------------------------------------------------*/

/**
 * @brief Read raw sensor data (temperature, accel, gyro) from registers
 * @param[in] dev Pointer to the ICM42688 connection handler
 * @param[out] raw_data_buffer Buffer to store the raw sensor data
 * @return icm42688_status_t Status code
 */
icm42688_status_t icm42688_read_raw_sensor_data(icm42688_connection_handler_t* dev,
                                                icm42688_raw_data_t*           raw_data_handler);

icm42688_status_t icm42688_read_and_convert_data(icm42688_connection_handler_t* dev,
                                                 icm42688_converted_data_t*     conv_data_handler);
/*----------------------------------------------------------------------------
 * FIFO Operations
 *---------------------------------------------------------------------------*/

/**
 * @brief Read FIFO data buffer
 * @param[in] dev Pointer to the ICM42688 connection handler
 * @param[out] fifo_buffer Buffer to store FIFO data
 * @return icm42688_status_t Status code
 * @note Uses dev->config.fifo_packet_size for buffer sizing
 */
icm42688_status_t icm42688_read_fifo(icm42688_connection_handler_t* dev, uint8_t* fifo_buffer);

/**
 * @brief Clear FIFO by reading all data
 * @param[in] dev Pointer to the ICM42688 connection handler
 * @return icm42688_status_t Status code
 */
icm42688_status_t icm42688_clear_fifo(icm42688_connection_handler_t* dev);

/**
 * @brief Get the current FIFO count from the ICM42688 sensor
 *
 * This function reads the FIFO count register(s) from the ICM42688 IMU sensor
 * to determine how many bytes of data are currently stored in the FIFO buffer.
 *
 * @param[in] dev Pointer to the ICM42688 connection handler structure containing
 *                device configuration and communication interface details
 *
 * @return uint16_t The number of bytes currently available in the FIFO buffer.
 *                  Returns 0 if the FIFO is empty or if there is a communication error.
 *
 * @note The FIFO count represents the total number of bytes, not the number of samples.
 *       Multiple bytes may represent a single sensor reading depending on the FIFO
 *       configuration and enabled sensors.
 *
 * @warning Ensure that the device pointer is valid and properly initialized before
 *          calling this function.
 */
uint16_t icm42688_get_fifo_count(icm42688_connection_handler_t* dev);

/**
 * @brief Check the number of unread data samples available in the ICM42688 FIFO.
 *
 * This function queries the ICM42688 sensor to determine how many data samples
 * are currently stored in the FIFO buffer and have not yet been read by the host.
 *
 * @param[in] dev Pointer to the ICM42688 connection handler structure containing
 *                the device interface and configuration information.
 *
 * @return uint16_t The number of unread data samples available in the FIFO buffer.
 *                  Returns 0 if no unread data is available or if an error occurs.
 */
uint16_t icm42688_check_unread_data(icm42688_connection_handler_t* dev);

/*============================================================================
 * DMA OPERATIONS (SPI ONLY)
 *============================================================================*/

/**
 * @brief Initiate a DMA-compatible SPI read sequence
 *
 * This function starts a SPI read transaction by:
 * 1. Asserting CS (chip select) low
 * 2. Sending the read command byte (reg | 0x80)
 * 3. Leaving CS asserted for DMA to continue reading
 *
 * @warning This function is SPI-only. For I2C, use standard read functions.
 * @warning CS remains LOW after this call. You MUST call icm42688_dma_end_spi_fifo_read()
 *          after the DMA transfer completes to release CS.
 * @warning Do not use this with I2C devices - it will return an error.
 *
 * @param[in] dev Pointer to the ICM42688 connection handler (must be SPI)
 * @param[in] reg Register address to read from (read bit will be set automatically)
 *
 * @return icm42688_status_t
 *         - ICM42688_OK: Command sent successfully, DMA can proceed
 *         - ICM42688_ERR_NULL_PTR: dev is NULL
 *         - ICM42688_ERR_INVALID_ARG: Device is not configured for SPI
 *
 * @par Typical Usage:
 * @code
 * // 1. Start DMA read sequence
 * icm42688_dma_start_spi_fifo_read(&dev, B0_REG_FIFO_DATA);
 *
 * // 2. Configure and start DMA channel (external code)
 * dma_channel_set_read_addr(dma_chan, &spi_get_hw(spi0)->dr, false);
 * dma_channel_set_write_addr(dma_chan, buffer, false);
 * dma_channel_set_trans_count(dma_chan, length, true);
 *
 * // 3. Wait for DMA completion (IRQ or polling)
 * while (dma_channel_is_busy(dma_chan));
 *
 * // 4. End transaction
 * icm42688_dma_end_spi_fifo_read(&dev);
 * @endcode
 *
 * @see icm42688_dma_end_spi_fifo_read
 */
icm42688_status_t icm42688_dma_start_spi_fifo_read(icm42688_connection_handler_t* dev);

/**
 * @brief Complete a DMA read transaction by releasing chip select
 *
 * This function releases the SPI chip select (CS) line after a DMA transfer
 * initiated by icm42688_dma_start_spi_fifo_read() has completed.
 *
 * @warning Only call this after the DMA transfer is fully complete.
 * @warning Calling this without a prior icm42688_dma_start_spi_fifo_read() may
 *          cause undefined behavior.
 *
 * @param[in] dev Pointer to the ICM42688 connection handler
 *
 * @return icm42688_status_t
 *         - ICM42688_OK: CS released successfully
 *         - ICM42688_ERR_NULL_PTR: dev is NULL
 *         - ICM42688_ERR_INVALID_ARG: Device is not configured for SPI
 *
 * @note This function is lightweight and typically called from a DMA IRQ handler.
 *
 * @see icm42688_dma_start_spi_fifo_read
 */
icm42688_status_t icm42688_dma_end_spi_fifo_read(icm42688_connection_handler_t* dev);

/**
 * @brief Calculate required buffer size for FIFO DMA transfers
 *
 * This function calculates the minimum buffer size needed to hold FIFO data
 * based on the configured watermark and packet size (which depends on high-res mode).
 *
 * @param[in] dev Pointer to the ICM42688 connection handler with configuration set
 *
 * @return size_t Required buffer size in bytes
 *         - Returns (watermark × packet_size) for valid configuration
 *         - Returns 0 if dev is NULL or configuration is invalid
 *
 * @note Call this after icm42688_init_spi/i2c and before allocating DMA buffers.
 * @note In high-res mode (fifo_hires=true): uses Packet 4 (20 bytes)
 * @note In normal mode (fifo_hires=false): uses Packet 3 (16 bytes)
 *
 * @par Example Usage:
 * @code
 * icm42688_config_t config = ICM42688_IMU_CONFIG_TREMOR;
 * icm42688_connection_handler_t imu_dev;
 * icm42688_init_spi(&imu_dev, spi0, CS_PIN, &config);
 *
 * // Calculate required buffer size
 * size_t buffer_size = icm42688_get_required_buffer_size(&imu_dev);
 * uint8_t buffer_A[buffer_size] __attribute__((aligned(4)));
 * uint8_t buffer_B[buffer_size] __attribute__((aligned(4)));
 * @endcode
 *
 * @see icm42688_configure_fifo
 */
size_t icm42688_get_required_buffer_size(const icm42688_connection_handler_t* dev);

#ifdef __cplusplus
}
#endif

#endif /* ICM42688_REG_H */