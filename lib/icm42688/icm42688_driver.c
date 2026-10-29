/**
 * @file icm42688_driver.c
 * @brief ICM42688 IMU driver implementation
 *
 * This file implements the complete driver for the ICM42688 6-axis IMU sensor,
 * supporting both I2C and SPI communication interfaces. It provides functions
 * for initialization, configuration, data reading, FIFO management, and
 * interrupt handling.
 *
 * Features:
 * - Multi-interface support (I2C/SPI)
 * - Configurable gyroscope and accelerometer ranges
 * - FIFO buffer management
 * - Hardware interrupt configuration
 * - Low-power modes
 * - Self-test capabilities
 *
 * @author Frédéric Marçon
 * @date 2026
 * @copyright Academic Free License v3.0
 */

#include "icm42688_driver.h"
#include "hardware/gpio.h"
#include "icm42688_register.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*============================================================================
 * MACROS AND CONSTANTS
 *============================================================================*/

/* --- fifo datasizes (see docs) --- */
#define FIFO_MAX_DATA_SIZE 2048 /*Bytes*/
#define FIFO_PACKET_1_SIZE 8    /*Bytes*/
#define FIFO_PACKET_2_SIZE 8    /*Bytes*/
#define FIFO_PACKET_3_SIZE 16   /*Bytes*/
#define FIFO_PACKET_4_SIZE 20   /*Bytes*/

#define SENSOR_STARTUP_DELAY_MS 200
#define PLL_READY_MAX_RETRIES   3
#define PLL_DELAY_MS            100

/* These private variable are hardcoded and can be changed here see icm_init_* */
static bool _fifo_count_bigendian  = false;
static bool _sensor_data_bigendian = false;
static bool _fifo_count_in_rec     = true;

/*============================================================================
 * I2C BUS IMPLEMENTATION
 *============================================================================*/

static bool i2c_read_regs(icm42688_connection_handler_t* dev, uint8_t reg, uint8_t* buf,
                          size_t len) {
    if (i2c_write_blocking(dev->i2c, dev->i2c_addr, &reg, 1, true) != 1)
        return false;
    return i2c_read_blocking(dev->i2c, dev->i2c_addr, buf, len, false) == (int)len;
}

static bool i2c_write_reg(icm42688_connection_handler_t* dev, uint8_t reg, uint8_t val) {
    uint8_t buf[2] = {reg, val};
    return i2c_write_blocking(dev->i2c, dev->i2c_addr, buf, 2, false) == 2;
}

static bool i2c_write_reg_with_mask(icm42688_connection_handler_t* dev, uint8_t reg, uint8_t val,
                                    uint8_t mask) {
    uint8_t current = 0;
    if (!i2c_read_regs(dev, reg, &current, 1))
        return false;
    current = (current & ~mask) | (val & mask);
    return i2c_write_reg(dev, reg, current);
}

/*============================================================================
 * SPI BUS IMPLEMENTATION
 *============================================================================*/

static bool spi_read_regs(icm42688_connection_handler_t* dev, uint8_t reg, uint8_t* buf,
                          size_t len) {
    uint8_t cmd = reg | 0x80; /* Set read bit */
    gpio_put(dev->spi_cs_pin, 0);
    spi_write_blocking(dev->spi, &cmd, 1);
    spi_read_blocking(dev->spi, 0, buf, len);
    gpio_put(dev->spi_cs_pin, 1);
    return true;
}

static bool spi_write_reg(icm42688_connection_handler_t* dev, uint8_t reg, uint8_t val) {
    uint8_t buf[2] = {reg & 0x7F, val}; /* Clear read bit */
    gpio_put(dev->spi_cs_pin, 0);
    spi_write_blocking(dev->spi, buf, 2);
    gpio_put(dev->spi_cs_pin, 1);
    return true;
}

static bool spi_write_reg_with_mask(icm42688_connection_handler_t* dev, uint8_t reg, uint8_t val,
                                    uint8_t mask) {
    uint8_t current = 0;
    if (!spi_read_regs(dev, reg, &current, 1))
        return false;
    current = (current & ~mask) | (val & mask);
    return spi_write_reg(dev, reg, current);
}

/*============================================================================
 * BUS INITIALIZATION
 *============================================================================*/

icm42688_status_t icm42688_init_i2c(icm42688_connection_handler_t* dev, i2c_inst_t* i2c,
                                    uint8_t addr, const icm42688_config_t* config) {
    if (dev == NULL || i2c == NULL || config == NULL) {
        return ICM42688_ERR_NULL_PTR;
    }

    dev->bus_type = ICM42688_BUS_I2C; /* Not yet used */
    dev->i2c      = i2c;
    dev->i2c_addr = addr;
    dev->spi      = NULL;

    /* Store configuration */
    dev->config                 = *config;
    dev->fifo_count_endian_big  = _fifo_count_bigendian;
    dev->sensor_data_endian_big = _sensor_data_bigendian;
    dev->fifo_count_in_rec      = _fifo_count_in_rec;

    /* Attach I2C functions */
    dev->read_regs      = i2c_read_regs;
    dev->write_reg      = i2c_write_reg;
    dev->write_reg_mask = i2c_write_reg_with_mask;

    if (!icm42688_is_connected(dev)) {
        return ICM42688_ERR_NOT_FOUND;
    }

    return ICM42688_OK;
}

icm42688_status_t icm42688_init_spi(icm42688_connection_handler_t* dev, spi_inst_t* spi,
                                    uint8_t cs_pin, const icm42688_config_t* config) {
    if (dev == NULL || spi == NULL || config == NULL) {
        return ICM42688_ERR_NULL_PTR;
    }

    dev->bus_type   = ICM42688_BUS_SPI; /* Not yet used */
    dev->spi        = spi;
    dev->spi_cs_pin = cs_pin;
    dev->i2c        = NULL;

    /* Store configuration */
    dev->config                 = *config;
    dev->fifo_count_endian_big  = _fifo_count_bigendian;
    dev->sensor_data_endian_big = _sensor_data_bigendian;
    dev->fifo_count_in_rec      = _fifo_count_in_rec;

    /* Attach SPI functions */
    dev->read_regs      = spi_read_regs;
    dev->write_reg      = spi_write_reg;
    dev->write_reg_mask = spi_write_reg_with_mask;

    if (!icm42688_is_connected(dev)) {
        return ICM42688_ERR_NOT_FOUND;
    }

    return ICM42688_OK;
}

/*============================================================================
 * STATIC HELPER FUNCTIONS
 *============================================================================*/

/**
 * @brief Calculate LSB (Least Significant Bit) conversion factors for ICM42688 sensor data
 *
 * This function computes the conversion factors needed to translate raw sensor readings
 * into physical units (g for accelerometer, degrees per second for gyroscope). It handles
 * both register data and FIFO data, with special consideration for FIFO high-resolution mode.
 *
 * @param[in] dev Pointer to the ICM42688 connection handler containing sensor configuration
 * @param[out] lsb Pointer to structure where conversion factors will be stored
 *
 * @return icm42688_status_t Status code:
 *         - ICM42688_OK: Conversion factors calculated successfully
 *         - ICM42688_ERR_INVALID_ARG: NULL pointer provided or invalid range configuration
 *
 * @note In FIFO high-resolution mode, fixed conversion factors are used for FIFO data
 *       (1/8192 g/LSB for accel, 1/131 dps/LSB for gyro), while register data uses
 *       range-dependent scales.
 * @note In normal mode, both FIFO and register data use the same range-dependent scales.
 *
 * @see icm42688_conversion_factors_t
 * @see icm42688_connection_handler_t
 */
static icm42688_status_t icm42688_get_lsb(icm42688_connection_handler_t* dev,
                                          icm42688_conversion_factors_t* lsb) {
    if (dev == NULL || lsb == NULL) {
        return ICM42688_ERR_INVALID_ARG;
    }
    if (dev->config.fifo_hires) { /* p37 sensitivity scale for fifo hi-res*/
        lsb->fifo_accel_g_per_lsb = 1.0f / 8192.0f;
        lsb->fifo_gyro_g_per_lsb  = 1.0f / 131.0f;
    }

    /* Register data always uses range-dependent scale, regardless of FIFO high-res mode */
    switch (dev->config.accel_range) {
    case ICM42688_ACCEL_RANGE_2G:
        lsb->fs_accel_g_per_lsb = 1.0f / 16384.0f;
        break;
    case ICM42688_ACCEL_RANGE_4G:
        lsb->fs_accel_g_per_lsb = 1.0f / 8192.0f;
        break;
    case ICM42688_ACCEL_RANGE_8G:
        lsb->fs_accel_g_per_lsb = 1.0f / 4096.0f;
        break;
    case ICM42688_ACCEL_RANGE_16G:
        lsb->fs_accel_g_per_lsb = 1.0f / 2048.0f;
        break;
    default:
        return ICM42688_ERR_INVALID_ARG;
    }

    /* If not using FIFO hi-res mode, use the same scale for FIFO */
    if (!dev->config.fifo_hires) {
        lsb->fifo_accel_g_per_lsb = lsb->fs_accel_g_per_lsb;
    }

    switch (dev->config.gyro_range) {
    case ICM42688_GYRO_RANGE_15_625DPS:
        lsb->fs_gyro_g_per_lsb = 1.0f / 2097.2f;
        break;
    case ICM42688_GYRO_RANGE_31_25DPS:
        lsb->fs_gyro_g_per_lsb = 1.0f / 1048.6f;
        break;
    case ICM42688_GYRO_RANGE_62_5DPS:
        lsb->fs_gyro_g_per_lsb = 1.0f / 524.3f;
        break;
    case ICM42688_GYRO_RANGE_125DPS:
        lsb->fs_gyro_g_per_lsb = 1.0f / 262.0f;
        break;
    case ICM42688_GYRO_RANGE_250DPS:
        lsb->fs_gyro_g_per_lsb = 1.0f / 131.0f;
        break;
    case ICM42688_GYRO_RANGE_500DPS:
        lsb->fs_gyro_g_per_lsb = 1.0f / 65.5f;
        break;
    case ICM42688_GYRO_RANGE_1000DPS:
        lsb->fs_gyro_g_per_lsb = 1.0f / 32.8f;
        break;
    case ICM42688_GYRO_RANGE_2000DPS:
        lsb->fs_gyro_g_per_lsb = 1.0f / 16.4f;
        break;
    default:
        return ICM42688_ERR_INVALID_ARG;
    }

    /* If not using FIFO hi-res mode, use the same scale for FIFO */
    if (!dev->config.fifo_hires) {
        lsb->fifo_gyro_g_per_lsb = lsb->fs_gyro_g_per_lsb;
    }

    return ICM42688_OK;
};

/**
 * @brief Checks if the configured FIFO watermark is compatible with the maximum FIFO size.
 *
 * This function validates that the product of the configured FIFO watermark and the
 * packet size does not exceed the maximum FIFO data size. The packet size depends on
 * whether high-resolution mode is enabled.
 *
 * @note This implementation assumes both gyroscope and accelerometer are enabled by
 *       default, therefore only Packet 3 (16 bytes) and Packet 4 (20 bytes) formats
 *       are supported. Packet 1 (gyro only, 8 bytes) and Packet 2 (accel only, 8 bytes)
 *       are not handled.
 *
 * @param[in]  dev             Pointer to the ICM42688 device connection handler structure.
 * @param[out] is_compatible   Pointer to a boolean that will be set to true if the
 *                             watermark configuration is compatible, false otherwise.
 *
 * @return icm42688_status_t
 *         - ICM42688_OK: Operation completed successfully
 *         - ICM42688_ERR_INVALID_ARG: NULL pointer provided for dev or is_compatible
 *
 * @see FIFO_PACKET_3_SIZE, FIFO_PACKET_4_SIZE, FIFO_MAX_DATA_SIZE
 * @todo We were too lazy to code a complete check of all possible packet formats so feel free to do
 * it
 */
static icm42688_status_t icm42688_check_watermark(icm42688_connection_handler_t* dev,
                                                  bool*                          is_compatible) {
    if (dev == NULL || is_compatible == NULL) {
        return ICM42688_ERR_INVALID_ARG;
    }
    if (dev->config.fifo_hires) { /* Packet 4*/
        *is_compatible = ((dev->config.fifo_watermark * FIFO_PACKET_4_SIZE) <= FIFO_MAX_DATA_SIZE);
        dev->config.fifo_packet_size = FIFO_PACKET_4_SIZE;
    } else { /* Packet 3 */
        *is_compatible = ((dev->config.fifo_watermark * FIFO_PACKET_3_SIZE) <= FIFO_MAX_DATA_SIZE);
        dev->config.fifo_packet_size = FIFO_PACKET_3_SIZE;
    }
    return ICM42688_OK;
};

/**
 * @brief Selects the register bank for the ICM42688 device.
 *
 * This function writes the specified bank value to the REG_BANK_SEL register,
 * allowing access to different register banks within the ICM42688 sensor.
 * Isn't it an awesome function? It is so simple and elegant that it deserves its own documentation
 * block!
 * @param dev Pointer to the ICM42688 connection handler structure.
 * @param bank The register bank to select.
 */
static inline void icm42688_select_bank(icm42688_connection_handler_t* dev, uint8_t bank) {
    dev->write_reg(dev, B0_REG_REG_BANK_SEL, bank);
}

/*============================================================================
 * 3dB FILTER CONFIGURATION (STATIC)
 *============================================================================*/

/**
 * @brief ICM42688 3dB filter parameters structure
 *
 * Structure containing the Anti-Aliasing Filter (AAF) configuration parameters
 * for both accelerometer and gyroscope of the ICM42688 IMU sensor.
 *
 * @note These parameters are used to configure the sensor's digital low-pass filter
 *       to achieve a specific 3dB bandwidth.
 */
typedef struct {
    int accel_aaf_delt;
    int accel_aaf_deltsqr;
    int accel_aaf_bitshift;
    int gyro_aaf_delt;
    int gyro_aaf_deltsqr;
    int gyro_aaf_bitshift;
} icm42688_3dBfilter_params_t;

/**
 * @brief Lookup table containing 3dB filter parameters for the ICM42688 sensor.
 *
 * This table stores filter configuration parameters used to configure the
 * digital low-pass filter (DLPF) characteristics for the ICM42688 IMU.
 * Each entry defines the filter response at specific cutoff frequencies.
 * It was pain in the ass to code and check this.
 */
static const icm42688_3dBfilter_params_t icm42688_filter_params[] = {
    {1, 1, 15, 1, 1, 5},        /* FILTER_3dB_BW_42HZ */
    {2, 4, 13, 2, 4, 13},       /* FILTER_3dB_BW_84HZ */
    {3, 9, 12, 3, 9, 12},       /* FILTER_3dB_BW_126HZ */
    {4, 16, 11, 4, 16, 11},     /* FILTER_3dB_BW_170HZ */
    {5, 25, 10, 5, 25, 10},     /* FILTER_3dB_BW_213HZ */
    {6, 36, 10, 6, 36, 10},     /* FILTER_3dB_BW_258HZ */
    {7, 49, 9, 7, 49, 9},       /* FILTER_3dB_BW_303HZ */
    {8, 64, 9, 8, 64, 9},       /* FILTER_3dB_BW_348HZ */
    {9, 81, 9, 9, 81, 9},       /* FILTER_3dB_BW_394HZ */
    {10, 100, 8, 10, 100, 8},   /* FILTER_3dB_BW_441HZ */
    {11, 122, 8, 11, 122, 8},   /* FILTER_3dB_BW_488HZ */
    {12, 144, 8, 12, 144, 8},   /* FILTER_3dB_BW_536HZ */
    {13, 170, 8, 13, 170, 8},   /* FILTER_3dB_BW_585HZ */
    {14, 196, 7, 14, 196, 7},   /* FILTER_3dB_BW_634HZ */
    {15, 224, 7, 15, 224, 7},   /* FILTER_3dB_BW_684HZ */
    {16, 256, 7, 16, 256, 7},   /* FILTER_3dB_BW_734HZ, */
    {17, 288, 7, 17, 288, 7},   /* FILTER_3dB_BW_785HZ, */
    {18, 324, 7, 18, 324, 7},   /* FILTER_3dB_BW_837HZ, */
    {19, 360, 6, 19, 360, 6},   /* FILTER_3dB_BW_890HZ, */
    {20, 400, 6, 20, 400, 6},   /* FILTER_3dB_BW_943HZ, */
    {21, 440, 6, 21, 440, 6},   /* FILTER_3dB_BW_997HZ, */
    {22, 488, 6, 22, 488, 6},   /* FILTER_3dB_BW_1051HZ, */
    {23, 528, 6, 23, 528, 6},   /* FILTER_3dB_BW_1107HZ, */
    {24, 576, 6, 24, 576, 6},   /* FILTER_3dB_BW_1163HZ, */
    {25, 624, 5, 25, 624, 5},   /* FILTER_3dB_BW_1220HZ, */
    {26, 680, 5, 26, 680, 5},   /* FILTER_3dB_BW_1277HZ, */
    {27, 736, 5, 27, 736, 5},   /* FILTER_3dB_BW_1336HZ, */
    {28, 784, 5, 28, 784, 5},   /* FILTER_3dB_BW_1395HZ, */
    {29, 848, 5, 29, 848, 5},   /* FILTER_3dB_BW_1454HZ, */
    {30, 896, 5, 30, 896, 5},   /* FILTER_3dB_BW_1515HZ, */
    {31, 960, 5, 31, 960, 5},   /* FILTER_3dB_BW_1577HZ */
    {32, 1024, 5, 32, 1024, 5}, /* FILTER_3dB_BW_1639HZ */
    {33, 1088, 5, 33, 1088, 5}, /* FILTER_3dB_BW_1702HZ */
    {34, 1152, 5, 34, 1152, 5}, /* FILTER_3dB_BW_1766HZ */
    {35, 1232, 5, 35, 1232, 5}, /* FILTER_3dB_BW_1830HZ */
    {36, 1296, 5, 36, 1296, 5}, /* FILTER_3dB_BW_1896HZ */
    {37, 1376, 4, 37, 1376, 4}, /* FILTER_3dB_BW_1962HZ */
    {38, 1440, 4, 38, 1440, 4}, /* FILTER_3dB_BW_2029HZ */
    {39, 1536, 4, 39, 1536, 4}, /* FILTER_3dB_BW_2097HZ */
    {40, 1600, 4, 40, 1600, 4}, /* FILTER_3dB_BW_2166HZ */
    {41, 1696, 4, 41, 1696, 4}, /* FILTER_3dB_BW_2235HZ */
    {42, 1760, 4, 42, 1760, 4}, /* FILTER_3dB_BW_2306HZ */
    {43, 1856, 4, 43, 1856, 4}, /* FILTER_3dB_BW_2377HZ */
    {44, 1952, 4, 44, 1952, 4}, /* FILTER_3dB_BW_2449HZ */
    {45, 2016, 4, 45, 2016, 4}, /* FILTER_3dB_BW_2522HZ */
    {46, 2112, 4, 46, 2112, 4}, /* FILTER_3dB_BW_2596HZ */
    {47, 2208, 4, 47, 2208, 4}, /* FILTER_3dB_BW_2671HZ */
    {48, 2304, 4, 48, 2304, 4}, /* FILTER_3dB_BW_2746HZ */
    {49, 2400, 4, 49, 2400, 4}, /* FILTER_3dB_BW_2823HZ */
    {50, 2496, 4, 50, 2496, 4}, /* FILTER_3dB_BW_2900HZ */
    {51, 2592, 4, 51, 2592, 4}, /* FILTER_3dB_BW_2978HZ */
    {52, 2720, 4, 52, 2720, 4}, /* FILTER_3dB_BW_3057HZ */
    {53, 2816, 3, 53, 2816, 3}, /* FILTER_3dB_BW_3137HZ */
    {54, 2944, 3, 54, 2944, 3}, /* FILTER_3dB_BW_3217HZ */
    {55, 3008, 3, 55, 3008, 3}, /* FILTER_3dB_BW_3299HZ */
    {56, 3136, 3, 56, 3136, 3}, /* FILTER_3dB_BW_3381HZ */
    {57, 3264, 3, 57, 3264, 3}, /* FILTER_3dB_BW_3464HZ */
    {58, 3392, 3, 58, 3392, 3}, /* FILTER_3dB_BW_3548HZ */
    {59, 3456, 3, 59, 3456, 3}, /* FILTER_3dB_BW_3633HZ */
    {60, 3584, 3, 60, 3584, 3}, /* FILTER_3dB_BW_3718HZ */
    {61, 3712, 3, 61, 3712, 3}, /* FILTER_3dB_BW_3805HZ */
    {62, 3840, 3, 62, 3840, 3}, /* FILTER_3dB_BW_3892HZ */
    {63, 3968, 3, 63, 3968, 3}  /* FILTER_3dB_BW_3979HZ */
};

/**
 * @brief Configures the 3dB anti-aliasing filter bandwidth for the ICM42688
 * sensor.
 *
 * @details This is a static function used internally within this translation
 * unit. It sets up the anti-aliasing filter (AAF) parameters based on the
 * specified bandwidth configuration. The function validates input parameters
 * and retrieves the appropriate filter parameters from the lookup table.
 *
 * @param[in] handle Pointer to the ICM42688 connection handler structure
 * @param[in] config Pointer to the ICM42688 configuration structure containing
 *                   the desired AAF filter bandwidth setting
 *
 * @return icm42688_status_t Status code indicating success or failure
 * @todo Check bitshifts / masks
 */
static icm42688_status_t icm42688_set_3dB_filter(icm42688_connection_handler_t* dev) {
    if (dev == NULL) {
        return ICM42688_ERR_NULL_PTR;
    }

    if (dev->config.aaf_filter_bw > ICM42688_FILTER_3dB_BW_3979HZ) {
        return ICM42688_ERR_INVALID_ARG;
    }

    icm42688_3dBfilter_params_t params = icm42688_filter_params[dev->config.aaf_filter_bw];
    /* TODO: Check if the filter is correctly configured */
    /* Configure AAF (Anti-Aliasing Filter) for Gyro and Accel */
    /* --- GYRO --- */

    icm42688_select_bank(dev, BANK_SEL_BANK_1); /*B1*/
    dev->write_reg_mask(dev, B1_REG_GYRO_CONFIG_STATIC3, (uint8_t)params.gyro_aaf_delt,
                        GYRO_CONFIG_STATIC3_MASK_GYRO_AAF_DELT);

    dev->write_reg_mask(dev, B1_REG_GYRO_CONFIG_STATIC4, (uint8_t)(params.gyro_aaf_deltsqr & 0xFF),
                        GYRO_CONFIG_STATIC4_MASK_GYRO_AAF_DELTSQR_7_0);
    dev->write_reg_mask(dev, B1_REG_GYRO_CONFIG_STATIC5,
                        (uint8_t)((params.gyro_aaf_deltsqr >> 8) & 0x0F),
                        GYRO_CONFIG_STATIC5_MASK_GYRO_AAF_DELTSQR_11_8);

    dev->write_reg_mask(dev, B1_REG_GYRO_CONFIG_STATIC5,
                        (uint8_t)((params.gyro_aaf_bitshift << 4) & 0xF0),
                        GYRO_CONFIG_STATIC5_MASK_GYRO_AAF_BITSHIFT);
    /* --- ACCEL --- */
    icm42688_select_bank(dev, BANK_SEL_BANK_2); /*B2*/
    dev->write_reg_mask(dev, B2_REG_ACCEL_CONFIG_STATIC2, (uint8_t)(params.accel_aaf_delt << 1),
                        ACCEL_CONFIG_STATIC2_MASK_ACCEL_AAF_DELT);

    dev->write_reg_mask(dev, B2_REG_ACCEL_CONFIG_STATIC3,
                        (uint8_t)(params.accel_aaf_deltsqr & 0xFF),
                        ACCEL_CONFIG_STATIC3_MASK_ACCEL_AAF_DELTSQR_7_0);
    dev->write_reg_mask(dev, B2_REG_ACCEL_CONFIG_STATIC4,
                        (uint8_t)((params.accel_aaf_deltsqr >> 8) & 0x0F),
                        ACCEL_CONFIG_STATIC4_MASK_ACCEL_AAF_DELTSQR_11_8);

    dev->write_reg_mask(dev, B2_REG_ACCEL_CONFIG_STATIC4,
                        (uint8_t)((params.accel_aaf_bitshift << 4) & 0xF0),
                        ACCEL_CONFIG_STATIC4_MASK_ACCEL_AAF_BITSHIFT);
    /* --- AAF ENABLE AFTER CONFIG --- */
    dev->write_reg_mask(dev, B2_REG_ACCEL_CONFIG_STATIC2,
                        ACCEL_CONFIG_STATIC2_VALUE_ACCEL_AAF_ENABLE,
                        ACCEL_CONFIG_STATIC2_MASK_ACCEL_AAF_DISABLE);

    icm42688_select_bank(dev, BANK_SEL_BANK_1); /*B1*/
    dev->write_reg_mask(dev, B1_REG_GYRO_CONFIG_STATIC2, GYRO_CONFIG_STATIC2_VALUE_GYRO_AAF_ENABLE,
                        GYRO_CONFIG_STATIC2_MASK_GYRO_AAF_DISABLE);
    /* --- NF DISABLE --- */
    dev->write_reg_mask(dev, B1_REG_GYRO_CONFIG_STATIC2, GYRO_CONFIG_STATIC2_VALUE_GYRO_NF_DISABLE,
                        GYRO_CONFIG_STATIC2_MASK_GYRO_NF_DISABLE);

    /* Configure UI filter */
    icm42688_select_bank(dev, BANK_SEL_BANK_0); /*B0*/
    dev->write_reg_mask(dev, B0_REG_GYRO_CONFIG1, (uint8_t)(dev->config.ui_filter_order),
                        GYRO_CONFIG1_MASK_GYRO_UI_FILT_ORD);
    dev->write_reg_mask(dev, B0_REG_GYRO_ACCEL_CONFIG0,
                        (uint8_t)(dev->config.ui_filter_number & 0x0F),
                        GYRO_ACCEL_CONFIG0_MASK_GYRO_UI_FILT_BW);
    dev->write_reg_mask(dev, B0_REG_ACCEL_CONFIG1, (uint8_t)(dev->config.ui_filter_order),
                        ACCEL_CONFIG1_MASK_ACCEL_UI_FILT_ORD);
    dev->write_reg_mask(dev, B0_REG_GYRO_ACCEL_CONFIG0,
                        ((uint8_t)(dev->config.ui_filter_number) << 4) & 0xF0,
                        GYRO_ACCEL_CONFIG0_MASK_ACCEL_UI_FILT_BW);

    return ICM42688_OK;
}

/*============================================================================
 * CLOCK CONFIGURATION (STATIC)
 *============================================================================*/

/**
 * @brief Configures the clock input source for the ICM42688 device.
 *
 * This function sets up the device to use either an external clock input (CLKIN) or the internal
 * clock, depending on the configuration in the device handler. If external clocking is requested,
 * it attempts to enable and verify the PLL_RDY interrupt, falling back to the internal clock if the
 * PLL does not become ready. The function also configures the appropriate registers for interrupt
 * mapping and clock selection. If Clockin is used (FSY) pin can't be used as interrupt pin.
 *
 * @param[in,out] dev Pointer to the device connection handler structure.
 *
 * @return ICM42688_OK on success,
 *         ICM42688_ERR_NULL_PTR if the device pointer is NULL,
 *         ICM42688_ERR_CLOCKIN_CONFIG if external clocking was requested but PLL did not become
 * ready.
 */
static icm42688_status_t icm42688_configure_clockin(icm42688_connection_handler_t* dev) {
    if (dev == NULL) {
        return ICM42688_ERR_NULL_PTR;
    }

    bool    pll_ready   = false;
    uint8_t int_source0 = 0U;
    bool    use_ext     = dev->config.use_external_clkin;

    if (use_ext) {
        /* unmap all interrupts from INT2 */
        icm42688_select_bank(dev, BANK_SEL_BANK_0); /*B0*/

        dev->write_reg(dev, B0_REG_INT_SOURCE3, 0b0);

        /* Disable APEX interrupts from INT2 */
        dev->write_reg(dev, B0_REG_INT_SOURCE4, 0b0);

        /* Set PIN9 as CLKIN input */
        icm42688_select_bank(dev, BANK_SEL_BANK_1); /*B1*/
        dev->write_reg_mask(dev, B1_REG_INTF_CONFIG5, INTF_CONFIG5_MASK_PIN9_FUNCTION,
                            INTF_CONFIG5_VALUE_PIN9_FUNCTION_CLKIN);

        /* Enable PLL_RDY interrupt in int1 temporarily */
        icm42688_select_bank(dev, BANK_SEL_BANK_0); /*B0*/

        dev->write_reg_mask(dev, B0_REG_INT_CONFIG,
                            INT_CONFIG_MASK_INT1_MODE | INT_CONFIG_MASK_INT1_DRIVE_CIRCUIT |
                                INT_CONFIG_MASK_INT1_POLARITY,
                            INT_CONFIG_VALUE_INT1_MODE_LATCHED |
                                INT_CONFIG_VALUE_INT1_DRIVE_CIRCUIT_PUSH_PULL |
                                INT_CONFIG_VALUE_INT1_POLARITY_HIGH);

        dev->write_reg_mask(dev, B0_REG_INT_CONFIG1, INT_CONFIG1_MASK_INT_ASYNC_RESET,
                            INT_CONFIG1_VALUE_INT_ASYNC_RESET_INT);

        dev->write_reg_mask(dev, B0_REG_INT_SOURCE0, INT_SOURCE0_MASK_PLL_RDY_INT1_EN,
                            INT_SOURCE0_VALUE_PLL_RDY_INT1_EN_ENABLE);

        /* Wait for PLL_RDY interrupt (bit 5 of INT_STATUS) */
        uint8_t status = 0;
        for (int i = 0; i < PLL_READY_MAX_RETRIES; ++i) {
            sleep_ms(PLL_DELAY_MS);
            dev->read_regs(dev, B0_REG_INT_STATUS, &status, 1);
            if (status & (1 << 5)) {
                pll_ready = true;
                break;
            }
        }

        if (!pll_ready) {
            use_ext = false; /* FALL BACK TO INTERNAL CLOCK */
        }

        /* Clear PLL_RDY interrupt */
        dev->write_reg_mask(dev, B0_REG_INT_SOURCE0, INT_SOURCE0_MASK_PLL_RDY_INT1_EN,
                            INT_SOURCE0_VALUE_PLL_RDY_INT1_EN_DISABLE);
    }

    /*  INTF_CONFIG1 to apply final clock configuration */
    icm42688_select_bank(dev, BANK_SEL_BANK_0); /*B0*/

    dev->write_reg_mask(dev, B0_REG_INTF_CONFIG1, INTF_CONFIG1_MASK_RTC_MODE,
                        use_ext ? INTF_CONFIG1_VALUE_RTC_MODE_REQ
                                : INTF_CONFIG1_VALUE_RTC_MODE_DIS);

    dev->write_reg_mask(dev, B0_REG_INTF_CONFIG1, INTF_CONFIG1_MASK_CLKSEL,
                        use_ext ? INTF_CONFIG1_VALUE_CLKSEL_PLL
                                : INTF_CONFIG1_VALUE_CLKSEL_INTERNAL);

    return (dev->config.use_external_clkin && !pll_ready) ? ICM42688_ERR_CLOCKIN_CONFIG
                                                          : ICM42688_OK;
};

/*============================================================================
 * PUBLIC FUNCTIONS - SENSOR CONFIGURATION AND CONTROL
 *============================================================================*/

bool icm42688_is_connected(icm42688_connection_handler_t* dev) {
    if (dev == NULL || dev->read_regs == NULL) {
        return false;
    }
    uint8_t whoami = 0;
    if (!dev->read_regs(dev, B0_REG_WHO_AM_I, &whoami, 1)) {
        return false;
    }
    return whoami == ICM42688_WHOAMI_VALUE;
}

icm42688_status_t icm42688_configure_accel_gyro(icm42688_connection_handler_t* dev) {
    if (dev == NULL) {
        return ICM42688_ERR_NULL_PTR;
    }
    icm42688_configure_clockin(dev);
    icm42688_set_3dB_filter(dev);

    icm42688_select_bank(dev, BANK_SEL_BANK_0); /*B0*/
    /* --- ACCEL --- */
    dev->write_reg_mask(dev, B0_REG_ACCEL_CONFIG0,
                        (uint8_t)(dev->config.accel_odr) | (uint8_t)(dev->config.accel_range),
                        ACCEL_CONFIG0_MASK_ACCEL_ODR | ACCEL_CONFIG0_MASK_ACCEL_RANGE);
    /* --- GYRO --- */
    dev->write_reg_mask(dev, B0_REG_GYRO_CONFIG0,
                        (uint8_t)(dev->config.gyro_odr) | (uint8_t)(dev->config.gyro_range),
                        GYRO_CONFIG0_MASK_GYRO_ODR | GYRO_CONFIG0_MASK_GYRO_RANGE);
    /* --- FILTER ---*/
    icm42688_status_t status = icm42688_set_3dB_filter(dev);
    if (status != ICM42688_OK) {
        return status;
    }
    return ICM42688_OK;
};

icm42688_status_t icm42688_configure_fifo(icm42688_connection_handler_t* dev) {
    if (dev == NULL) {
        return ICM42688_ERR_NULL_PTR;
    }
    icm42688_select_bank(dev, BANK_SEL_BANK_0); /*B0*/

    /* --- FIFO MODE --- */
    dev->write_reg_mask(dev, B0_REG_FIFO_CONFIG, (uint8_t)(dev->config.fifo_mode),
                        FIFO_CONFIG_MASK_MODE);

    /* --- FIFO DATA FORMAT --- */
    uint8_t config1_val = 0;
    config1_val |= dev->config.fifo_hires
                       ? FIFO_CONFIG1_VALUE_FIFO_HIRES_EN_ENABLE
                       : FIFO_CONFIG1_VALUE_FIFO_HIRES_EN_DISABLE; /*Enable 3 bytes HiRes*/
    config1_val |= FIFO_CONFIG1_VALUE_FIFO_ACCEL_EN_ENABLE;        /* Enable Accel in fifo */
    config1_val |= FIFO_CONFIG1_VALUE_FIFO_GYRO_EN_ENABLE;         /* Enable Gyro in fifo */
    /* ACCEL AND GYRO WERE HARDCODED ENABLED because have not so much
     time to code packet size calculation and things related...*/
    config1_val |= dev->config.temp_in_fifo
                       ? FIFO_CONFIG1_VALUE_FIFO_TEMP_EN_ENABLE
                       : FIFO_CONFIG1_VALUE_FIFO_TEMP_EN_DISABLE; /* Enable temperature */
    config1_val |= dev->config.fsync_in_fifo
                       ? FIFO_CONFIG1_VALUE_FIFO_TMST_FSYNC_EN_ENABLE   /* fsync and odr timestamp*/
                       : FIFO_CONFIG1_VALUE_FIFO_TMST_FSYNC_EN_DISABLE; /* only odr timestamp*/

    config1_val |= FIFO_CONFIG1_VALUE_FIFO_RESUME_PARTIAL_RD_DISABLE; /*TEST: check cause fifo read
                                                                         can be partial ???*/
    config1_val |= FIFO_CONFIG1_VALUE_FIFO_WM_GT_TH_DISABLE; /* TODO: DMA : Trigger fifo wm IRQ on
                                                                ODR if FIFO_COUNT > WM*/

    dev->write_reg(dev, B0_REG_FIFO_CONFIG1, config1_val);

    /* --- FIFO COUNT MODE AND ENDIANESS --- */
    uint8_t intf_config0_val = INTF_CONFIG0_VALUE_FIFO_HOLD_LAST_DATA_EN_0;

    /* Configure FIFO count mode based on dev settings */
    intf_config0_val |= dev->fifo_count_in_rec ? INTF_CONFIG0_VALUE_FIFO_COUNT_REC_IN_REC
                                               : INTF_CONFIG0_VALUE_FIFO_COUNT_REC_IN_BYTES;

    /* Configure FIFO count endianness */
    intf_config0_val |= dev->fifo_count_endian_big ? INTF_CONFIG0_VALUE_FIFO_COUNT_ENDIAN_BIG
                                                   : INTF_CONFIG0_VALUE_FIFO_COUNT_ENDIAN_LITTLE;

    /* Configure sensor data endianness */
    intf_config0_val |= dev->sensor_data_endian_big ? INTF_CONFIG0_VALUE_SENSOR_DATA_ENDIAN_BIG
                                                    : INTF_CONFIG0_VALUE_SENSOR_DATA_ENDIAN_LITTLE;

    dev->write_reg(dev, B0_REG_INTF_CONFIG0, intf_config0_val);

    /* --- FIFO WATERMARK --- */
    bool              fifo_wm_ok = false;
    icm42688_status_t status     = icm42688_check_watermark(dev, &fifo_wm_ok);
    if ((status == ICM42688_OK) && fifo_wm_ok) {
        dev->write_reg(dev, B0_REG_FIFO_CONFIG2, dev->config.fifo_watermark & 0xFF);
        dev->write_reg_mask(dev, B0_REG_FIFO_CONFIG3, (dev->config.fifo_watermark >> 8) & 0x0F,
                            FIFO_CONFIG3_MASK_FIFO_WM);
    } else {
        return status;
    }

    return ICM42688_OK;
};

icm42688_status_t icm42688_configure_interrupt(icm42688_connection_handler_t* dev) {
    if (dev == NULL) {
        return ICM42688_ERR_NULL_PTR;
    }
    icm42688_select_bank(dev, BANK_SEL_BANK_0);

    /* INTERRUPT MODE CONFIGURATION */
    dev->write_reg_mask(dev, B0_REG_INT_CONFIG,
                        INT_CONFIG_VALUE_INT1_MODE_PULSE |
                            INT_CONFIG_VALUE_INT1_DRIVE_CIRCUIT_PUSH_PULL |
                            INT_CONFIG_VALUE_INT1_POLARITY_HIGH,
                        INT_CONFIG_MASK_INT1_MODE | INT_CONFIG_MASK_INT1_DRIVE_CIRCUIT |
                            INT_CONFIG_MASK_INT1_POLARITY);
    if (!dev->config.use_external_clkin) { /* INT2 is not available when using external clock */
        dev->write_reg_mask(dev, B0_REG_INT_CONFIG,
                            INT_CONFIG_VALUE_INT2_MODE_PULSE |
                                INT_CONFIG_VALUE_INT2_DRIVE_CIRCUIT_PUSH_PULL |
                                INT_CONFIG_VALUE_INT2_POLARITY_HIGH,
                            INT_CONFIG_MASK_INT2_MODE | INT_CONFIG_MASK_INT2_DRIVE_CIRCUIT |
                                INT_CONFIG_MASK_INT2_POLARITY);
    }

    /* CLEAR IRQ DDRY or THS or FULL ON DATA READ (HARDCODED) */
    dev->write_reg_mask(dev, B0_REG_INT_CONFIG0,
                        INT_CONFIG0_VALUE_UI_DRDY_INT_CLEAR_ON_DATA_READ |
                            INT_CONFIG0_VALUE_FIFO_THS_INT_CLEAR_ON_FIFO_READ |
                            INT_CONFIG0_VALUE_FIFO_FULL_INT_CLEAR_ON_FIFO_READ,
                        INT_CONFIG0_MASK_UI_DRDY_INT_CLEAR_ON |
                            INT_CONFIG0_MASK_FIFO_THS_INT_CLEAR_ON |
                            INT_CONFIG0_MASK_FIFO_FULL_INT_CLEAR_ON);

    /* IRQ PULE DURATION / DEASSERETION (HARDCODED) */
    /* Note: ODR enum values are INVERSELY ordered (smaller value = higher freq).
     * ICM42688_ACCEL_ODR_4KHZ=0x04, ICM42688_ACCEL_ODR_1KHZ=0x06 → use <= for ">= 4kHz" */
    /* I get screwed with this one...*/
    bool is_one_odr_sup_4000 = (dev->config.accel_odr <= ICM42688_ACCEL_ODR_4KHZ) ||
                               (dev->config.gyro_odr <= ICM42688_GYRO_ODR_4KHZ);

    dev->write_reg_mask(dev, B0_REG_INT_CONFIG1,
                        is_one_odr_sup_4000 ? INT_CONFIG1_VALUE_INT_TPULSE_DURATION_8US
                                            : INT_CONFIG1_VALUE_INT_TPULSE_DURATION_100US,
                        INT_CONFIG1_MASK_INT_TPULSE_DURATION);

    dev->write_reg_mask(dev, B0_REG_INT_CONFIG1,
                        is_one_odr_sup_4000 ? INT_CONFIG1_VALUE_INT_TDEASSERT_DISABLE_ON
                                            : INT_CONFIG1_VALUE_INT_TDEASSERT_DISABLE_100US,
                        INT_CONFIG1_MASK_INT_TDEASSERT_DISABLE);

    dev->write_reg_mask(dev, B0_REG_INT_CONFIG1, INT_CONFIG1_VALUE_INT_ASYNC_RESET_INT,
                        INT_CONFIG1_MASK_INT_ASYNC_RESET); /* NEEDED 0->1 for int1 and 2 op but
                                                              already default setting */

    /* INTERRUPT MAPPING (HARDCODED) */
    dev->write_reg_mask(dev, B0_REG_INT_SOURCE0, INT_SOURCE0_VALUE_FIFO_THS_INT1_EN_ENABLE,
                        INT_SOURCE0_MASK_FIFO_THS_INT1_EN);

    if (!dev->config.use_external_clkin) { /* INT2 is not available if external clock is used */
        dev->write_reg(dev, B0_REG_INT_SOURCE3, INT_SOURCE3_VALUE_FIFO_FULL_INT2_EN_ENABLE);
    }
    return ICM42688_OK;
}

icm42688_status_t icm42688_start_sensor(icm42688_connection_handler_t* dev) {
    if (dev == NULL) {
        return ICM42688_ERR_NULL_PTR;
    }

    /* Calculate and store conversion factors in device handler */
    icm42688_status_t status = icm42688_get_lsb(dev, &dev->conversion_factors);
    if (status != ICM42688_OK) {
        return status;
    }

    icm42688_select_bank(dev, BANK_SEL_BANK_0); /*B0*/
    dev->write_reg_mask(dev, B0_REG_PWR_MGMT0,
                        PWR_MGMT0_VALUE_ACCEL_MODE_LOW_NOISE | PWR_MGMT0_VALUE_GYRO_MODE_LOW_NOISE,
                        PWR_MGMT0_MASK_ACCEL_MODE | PWR_MGMT0_MASK_GYRO_MODE);
    sleep_ms(SENSOR_STARTUP_DELAY_MS);
    return ICM42688_OK;
}

icm42688_status_t icm42688_stop_sensor(icm42688_connection_handler_t* dev) {
    if (dev == NULL) {
        return ICM42688_ERR_NULL_PTR;
    }
    icm42688_select_bank(dev, BANK_SEL_BANK_0);
    dev->write_reg_mask(dev, B0_REG_PWR_MGMT0,
                        PWR_MGMT0_VALUE_ACCEL_MODE_OFF0 | PWR_MGMT0_VALUE_GYRO_MODE_OFF0,
                        PWR_MGMT0_MASK_ACCEL_MODE | PWR_MGMT0_MASK_GYRO_MODE);
    sleep_ms(SENSOR_STARTUP_DELAY_MS);
    return ICM42688_OK;
}

/*============================================================================
 * PUBLIC FUNCTIONS - DATA READING FUNCTIONS
 *============================================================================*/
icm42688_status_t icm42688_read_raw_sensor_data(icm42688_connection_handler_t* dev,
                                                icm42688_raw_data_t*           raw_data_handler) {
    if (dev == NULL || raw_data_handler == NULL) {
        return ICM42688_ERR_NULL_PTR;
    }

    icm42688_select_bank(dev, BANK_SEL_BANK_0);

    uint8_t temp_buffer[14];
    if (!dev->read_regs(dev, B0_REG_TEMP_DATA1, &temp_buffer[0], 14)) {
        return ICM42688_ERR_BUS;
    }

    if (dev->sensor_data_endian_big) {
        raw_data_handler->temp   = (int16_t)((temp_buffer[0] << 8) | temp_buffer[1]);
        raw_data_handler->accelX = (int16_t)((temp_buffer[2] << 8) | temp_buffer[3]);
        raw_data_handler->accelY = (int16_t)((temp_buffer[4] << 8) | temp_buffer[5]);
        raw_data_handler->accelZ = (int16_t)((temp_buffer[6] << 8) | temp_buffer[7]);
        raw_data_handler->gyroX  = (int16_t)((temp_buffer[8] << 8) | temp_buffer[9]);
        raw_data_handler->gyroY  = (int16_t)((temp_buffer[10] << 8) | temp_buffer[11]);
        raw_data_handler->gyroZ  = (int16_t)((temp_buffer[12] << 8) | temp_buffer[13]);
    } else {
        raw_data_handler->temp   = (int16_t)((temp_buffer[1] << 8) | temp_buffer[0]);
        raw_data_handler->accelX = (int16_t)((temp_buffer[3] << 8) | temp_buffer[2]);
        raw_data_handler->accelY = (int16_t)((temp_buffer[5] << 8) | temp_buffer[4]);
        raw_data_handler->accelZ = (int16_t)((temp_buffer[7] << 8) | temp_buffer[6]);
        raw_data_handler->gyroX  = (int16_t)((temp_buffer[9] << 8) | temp_buffer[8]);
        raw_data_handler->gyroY  = (int16_t)((temp_buffer[11] << 8) | temp_buffer[10]);
        raw_data_handler->gyroZ  = (int16_t)((temp_buffer[13] << 8) | temp_buffer[12]);
    }

    return ICM42688_OK;
}

icm42688_status_t icm42688_read_and_convert_data(icm42688_connection_handler_t* dev,
                                                 icm42688_converted_data_t*     conv_data_handler) {
    if (dev == NULL || conv_data_handler == NULL) {
        return ICM42688_ERR_NULL_PTR;
    }
    icm42688_raw_data_t raw_data;
    icm42688_status_t   status = icm42688_read_raw_sensor_data(dev, &raw_data);
    if (status != ICM42688_OK) {
        return status;
    }

    /* Use full-scale conversion factors for register data (not FIFO) */
    conv_data_handler->accelX = raw_data.accelX * dev->conversion_factors.fs_accel_g_per_lsb;
    conv_data_handler->accelY = raw_data.accelY * dev->conversion_factors.fs_accel_g_per_lsb;
    conv_data_handler->accelZ = raw_data.accelZ * dev->conversion_factors.fs_accel_g_per_lsb;
    conv_data_handler->gyroX  = raw_data.gyroX * dev->conversion_factors.fs_gyro_g_per_lsb;
    conv_data_handler->gyroY  = raw_data.gyroY * dev->conversion_factors.fs_gyro_g_per_lsb;
    conv_data_handler->gyroZ  = raw_data.gyroZ * dev->conversion_factors.fs_gyro_g_per_lsb;
    conv_data_handler->temp   = (raw_data.temp / 132.48f) + 25.0f;

    return ICM42688_OK;
}

/*============================================================================
 * PUBLIC FUNCTIONS - FIFO OPERATION
 *============================================================================*/

icm42688_status_t icm42688_read_fifo(icm42688_connection_handler_t* dev, uint8_t* fifo_buffer) {
    if (dev == NULL || fifo_buffer == NULL) {
        return ICM42688_ERR_NULL_PTR;
    }

    /* Calculate read length */
    size_t length = (size_t)(dev->config.fifo_watermark * dev->config.fifo_packet_size);
    if (length > FIFO_MAX_DATA_SIZE) {
        return ICM42688_ERR_INVALID_ARG;
    }
    icm42688_select_bank(dev, BANK_SEL_BANK_0);

    if (!dev->read_regs(dev, B0_REG_FIFO_DATA, fifo_buffer, length)) {
        return ICM42688_ERR_BUS;
    }

    return ICM42688_OK;
}

icm42688_status_t icm42688_clear_fifo(icm42688_connection_handler_t* dev) {
    uint16_t length = icm42688_get_fifo_count(dev);
    if (length == 0) {
        return ICM42688_OK;
    }
    uint8_t temp_buffer[length];
    if (!dev->read_regs(dev, B0_REG_FIFO_DATA, temp_buffer, length)) {
        return ICM42688_ERR_BUS;
    }

    return ICM42688_OK;
}

uint16_t icm42688_get_fifo_count(icm42688_connection_handler_t* dev) {
    if (dev == NULL) {
        return 0;
    }

    /* Burst-read COUNTH+COUNTL in one SPI transaction to hold the internal latch.*/
    uint8_t temp_buffer[2];
    if (!dev->read_regs(dev, B0_REG_FIFO_COUNTH, temp_buffer, 2)) {
        return 0;
    }
    /* Read INTF_CONFIG0 from the chip to determine the actual count unit and endianness.
     * The host flag (dev->fifo_count_in_rec) reflects the intended config, but we
     * verify against the live register so a failed write or a reset doesn't
     * silently corrupt every subsequent FIFO_COUNT read. */
    uint8_t intf_cfg0 = 0;
    if (!dev->read_regs(dev, B0_REG_INTF_CONFIG0, &intf_cfg0, 1)) {
        return 0;
    }

    bool chip_count_big_endian = (intf_cfg0 & INTF_CONFIG0_MASK_FIFO_COUNT_ENDIAN) != 0; /* bit 5
                                                                                          */
    bool chip_count_in_rec = (intf_cfg0 & INTF_CONFIG0_MASK_FIFO_COUNT_REC_MODE) != 0;   /*
     bit 6 */

    /* temp_buffer[0]=FIFO_COUNTH  temp_buffer[1]=FIFO_COUNTL
     * Reconstruct the 13-bit raw value respecting FIFO_COUNT_ENDIAN. */
    uint16_t raw = chip_count_big_endian
                       ? (uint16_t)(((temp_buffer[0] & 0x1F) << 8) | temp_buffer[1])
                       : (uint16_t)(((temp_buffer[1] & 0x1F) << 8) | temp_buffer[0]);

    if (chip_count_in_rec) {
        /* Count is already in records — return directly. */
        return raw;
    } else {
        /* Count is in bytes — convert to records using the configured packet size. */
        uint8_t pkt = dev->config.fifo_packet_size;
        if (pkt == 0)
            return 0;
        return raw / pkt;
    }
}

uint16_t icm42688_check_unread_data(icm42688_connection_handler_t* dev) {
    if (dev == NULL) {
        return 0;
    }

    uint8_t temp_buffer[2];
    if (!dev->read_regs(dev, B0_REG_FIFO_LOST_PKT1, &temp_buffer[0], 1)) {
        return 0;
    }
    if (!dev->read_regs(dev, B0_REG_FIFO_LOST_PKT0, &temp_buffer[1], 1)) {
        return 0;
    }
    return (temp_buffer[0] << 8) | temp_buffer[1];
}

/*============================================================================
 * DMA OPERATIONS (SPI ONLY)
 *============================================================================*/

icm42688_status_t icm42688_dma_start_spi_fifo_read(icm42688_connection_handler_t* dev) {
    if (dev == NULL) {
        return ICM42688_ERR_NULL_PTR;
    }

    /* Verify device is configured for SPI */
    if (dev->bus_type != ICM42688_BUS_SPI || dev->spi == NULL) {
        return ICM42688_ERR_INVALID_ARG;
    }
    icm42688_select_bank(dev, BANK_SEL_BANK_0);

    /* Prepare read command (set read bit) */
    uint8_t cmd = B0_REG_FIFO_DATA | 0x80;

    /* Assert CS low */
    gpio_put(dev->spi_cs_pin, 0);

    /* Send read command - DMA will handle reading the response */
    spi_write_blocking(dev->spi, &cmd, 1);

    /* CS remains LOW - DMA will continue reading */
    /* Caller MUST call icm42688_dma_end_spi_fifo_read() after DMA completes */

    return ICM42688_OK;
}

icm42688_status_t icm42688_dma_end_spi_fifo_read(icm42688_connection_handler_t* dev) {
    if (dev == NULL) {
        return ICM42688_ERR_NULL_PTR;
    }

    /* Verify device is configured for SPI */
    if (dev->bus_type != ICM42688_BUS_SPI) {
        return ICM42688_ERR_INVALID_ARG;
    }

    /* Release CS high */
    gpio_put(dev->spi_cs_pin, 1);

    return ICM42688_OK;
}

size_t icm42688_get_required_buffer_size(const icm42688_connection_handler_t* dev) {
    if (dev == NULL) {
        return 0;
    }

    uint8_t packet_size;
    if (dev->config.fifo_hires) {
        packet_size = FIFO_PACKET_4_SIZE; /* 20 bytes */
    } else {
        packet_size = FIFO_PACKET_3_SIZE; /* 16 bytes */
    }
    /* yeah we don't calculate for 3-dof only ! Because we're lazy and we don't allow gyro or accel
     * off ;)*/

    /* Calculate total buffer size: watermark × packet_size */
    size_t buffer_size = (size_t)(dev->config.fifo_watermark * packet_size);

    /* Sanity check: ensure buffer size doesn't exceed FIFO max */
    if (buffer_size > FIFO_MAX_DATA_SIZE) {
        return FIFO_MAX_DATA_SIZE;
    }

    return buffer_size;
}

#ifdef __cplusplus
}
#endif
/* I put a comment here to prove that maybe a human has open the file and write this line */
