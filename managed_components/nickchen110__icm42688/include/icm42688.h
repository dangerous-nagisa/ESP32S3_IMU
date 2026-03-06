/*
 * SPDX-FileCopyrightText: 2023-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "driver/i2c_master.h"

#define ICM42688_I2C_ADDRESS         0x68 /*!< I2C address with SDO pin low */
#define ICM42688_I2C_ADDRESS_1       0x69 /*!< I2C address with SDO pin high */

/* ICM42688P Registers */
#define ICM42688_WHO_AM_I            0x75 /*!< Device identification register */
#define ICM42688_OIS_CONF            0x04 /*!< OIS configuration register */
#define ICM42688_COM_CFG             0x05 /*!< Communication configuration */
#define ICM42688_INT_CFG1            0x06 /*!< Interrupt configuration 1 */
#define ICM42688_INT_CFG2            0x07 /*!< Interrupt configuration 2 */
#define ICM42688_HPF_LPF_CFG         0x08 /*!< High-pass and low-pass filter configuration */
#define ICM42688_DATA_SAT            0x0A /*!< Data saturation status */
#define ICM42688_DATA_STAT           0x0B /*!< Data status */
#define ICM42688_ACC_XH              0x1F /*!< Accelerometer X-axis high byte */
#define ICM42688_ACC_XL              0x20 /*!< Accelerometer X-axis low byte */
#define ICM42688_ACC_YH              0x21 /*!< Accelerometer Y-axis high byte */
#define ICM42688_ACC_YL              0x22 /*!< Accelerometer Y-axis low byte */
#define ICM42688_ACC_ZH              0x23 /*!< Accelerometer Z-axis high byte */
#define ICM42688_ACC_ZL              0x24 /*!< Accelerometer Z-axis low byte */
#define ICM42688_GYR_XH              0x25 /*!< Gyroscope X-axis high byte */
#define ICM42688_GYR_XL              0x26 /*!< Gyroscope X-axis low byte */
#define ICM42688_GYR_YH              0x27 /*!< Gyroscope Y-axis high byte */
#define ICM42688_GYR_YL              0x28 /*!< Gyroscope Y-axis low byte */
#define ICM42688_GYR_ZH              0x29 /*!< Gyroscope Z-axis high byte */
#define ICM42688_GYR_ZL              0x2A /*!< Gyroscope Z-axis low byte */
#define ICM42688_TEMP_H              0x1D /*!< Temperature high byte */
#define ICM42688_TEMP_L              0x1E /*!< Temperature low byte */
#define ICM42688_ACC_CONF            0x50 /*!< Accelerometer configuration */
#define ICM42688_ACC_RANGE           0x50 /*!< Accelerometer full-scale range */
#define ICM42688_GYR_CONF            0x4F /*!< Gyroscope configuration */
#define ICM42688_GYR_RANGE           0x4F /*!< Gyroscope full-scale range */
#define ICM42688_PWR_CTRL            0x4E /*!< Power control register */
#define ICM42688_SOFT_RST            0x11 /*!< Software reset register */

/* Sensitivity definitions for accelerometer */
#define ACCE_FS_2G_SENSITIVITY       (16384.0f)  /* 1g = 16384 LSB */
#define ACCE_FS_4G_SENSITIVITY       (8192.0f)   /* 1g = 8192 LSB */
#define ACCE_FS_8G_SENSITIVITY       (4096.0f)   /* 1g = 4096 LSB */
#define ACCE_FS_16G_SENSITIVITY      (2048.0f)   /* 1g = 2048 LSB */

/* Sensitivity definitions for gyroscope */
#define GYRO_FS_125DPS_SENSITIVITY   (262.144f)  /* 1 dps = 262.144 LSB */
#define GYRO_FS_250DPS_SENSITIVITY   (131.072f)  /* 1 dps = 131.072 LSB */
#define GYRO_FS_500DPS_SENSITIVITY   (65.536f)   /* 1 dps = 65.536 LSB */
#define GYRO_FS_1000DPS_SENSITIVITY  (32.768f)   /* 1 dps = 32.768 LSB */
#define GYRO_FS_2000DPS_SENSITIVITY  (16.384f)   /* 1 dps = 16.384 LSB */

typedef enum {
    ACCE_FS_2G = 0,     /*!< Accelerometer full scale range is +/- 2g */
    ACCE_FS_4G = 1,     /*!< Accelerometer full scale range is +/- 4g */
    ACCE_FS_8G = 2,     /*!< Accelerometer full scale range is +/- 8g */
    ACCE_FS_16G = 3,    /*!< Accelerometer full scale range is +/- 16g */
} icm42688_acce_fs_t;

typedef enum {
    ACCE_ODR_1_5625HZ = 15, /*!< Accelerometer ODR 1.5625 Hz */
    ACCE_ODR_3_125HZ  = 14, /*!< Accelerometer ODR 3.125 Hz */
    ACCE_ODR_6_25HZ   = 13, /*!< Accelerometer ODR 6.25 Hz */
    ACCE_ODR_12_5HZ   = 12, /*!< Accelerometer ODR 12.5 Hz */
    ACCE_ODR_25HZ     = 11, /*!< Accelerometer ODR 25 Hz */
    ACCE_ODR_50HZ     = 10, /*!< Accelerometer ODR 50 Hz */
    ACCE_ODR_100HZ    = 9,  /*!< Accelerometer ODR 100 Hz */
    ACCE_ODR_200HZ    = 8,  /*!< Accelerometer ODR 200 Hz */
    ACCE_ODR_400HZ    = 7,  /*!< Accelerometer ODR 400 Hz */
    ACCE_ODR_800HZ    = 6,  /*!< Accelerometer ODR 800 Hz */
    ACCE_ODR_1600HZ   = 5,  /*!< Accelerometer ODR 1600 Hz */
} icm42688_acce_odr_t;

typedef enum {
    GYRO_FS_125DPS  = 4,    /*!< Gyroscope full scale range is +/- 125 dps */
    GYRO_FS_250DPS  = 3,    /*!< Gyroscope full scale range is +/- 250 dps */
    GYRO_FS_500DPS  = 2,    /*!< Gyroscope full scale range is +/- 500 dps */
    GYRO_FS_1000DPS = 1,    /*!< Gyroscope full scale range is +/- 1000 dps */
    GYRO_FS_2000DPS = 0,    /*!< Gyroscope full scale range is +/- 2000 dps */
} icm42688_gyro_fs_t;

typedef enum {
    GYRO_ODR_25HZ   = 11,   /*!< Gyroscope ODR 25 Hz */
    GYRO_ODR_50HZ   = 10,   /*!< Gyroscope ODR 50 Hz */
    GYRO_ODR_100HZ  = 9,    /*!< Gyroscope ODR 100 Hz */
    GYRO_ODR_200HZ  = 8,    /*!< Gyroscope ODR 200 Hz */
    GYRO_ODR_400HZ  = 7,    /*!< Gyroscope ODR 400 Hz */
    GYRO_ODR_800HZ  = 6,    /*!< Gyroscope ODR 800 Hz */
    GYRO_ODR_1600HZ = 5,    /*!< Gyroscope ODR 1600 Hz */
    GYRO_ODR_3200HZ = 13,   /*!< Gyroscope ODR 3200 Hz */
} icm42688_gyro_odr_t;

typedef enum {
    ACCE_PWR_OFF      = 0,     /*!< Accelerometer power off */
    ACCE_PWR_ON       = 1,     /*!< Accelerometer power on */
} icm42688_acce_pwr_t;

typedef enum {
    GYRO_PWR_OFF      = 0,     /*!< Gyroscope power off */
    GYRO_PWR_ON       = 1,     /*!< Gyroscope power on */
} icm42688_gyro_pwr_t;

typedef enum {
    TEMP_PWR_OFF      = 0,     /*!< Temperature sensor power off */
    TEMP_PWR_ON       = 1,     /*!< Temperature sensor power on */
} icm42688_temp_pwr_t;

typedef struct {
    icm42688_acce_fs_t  acce_fs;    /*!< Accelerometer full scale range */
    icm42688_acce_odr_t acce_odr;   /*!< Accelerometer ODR selection */
    icm42688_gyro_fs_t  gyro_fs;    /*!< Gyroscope full scale range */
    icm42688_gyro_odr_t gyro_odr;   /*!< Gyroscope ODR selection */
} icm42688_cfg_t;

typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
} icm42688_raw_value_t;

typedef struct {
    float x;
    float y;
    float z;
} icm42688_value_t;

typedef struct {
    float roll;
    float pitch;
} complimentary_angle_t;

typedef void *icm42688_handle_t;

/**
 * @brief Create and init sensor object
 *
 * @param[in]  i2c_bus    I2C bus handle. Obtained from i2c_new_master_bus()
 * @param[in]  dev_addr   I2C device address of sensor. Can be ICM42688_I2C_ADDRESS or ICM42688_I2C_ADDRESS_1
 * @param[out] handle_ret Handle to created ICM42688 driver object
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_ERR_NO_MEM Not enough memory for the driver
 *     - ESP_ERR_NOT_FOUND Sensor not found on the I2C bus
 *     - Others Error from underlying I2C driver
 */
esp_err_t icm42688_create(i2c_master_bus_handle_t i2c_bus, const uint8_t dev_addr, icm42688_handle_t *handle_ret);

/**
 * @brief Delete and release a sensor object
 *
 * @param sensor object handle of icm42688
 */
void icm42688_delete(icm42688_handle_t sensor);

/**
 * @brief Get device identification of ICM42688
 *
 * @param sensor object handle of icm42688
 * @param deviceid a pointer of device ID
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t icm42688_get_deviceid(icm42688_handle_t sensor, uint8_t *deviceid);

/**
 * @brief Configure accelerometer and gyroscope
 *
 * @param sensor object handle of icm42688
 * @param config Accelerometer and gyroscope configuration structure
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t icm42688_config(icm42688_handle_t sensor, const icm42688_cfg_t *config);

/**
 * @brief Set power mode for accelerometer, gyroscope and temperature sensor
 *
 * @param sensor object handle of icm42688
 * @param acce_state power mode of accelerometer
 * @param gyro_state power mode of gyroscope
 * @param temp_state power mode of temperature sensor
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t icm42688_set_power_mode(icm42688_handle_t sensor, icm42688_acce_pwr_t acce_state,
                                  icm42688_gyro_pwr_t gyro_state, icm42688_temp_pwr_t temp_state);

/**
 * @brief Software reset the sensor
 *
 * @param sensor object handle of icm42688
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t icm42688_soft_reset(icm42688_handle_t sensor);

/**
 * @brief Get accelerometer sensitivity
 *
 * @param sensor object handle of icm42688
 * @param sensitivity accelerometer sensitivity
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t icm42688_get_acce_sensitivity(icm42688_handle_t sensor, float *sensitivity);

/**
 * @brief Get gyroscope sensitivity
 *
 * @param sensor object handle of icm42688
 * @param sensitivity gyroscope sensitivity
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t icm42688_get_gyro_sensitivity(icm42688_handle_t sensor, float *sensitivity);

/**
 * @brief Read raw temperature measurements
 *
 * @param sensor object handle of icm42688
 * @param value raw value of a temperature measurement in two's complement
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t icm42688_get_temp_raw_value(icm42688_handle_t sensor, uint16_t *value);

/**
 * @brief Read raw accelerometer measurements
 *
 * @param sensor object handle of icm42688
 * @param value raw accelerometer measurements
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t icm42688_get_acce_raw_value(icm42688_handle_t sensor, icm42688_raw_value_t *value);

/**
 * @brief Read raw gyroscope measurements
 *
 * @param sensor object handle of icm42688
 * @param value raw gyroscope measurements
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t icm42688_get_gyro_raw_value(icm42688_handle_t sensor, icm42688_raw_value_t *value);

/**
 * @brief Read accelerometer measurements
 *
 * @param sensor object handle of icm42688
 * @param value accelerometer measurements
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t icm42688_get_acce_value(icm42688_handle_t sensor, icm42688_value_t *value);

/**
 * @brief Read gyro values
 *
 * @param sensor object handle of icm42688
 * @param value gyroscope measurements
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t icm42688_get_gyro_value(icm42688_handle_t sensor, icm42688_value_t *value);

/**
 * @brief Read temperature value
 *
 * @param sensor object handle of icm42688
 * @param value temperature measurements in Celsius
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t icm42688_get_temp_value(icm42688_handle_t sensor, float *value);

/**
 * @brief Use complementary filter to calculate roll and pitch angles
 *
 * @param sensor object handle of icm42688
 * @param acce_value accelerometer measurements
 * @param gyro_value gyroscope measurements
 * @param complimentary_angle calculated complimentary angle
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t icm42688_complimentary_filter(icm42688_handle_t sensor, const icm42688_value_t *acce_value,
                                        const icm42688_value_t *gyro_value, complimentary_angle_t *complimentary_angle);

/**
 * @brief Read a register
 *
 * @param sensor object handle of icm42688
 * @param reg register address
 * @param val value of the register
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t icm42688_read_register(icm42688_handle_t sensor, uint8_t reg, uint8_t *val);

/**
 * @brief Write to a register
 *
 * @param sensor object handle of icm42688
 * @param reg register address
 * @param val value to write
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t icm42688_write_register(icm42688_handle_t sensor, uint8_t reg, uint8_t val);

#ifdef __cplusplus
}
#endif