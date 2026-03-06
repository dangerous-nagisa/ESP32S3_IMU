/*
 * SPDX-FileCopyrightText: 2023-2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <string.h>
#include <stdio.h>
#include <math.h>
#include <time.h>
#include <sys/time.h>
#include "esp_system.h"
#include "esp_check.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"
#include "icm42688.h"

#define I2C_CLK_SPEED 400000

#define ALPHA                       0.97f        /*!< Weight of gyroscope */
#define RAD_TO_DEG                  57.27272727f /*!< Radians to degrees */

#define ICM42688_ID                 0x47        /*!< ICM42688P device ID */

static uint8_t icm42688_accel_fs_to_reg(icm42688_acce_fs_t fs)
{
    switch (fs) {
    case ACCE_FS_2G:
        return 3;
    case ACCE_FS_4G:
        return 2;
    case ACCE_FS_8G:
        return 1;
    case ACCE_FS_16G:
    default:
        return 0;
    }
}

static float icm42688_accel_reg_to_sensitivity(uint8_t fs_sel)
{
    switch (fs_sel & 0x3) {
    case 3:
        return ACCE_FS_2G_SENSITIVITY;
    case 2:
        return ACCE_FS_4G_SENSITIVITY;
    case 1:
        return ACCE_FS_8G_SENSITIVITY;
    case 0:
    default:
        return ACCE_FS_16G_SENSITIVITY;
    }
}

static float icm42688_gyro_reg_to_sensitivity(uint8_t fs_sel)
{
    switch (fs_sel & 0x7) {
    case GYRO_FS_2000DPS:
        return GYRO_FS_2000DPS_SENSITIVITY;
    case GYRO_FS_1000DPS:
        return GYRO_FS_1000DPS_SENSITIVITY;
    case GYRO_FS_500DPS:
        return GYRO_FS_500DPS_SENSITIVITY;
    case GYRO_FS_250DPS:
        return GYRO_FS_250DPS_SENSITIVITY;
    case GYRO_FS_125DPS:
    default:
        return GYRO_FS_125DPS_SENSITIVITY;
    }
}

/*******************************************************************************
* Types definitions
*******************************************************************************/

typedef struct {
    i2c_master_dev_handle_t i2c_handle;
    bool initialized_filter;
    uint64_t previous_measurement_us;
    complimentary_angle_t previous_measurement;
} icm42688_dev_t;

/*******************************************************************************
* Function definitions
*******************************************************************************/
static esp_err_t icm42688_write(icm42688_handle_t sensor, const uint8_t reg_start_addr, const uint8_t *data_buf, const uint8_t data_len);
static esp_err_t icm42688_read(icm42688_handle_t sensor, const uint8_t reg_start_addr, uint8_t *data_buf, const uint8_t data_len);
static esp_err_t icm42688_get_raw_value(icm42688_handle_t sensor, uint8_t reg_start, icm42688_raw_value_t *value);

/*******************************************************************************
* Local variables
*******************************************************************************/
static const char *TAG = "ICM42688";

/*******************************************************************************
* Public API functions
*******************************************************************************/

esp_err_t icm42688_create(i2c_master_bus_handle_t i2c_bus, const uint8_t dev_addr, icm42688_handle_t *handle_ret)
{
    esp_err_t ret = ESP_OK;

    // Allocate memory and init the driver object
    icm42688_dev_t *sensor = (icm42688_dev_t *) calloc(1, sizeof(icm42688_dev_t));
    ESP_RETURN_ON_FALSE(sensor != NULL, ESP_ERR_NO_MEM, TAG, "Not enough memory");

    // Add new I2C device
    const i2c_device_config_t i2c_dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = dev_addr,
        .scl_speed_hz = I2C_CLK_SPEED,
    };
    ESP_GOTO_ON_ERROR(i2c_master_bus_add_device(i2c_bus, &i2c_dev_cfg, &sensor->i2c_handle), err, TAG, "Failed to add new I2C device");
    assert(sensor->i2c_handle);

    // Check device presence
    uint8_t dev_id = 0;
    icm42688_get_deviceid(sensor, &dev_id);
    ESP_GOTO_ON_FALSE(dev_id == ICM42688_ID, ESP_ERR_NOT_FOUND, err, TAG, "Incorrect Device ID (0x%02x). Expected 0x%02x", dev_id, ICM42688_ID);

    ESP_LOGD(TAG, "Found ICM42688P device, ID: 0x%02x", dev_id);
    
    // Perform software reset
    ret = icm42688_soft_reset(sensor);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Software reset failed");
    }
    
    // Delay for reset to complete
    esp_rom_delay_us(1000);
    
    *handle_ret = sensor;
    return ret;

err:
    icm42688_delete(sensor);
    return ret;
}

void icm42688_delete(icm42688_handle_t sensor)
{
    icm42688_dev_t *sens = (icm42688_dev_t *) sensor;

    if (sens->i2c_handle) {
        i2c_master_bus_rm_device(sens->i2c_handle);
    }

    free(sens);
}

esp_err_t icm42688_get_deviceid(icm42688_handle_t sensor, uint8_t *deviceid)
{
    esp_err_t ret = ESP_FAIL;

    assert(deviceid != NULL);

    for (int i = 0; (i < 5 && ret != ESP_OK); i++) {
        ret = icm42688_read(sensor, ICM42688_WHO_AM_I, deviceid, 1);
        if (ret == ESP_OK && *deviceid != 0) {
            break;
        }
        esp_rom_delay_us(1000);
    }

    return ret;
}

esp_err_t icm42688_config(icm42688_handle_t sensor, const icm42688_cfg_t *config)
{
    uint8_t accel_cfg0 = 0;
    uint8_t gyro_cfg0 = 0;

    assert(config != NULL);

    accel_cfg0 = (icm42688_accel_fs_to_reg(config->acce_fs) << 5) | (config->acce_odr & 0x0F);
    gyro_cfg0 = ((config->gyro_fs & 0x07) << 5) | (config->gyro_odr & 0x0F);

    ESP_RETURN_ON_ERROR(icm42688_write(sensor, ICM42688_ACC_CONF, &accel_cfg0, 1), TAG, "Failed to configure accelerometer");
    ESP_RETURN_ON_ERROR(icm42688_write(sensor, ICM42688_GYR_CONF, &gyro_cfg0, 1), TAG, "Failed to configure gyroscope");

    return ESP_OK;
}

esp_err_t icm42688_set_power_mode(icm42688_handle_t sensor, icm42688_acce_pwr_t acce_state,
                                  icm42688_gyro_pwr_t gyro_state, icm42688_temp_pwr_t temp_state)
{
    uint8_t pwr_ctrl = 0;

    pwr_ctrl |= (temp_state == TEMP_PWR_OFF ? 1 : 0) << 5;
    pwr_ctrl |= (gyro_state == GYRO_PWR_ON ? 0x03 : 0x00) << 2;
    pwr_ctrl |= (acce_state == ACCE_PWR_ON ? 0x03 : 0x00);

    ESP_RETURN_ON_ERROR(icm42688_write(sensor, ICM42688_PWR_CTRL, &pwr_ctrl, 1), TAG, "Failed to set power mode");
    esp_rom_delay_us(10000);

    return ESP_OK;
}

esp_err_t icm42688_soft_reset(icm42688_handle_t sensor)
{
    uint8_t reset_cmd = 0x01;
    ESP_RETURN_ON_ERROR(icm42688_write(sensor, ICM42688_SOFT_RST, &reset_cmd, 1), TAG, "Failed to perform soft reset");
    esp_rom_delay_us(10000);

    return ESP_OK;
}

esp_err_t icm42688_get_acce_sensitivity(icm42688_handle_t sensor, float *sensitivity)
{
    esp_err_t ret = ESP_FAIL;
    uint8_t accel_cfg0 = 0;

    assert(sensitivity != NULL);

    *sensitivity = 0;

    ret = icm42688_read(sensor, ICM42688_ACC_CONF, &accel_cfg0, 1);
    if (ret == ESP_OK) {
        *sensitivity = icm42688_accel_reg_to_sensitivity((accel_cfg0 >> 5) & 0x03);
    }

    return ret;
}

esp_err_t icm42688_get_gyro_sensitivity(icm42688_handle_t sensor, float *sensitivity)
{
    esp_err_t ret = ESP_FAIL;
    uint8_t gyro_cfg0 = 0;

    assert(sensitivity != NULL);

    *sensitivity = 0;

    ret = icm42688_read(sensor, ICM42688_GYR_CONF, &gyro_cfg0, 1);
    if (ret == ESP_OK) {
        *sensitivity = icm42688_gyro_reg_to_sensitivity((gyro_cfg0 >> 5) & 0x07);
    }

    return ret;
}

esp_err_t icm42688_get_temp_raw_value(icm42688_handle_t sensor, uint16_t *value)
{
    esp_err_t ret = ESP_FAIL;
    uint8_t data[2];

    assert(value != NULL);

    *value = 0;

    ret = icm42688_read(sensor, ICM42688_TEMP_H, data, sizeof(data));
    if (ret == ESP_OK) {
        *value = (uint16_t)((data[0] << 8) | data[1]);
    }

    return ret;
}

esp_err_t icm42688_get_acce_raw_value(icm42688_handle_t sensor, icm42688_raw_value_t *value)
{
    return icm42688_get_raw_value(sensor, ICM42688_ACC_XH, value);
}

esp_err_t icm42688_get_gyro_raw_value(icm42688_handle_t sensor, icm42688_raw_value_t *value)
{
    return icm42688_get_raw_value(sensor, ICM42688_GYR_XH, value);
}

esp_err_t icm42688_get_acce_value(icm42688_handle_t sensor, icm42688_value_t *value)
{
    esp_err_t ret;
    float sensitivity;
    icm42688_raw_value_t raw_value;

    assert(value != NULL);

    value->x = 0;
    value->y = 0;
    value->z = 0;

    ret = icm42688_get_acce_sensitivity(sensor, &sensitivity);
    ESP_RETURN_ON_ERROR(ret, TAG, "Get sensitivity error!");

    ret = icm42688_get_acce_raw_value(sensor, &raw_value);
    ESP_RETURN_ON_ERROR(ret, TAG, "Get raw value error!");

    value->x = raw_value.x / sensitivity;
    value->y = raw_value.y / sensitivity;
    value->z = raw_value.z / sensitivity;

    return ESP_OK;
}

esp_err_t icm42688_get_gyro_value(icm42688_handle_t sensor, icm42688_value_t *value)
{
    esp_err_t ret;
    float sensitivity;
    icm42688_raw_value_t raw_value;

    assert(value != NULL);

    value->x = 0;
    value->y = 0;
    value->z = 0;

    ret = icm42688_get_gyro_sensitivity(sensor, &sensitivity);
    ESP_RETURN_ON_ERROR(ret, TAG, "Get sensitivity error!");

    ret = icm42688_get_gyro_raw_value(sensor, &raw_value);
    ESP_RETURN_ON_ERROR(ret, TAG, "Get raw value error!");

    value->x = raw_value.x / sensitivity;
    value->y = raw_value.y / sensitivity;
    value->z = raw_value.z / sensitivity;

    return ESP_OK;
}

esp_err_t icm42688_get_temp_value(icm42688_handle_t sensor, float *value)
{
    esp_err_t ret;
    uint16_t raw_value;

    assert(value != NULL);

    *value = 0;

    ret = icm42688_get_temp_raw_value(sensor, &raw_value);
    ESP_RETURN_ON_ERROR(ret, TAG, "Get raw value error!");

    // Temperature conversion formula from datasheet: T = (raw_value / 512.0) + 23.0
    *value = ((int16_t)raw_value / 512.0f) + 23.0f;

    return ESP_OK;
}

/*******************************************************************************
* Private functions
*******************************************************************************/

static esp_err_t icm42688_get_raw_value(icm42688_handle_t sensor, uint8_t reg_start, icm42688_raw_value_t *value)
{
    esp_err_t ret = ESP_FAIL;
    uint8_t data[6];

    assert(value != NULL);

    value->x = 0;
    value->y = 0;
    value->z = 0;

    ret = icm42688_read(sensor, reg_start, data, sizeof(data));
    if (ret == ESP_OK) {
        value->x = (int16_t)((data[0] << 8) | data[1]);
        value->y = (int16_t)((data[2] << 8) | data[3]);
        value->z = (int16_t)((data[4] << 8) | data[5]);
    }

    return ret;
}

static esp_err_t icm42688_write(icm42688_handle_t sensor, const uint8_t reg_start_addr, const uint8_t *data_buf, const uint8_t data_len)
{
    icm42688_dev_t *sens = (icm42688_dev_t *) sensor;
    assert(sens);

    uint8_t write_buff[16] = {reg_start_addr};
    memcpy(&write_buff[1], data_buf, data_len);
    return i2c_master_transmit(sens->i2c_handle, write_buff, data_len + 1, -1);
}

static esp_err_t icm42688_read(icm42688_handle_t sensor, const uint8_t reg_start_addr, uint8_t *data_buf, const uint8_t data_len)
{
    uint8_t reg_buff[] = {reg_start_addr};
    icm42688_dev_t *sens = (icm42688_dev_t *) sensor;
    assert(sens);

    /* Write register number and read data */
    return i2c_master_transmit_receive(sens->i2c_handle, reg_buff, sizeof(reg_buff), data_buf, data_len, -1);
}

esp_err_t icm42688_complimentary_filter(icm42688_handle_t sensor, const icm42688_value_t *acce_value,
                                        const icm42688_value_t *gyro_value, complimentary_angle_t *complimentary_angle)
{
    icm42688_dev_t *sens = (icm42688_dev_t *) sensor;
    float measurement_delta;
    uint64_t current_time_us;
    float acc_roll_angle;
    float acc_pitch_angle;
    float gyro_roll_angle;
    float gyro_pitch_angle;

    // Calculate roll and pitch from accelerometer data
    acc_roll_angle = (atan2(acce_value->y, sqrt(acce_value->x * acce_value->x + acce_value->z * acce_value->z)) * RAD_TO_DEG);
    acc_pitch_angle = (atan2(-acce_value->x, sqrt(acce_value->y * acce_value->y + acce_value->z * acce_value->z)) * RAD_TO_DEG);

    if (!sens->initialized_filter) {
        sens->initialized_filter = true;
        sens->previous_measurement_us = esp_timer_get_time();
        sens->previous_measurement.roll = acc_roll_angle;
        sens->previous_measurement.pitch = acc_pitch_angle;
    }

    current_time_us = esp_timer_get_time();
    measurement_delta = (current_time_us - sens->previous_measurement_us) / 1000000.0f;
    sens->previous_measurement_us = current_time_us;

    // Integrate gyroscope data
    gyro_roll_angle = gyro_value->x * measurement_delta;
    gyro_pitch_angle = gyro_value->y * measurement_delta;

    // Apply complementary filter
    complimentary_angle->roll = (ALPHA * (sens->previous_measurement.roll + gyro_roll_angle)) + ((1 - ALPHA) * acc_roll_angle);
    complimentary_angle->pitch = (ALPHA * (sens->previous_measurement.pitch + gyro_pitch_angle)) + ((1 - ALPHA) * acc_pitch_angle);

    sens->previous_measurement.roll = complimentary_angle->roll;
    sens->previous_measurement.pitch = complimentary_angle->pitch;

    return ESP_OK;
}

esp_err_t icm42688_read_register(icm42688_handle_t sensor, uint8_t reg, uint8_t *val)
{
    return icm42688_read(sensor, reg, val, 1);
}

esp_err_t icm42688_write_register(icm42688_handle_t sensor, uint8_t reg, uint8_t val)
{
    return icm42688_write(sensor, reg, &val, 1);
}