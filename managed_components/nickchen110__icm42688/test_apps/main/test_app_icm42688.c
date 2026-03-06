/*
 * SPDX-FileCopyrightText: 2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include "unity.h"
#include "driver/i2c_master.h"
#include "icm42688.h"
#include "esp_system.h"
#include "esp_log.h"
#include "unity.h"
#include "unity_test_runner.h"
#include "unity_test_utils_memory.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// Pinout for ESP32-S3-BOX or similar development board
#define I2C_MASTER_SCL_IO 18       /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO 8        /*!< gpio number for I2C master data  */
#define I2C_MASTER_NUM I2C_NUM_0   /*!< I2C port number for master dev */

static const char *TAG = "icm42688 test";
static icm42688_handle_t icm42688 = NULL;
static i2c_master_bus_handle_t i2c_handle = NULL;

static void i2c_bus_init(void)
{
    const i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_MASTER_NUM,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .intr_priority = 0,
        .trans_queue_depth = 0,
    };

    esp_err_t ret = i2c_new_master_bus(&bus_config, &i2c_handle);
    TEST_ASSERT_EQUAL_MESSAGE(ESP_OK, ret, "I2C install returned error");
}

static void i2c_sensor_icm42688_init(void)
{
    esp_err_t ret;

    i2c_bus_init();
    
    // Create ICM42688P sensor instance
    ret = icm42688_create(i2c_handle, ICM42688_I2C_ADDRESS, &icm42688);
    TEST_ASSERT_EQUAL(ESP_OK, ret);
    TEST_ASSERT_NOT_NULL_MESSAGE(icm42688, "icm42688 create returned NULL");

    /* Configuration of the accelerometer and gyroscope */
    const icm42688_cfg_t imu_cfg = {
        .acce_fs = ACCE_FS_8G,
        .acce_odr = ACCE_ODR_100HZ,
        .gyro_fs = GYRO_FS_500DPS,
        .gyro_odr = GYRO_ODR_100HZ,
    };
    
    ret = icm42688_config(icm42688, &imu_cfg);
    TEST_ASSERT_EQUAL(ESP_OK, ret);
}

TEST_CASE("Sensor icm42688 basic test", "[icm42688]")
{
    esp_err_t ret;
    icm42688_value_t acc, gyro;
    float temperature;
    uint8_t device_id;

    i2c_sensor_icm42688_init();

    /* Test device ID reading */
    ret = icm42688_get_deviceid(icm42688, &device_id);
    TEST_ASSERT_EQUAL(ESP_OK, ret);
    ESP_LOGI(TAG, "Device ID: 0x%02X (expected: 0x6A)", device_id);
    TEST_ASSERT_EQUAL(0x6A, device_id);

    /* Enable accelerometer, gyroscope and temperature sensor */
    ret = icm42688_set_power_mode(icm42688, ACCE_PWR_ON, GYRO_PWR_ON, TEMP_PWR_ON);
    TEST_ASSERT_EQUAL(ESP_OK, ret);

    /* Wait for sensors to stabilize */
    vTaskDelay(pdMS_TO_TICKS(100));

    /* Read and log sensor data for 5 iterations */
    for (int i = 0; i < 5; i++) {
        vTaskDelay(pdMS_TO_TICKS(100));
        
        ret = icm42688_get_acce_value(icm42688, &acc);
        TEST_ASSERT_EQUAL(ESP_OK, ret);
        
        ret = icm42688_get_gyro_value(icm42688, &gyro);
        TEST_ASSERT_EQUAL(ESP_OK, ret);
        
        ret = icm42688_get_temp_value(icm42688, &temperature);
        TEST_ASSERT_EQUAL(ESP_OK, ret);
        
        ESP_LOGI(TAG, "Sample %d:", i+1);
        ESP_LOGI(TAG, "  Accel: X=%.3fg, Y=%.3fg, Z=%.3fg", acc.x, acc.y, acc.z);
        ESP_LOGI(TAG, "  Gyro:  X=%.3f°/s, Y=%.3f°/s, Z=%.3f°/s", gyro.x, gyro.y, gyro.z);
        ESP_LOGI(TAG, "  Temp:  %.1f°C", temperature);
        
        // Test raw value reading as well
        icm42688_raw_value_t acc_raw, gyro_raw;
        uint16_t temp_raw;
        
        ret = icm42688_get_acce_raw_value(icm42688, &acc_raw);
        TEST_ASSERT_EQUAL(ESP_OK, ret);
        
        ret = icm42688_get_gyro_raw_value(icm42688, &gyro_raw);
        TEST_ASSERT_EQUAL(ESP_OK, ret);
        
        ret = icm42688_get_temp_raw_value(icm42688, &temp_raw);
        TEST_ASSERT_EQUAL(ESP_OK, ret);
        
        ESP_LOGD(TAG, "  Raw Accel: X=%d, Y=%d, Z=%d", acc_raw.x, acc_raw.y, acc_raw.z);
        ESP_LOGD(TAG, "  Raw Gyro:  X=%d, Y=%d, Z=%d", gyro_raw.x, gyro_raw.y, gyro_raw.z);
        ESP_LOGD(TAG, "  Raw Temp:  %d", temp_raw);
    }

    /* Test sensitivity reading */
    float acc_sensitivity, gyro_sensitivity;
    ret = icm42688_get_acce_sensitivity(icm42688, &acc_sensitivity);
    TEST_ASSERT_EQUAL(ESP_OK, ret);
    ret = icm42688_get_gyro_sensitivity(icm42688, &gyro_sensitivity);
    TEST_ASSERT_EQUAL(ESP_OK, ret);
    
    ESP_LOGI(TAG, "Accelerometer sensitivity: %.0f LSB/g", acc_sensitivity);
    ESP_LOGI(TAG, "Gyroscope sensitivity: %.3f LSB/°/s", gyro_sensitivity);

    /* Cleanup */
    icm42688_delete(icm42688);
    ret = i2c_del_master_bus(i2c_handle);
    TEST_ASSERT_EQUAL(ESP_OK, ret);
    vTaskDelay(pdMS_TO_TICKS(10)); // Give FreeRTOS some time to free its resources
}

TEST_CASE("Sensor icm42688 complementary filter test", "[icm42688]")
{
    esp_err_t ret;
    icm42688_value_t acc, gyro;
    complimentary_angle_t angles;

    i2c_sensor_icm42688_init();

    /* Enable sensors */
    ret = icm42688_set_power_mode(icm42688, ACCE_PWR_ON, GYRO_PWR_ON, TEMP_PWR_OFF);
    TEST_ASSERT_EQUAL(ESP_OK, ret);

    /* Wait for sensors to stabilize */
    vTaskDelay(pdMS_TO_TICKS(100));

    /* Test complementary filter */
    ESP_LOGI(TAG, "Testing complementary filter...");
    
    for (int i = 0; i < 10; i++) {
        vTaskDelay(pdMS_TO_TICKS(100));
        
        ret = icm42688_get_acce_value(icm42688, &acc);
        TEST_ASSERT_EQUAL(ESP_OK, ret);
        
        ret = icm42688_get_gyro_value(icm42688, &gyro);
        TEST_ASSERT_EQUAL(ESP_OK, ret);
        
        ret = icm42688_complimentary_filter(icm42688, &acc, &gyro, &angles);
        TEST_ASSERT_EQUAL(ESP_OK, ret);
        
        ESP_LOGI(TAG, "Sample %d - Roll: %.2f°, Pitch: %.2f°", i+1, angles.roll, angles.pitch);
        
        // Verify angles are within reasonable range
        TEST_ASSERT(angles.roll >= -180.0f && angles.roll <= 180.0f);
        TEST_ASSERT(angles.pitch >= -90.0f && angles.pitch <= 90.0f);
    }

    /* Cleanup */
    icm42688_delete(icm42688);
    ret = i2c_del_master_bus(i2c_handle);
    TEST_ASSERT_EQUAL(ESP_OK, ret);
}

TEST_CASE("Sensor icm42688 configuration test", "[icm42688]")
{
    esp_err_t ret;
    
    i2c_bus_init();
    
    /* Test creating sensor with different I2C addresses */
    icm42688_handle_t sensor;
    
    // Test with primary address
    ret = icm42688_create(i2c_handle, ICM42688_I2C_ADDRESS, &sensor);
    TEST_ASSERT_EQUAL(ESP_OK, ret);
    
    uint8_t device_id;
    ret = icm42688_get_deviceid(sensor, &device_id);
    TEST_ASSERT_EQUAL(ESP_OK, ret);
    TEST_ASSERT_EQUAL(0x6A, device_id);
    
    icm42688_delete(sensor);
    
    // Test with alternate address (if available)
    // Note: This test requires the sensor's SDO pin to be pulled high
    ret = icm42688_create(i2c_handle, ICM42688_I2C_ADDRESS_1, &sensor);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Alternate I2C address (0x69) detected");
        ret = icm42688_get_deviceid(sensor, &device_id);
        TEST_ASSERT_EQUAL(ESP_OK, ret);
        TEST_ASSERT_EQUAL(0x6A, device_id);
        icm42688_delete(sensor);
    } else {
        ESP_LOGW(TAG, "No device found at alternate I2C address 0x69");
    }
    
    /* Test different configurations */
    icm42688_cfg_t test_configs[] = {
        // Low range, low ODR
        {
            .acce_fs = ACCE_FS_2G,
            .acce_odr = ACCE_ODR_25HZ,
            .gyro_fs = GYRO_FS_250DPS,
            .gyro_odr = GYRO_ODR_25HZ,
        },
        // Medium range, medium ODR
        {
            .acce_fs = ACCE_FS_4G,
            .acce_odr = ACCE_ODR_100HZ,
            .gyro_fs = GYRO_FS_500DPS,
            .gyro_odr = GYRO_ODR_100HZ,
        },
        // High range, high ODR
        {
            .acce_fs = ACCE_FS_16G,
            .acce_odr = ACCE_ODR_400HZ,
            .gyro_fs = GYRO_FS_2000DPS,
            .gyro_odr = GYRO_ODR_400HZ,
        }
    };
    
    for (int i = 0; i < sizeof(test_configs)/sizeof(test_configs[0]); i++) {
        ESP_LOGI(TAG, "Testing configuration %d...", i+1);
        
        ret = icm42688_create(i2c_handle, ICM42688_I2C_ADDRESS, &sensor);
        TEST_ASSERT_EQUAL(ESP_OK, ret);
        
        ret = icm42688_config(sensor, &test_configs[i]);
        TEST_ASSERT_EQUAL(ESP_OK, ret);
        
        ret = icm42688_set_power_mode(sensor, ACCE_PWR_ON, GYRO_PWR_ON, TEMP_PWR_ON);
        TEST_ASSERT_EQUAL(ESP_OK, ret);
        
        vTaskDelay(pdMS_TO_TICKS(50));
        
        // Quick read to verify sensor is working
        icm42688_value_t acc, gyro;
        float temp;
        
        ret = icm42688_get_acce_value(sensor, &acc);
        TEST_ASSERT_EQUAL(ESP_OK, ret);
        
        ret = icm42688_get_gyro_value(sensor, &gyro);
        TEST_ASSERT_EQUAL(ESP_OK, ret);
        
        ret = icm42688_get_temp_value(sensor, &temp);
        TEST_ASSERT_EQUAL(ESP_OK, ret);
        
        ESP_LOGI(TAG, "  Config %d working - Accel Z: %.3fg, Gyro Z: %.3f°/s, Temp: %.1f°C", 
                i+1, acc.z, gyro.z, temp);
        
        icm42688_delete(sensor);
    }
    
    /* Cleanup */
    ret = i2c_del_master_bus(i2c_handle);
    TEST_ASSERT_EQUAL(ESP_OK, ret);
}

TEST_CASE("Sensor icm42688 power mode test", "[icm42688]")
{
    esp_err_t ret;
    icm42688_value_t acc;
    
    i2c_sensor_icm42688_init();
    
    /* Test different power modes */
    ESP_LOGI(TAG, "Testing power modes...");
    
    // 1. All sensors off
    ret = icm42688_set_power_mode(icm42688, ACCE_PWR_OFF, GYRO_PWR_OFF, TEMP_PWR_OFF);
    TEST_ASSERT_EQUAL(ESP_OK, ret);
    vTaskDelay(pdMS_TO_TICKS(50));
    
    // 2. Accelerometer only
    ret = icm42688_set_power_mode(icm42688, ACCE_PWR_ON, GYRO_PWR_OFF, TEMP_PWR_OFF);
    TEST_ASSERT_EQUAL(ESP_OK, ret);
    vTaskDelay(pdMS_TO_TICKS(50));
    
    ret = icm42688_get_acce_value(icm42688, &acc);
    TEST_ASSERT_EQUAL(ESP_OK, ret);
    ESP_LOGI(TAG, "Accel only mode - Accel Z: %.3fg", acc.z);
    
    // 3. Gyroscope only
    ret = icm42688_set_power_mode(icm42688, ACCE_PWR_OFF, GYRO_PWR_ON, TEMP_PWR_OFF);
    TEST_ASSERT_EQUAL(ESP_OK, ret);
    vTaskDelay(pdMS_TO_TICKS(50));
    
    icm42688_value_t gyro;
    ret = icm42688_get_gyro_value(icm42688, &gyro);
    TEST_ASSERT_EQUAL(ESP_OK, ret);
    ESP_LOGI(TAG, "Gyro only mode - Gyro Z: %.3f°/s", gyro.z);
    
    // 4. All sensors on
    ret = icm42688_set_power_mode(icm42688, ACCE_PWR_ON, GYRO_PWR_ON, TEMP_PWR_ON);
    TEST_ASSERT_EQUAL(ESP_OK, ret);
    vTaskDelay(pdMS_TO_TICKS(50));
    
    float temp;
    ret = icm42688_get_acce_value(icm42688, &acc);
    TEST_ASSERT_EQUAL(ESP_OK, ret);
    ret = icm42688_get_gyro_value(icm42688, &gyro);
    TEST_ASSERT_EQUAL(ESP_OK, ret);
    ret = icm42688_get_temp_value(icm42688, &temp);
    TEST_ASSERT_EQUAL(ESP_OK, ret);
    
    ESP_LOGI(TAG, "All sensors on - Accel Z: %.3fg, Gyro Z: %.3f°/s, Temp: %.1f°C", 
            acc.z, gyro.z, temp);
    
    /* Cleanup */
    icm42688_delete(icm42688);
    ret = i2c_del_master_bus(i2c_handle);
    TEST_ASSERT_EQUAL(ESP_OK, ret);
}

TEST_CASE("Sensor icm42688 register access test", "[icm42688]")
{
    esp_err_t ret;
    uint8_t reg_value;
    
    i2c_sensor_icm42688_init();
    
    /* Test register read/write */
    ESP_LOGI(TAG, "Testing register access...");
    
    // Read WHO_AM_I register
    ret = icm42688_read_register(icm42688, ICM42688_WHO_AM_I, &reg_value);
    TEST_ASSERT_EQUAL(ESP_OK, ret);
    ESP_LOGI(TAG, "WHO_AM_I register: 0x%02X", reg_value);
    TEST_ASSERT_EQUAL(0x6A, reg_value);
    
    // Read PWR_CTRL register
    ret = icm42688_read_register(icm42688, ICM42688_PWR_CTRL, &reg_value);
    TEST_ASSERT_EQUAL(ESP_OK, ret);
    ESP_LOGI(TAG, "PWR_CTRL register: 0x%02X", reg_value);
    
    // Test writing and reading back a register
    // Note: We'll use a register that doesn't affect operation significantly
    uint8_t test_value = 0xAA;
    ret = icm42688_write_register(icm42688, ICM42688_SOFT_RST, test_value);
    // SOFT_RST is write-only, so we don't test read back
    
    // Test communication configuration register
    uint8_t com_cfg_value;
    ret = icm42688_read_register(icm42688, ICM42688_COM_CFG, &com_cfg_value);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "COM_CFG register: 0x%02X", com_cfg_value);
        
        // Modify and write back
        uint8_t new_com_cfg = com_cfg_value | 0x10; // Set address auto-increment
        ret = icm42688_write_register(icm42688, ICM42688_COM_CFG, new_com_cfg);
        TEST_ASSERT_EQUAL(ESP_OK, ret);
        
        // Read back to verify
        uint8_t read_back;
        ret = icm42688_read_register(icm42688, ICM42688_COM_CFG, &read_back);
        TEST_ASSERT_EQUAL(ESP_OK, ret);
        TEST_ASSERT_EQUAL(new_com_cfg, read_back);
        ESP_LOGI(TAG, "COM_CFG after write: 0x%02X", read_back);
    }
    
    /* Cleanup */
    icm42688_delete(icm42688);
    ret = i2c_del_master_bus(i2c_handle);
    TEST_ASSERT_EQUAL(ESP_OK, ret);
}

#define TEST_MEMORY_LEAK_THRESHOLD  (500)

void setUp(void)
{
    unity_utils_set_leak_level(TEST_MEMORY_LEAK_THRESHOLD);
    unity_utils_record_free_mem();
}

void tearDown(void)
{
    unity_utils_evaluate_leaks();
}

void app_main(void)
{
    // Initialize logging
    esp_log_level_set("*", ESP_LOG_INFO);
    esp_log_level_set("icm42688 test", ESP_LOG_VERBOSE);
    
    printf("\n");
    printf("=========================================\n");
    printf("ICM42688P Sensor Test Application\n");
    printf("=========================================\n");
    printf("\n");
    
    // Run all tests
    UNITY_BEGIN();
    
    RUN_TEST_CASE("Sensor icm42688 basic test");
    RUN_TEST_CASE("Sensor icm42688 complementary filter test");
    RUN_TEST_CASE("Sensor icm42688 configuration test");
    RUN_TEST_CASE("Sensor icm42688 power mode test");
    RUN_TEST_CASE("Sensor icm42688 register access test");
    
    UNITY_END();
    
    printf("\n");
    printf("=========================================\n");
    printf("All tests completed!\n");
    printf("=========================================\n");
    
    // Keep the task alive for serial monitor output
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}