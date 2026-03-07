#include <cmath>
#include <cinttypes>
#include <cstring>

#include "bmi160.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "driver/rmt_tx.h"
#include "esp_check.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "icm42688.h"
#include "i2cdev.h"
#include "led_strip.h"
#include "madgwick_filter.hpp"
#include "tinyusb.h"
#include "tusb_cdc_acm.h"

namespace {

static const char *TAG = "dual_imu";

constexpr gpio_num_t ICM_I2C_SDA = GPIO_NUM_8;
constexpr gpio_num_t ICM_I2C_SCL = GPIO_NUM_9;
constexpr gpio_num_t BMI_I2C_SDA = GPIO_NUM_16;
constexpr gpio_num_t BMI_I2C_SCL = GPIO_NUM_17;
constexpr gpio_num_t RGB_LED_GPIO = GPIO_NUM_48;  // RGB LED (WS2812)

constexpr i2c_port_num_t ICM_I2C_PORT = I2C_NUM_0;
constexpr i2c_port_t BMI_I2C_PORT = static_cast<i2c_port_t>(I2C_NUM_1);

constexpr uint32_t I2C_FREQ_HZ = 400000;
constexpr TickType_t SAMPLE_PERIOD_TICKS = pdMS_TO_TICKS(20);  // 50Hz采样
constexpr TickType_t CALIBRATION_WARMUP_TICKS = pdMS_TO_TICKS(2000);
constexpr TickType_t CALIBRATION_SAMPLE_DELAY_TICKS = pdMS_TO_TICKS(5);
constexpr int CALIBRATION_SAMPLES = 50;  // 降低到50个样本
constexpr float CALIBRATION_MAX_GYRO_NORM = 0.50f;  // 放宽陀螺仪阈值
constexpr float CALIBRATION_MIN_ACCEL_NORM = 0.85f;
constexpr float CALIBRATION_MAX_ACCEL_NORM = 1.15f;
constexpr float MADGWICK_BETA_RUN = 0.05f;
constexpr float DEG_TO_RAD = static_cast<float>(M_PI / 180.0);

constexpr uint8_t ICM_I2C_ADDRESS = ICM42688_I2C_ADDRESS;
constexpr uint8_t BMI_I2C_ADDRESS = BMI160_I2C_ADDRESS_VDD;

struct EulerAngles {
    float pitch;
    float roll;
    float yaw;
};

struct ImuSample {
    float ax;
    float ay;
    float az;
    float gx;
    float gy;
    float gz;
};

struct GyroBias {
    float x;
    float y;
    float z;
};

// USB CDC数据帧结构（88字节）
typedef struct __attribute__((packed)) {
    uint8_t header[2];          // 0xAA 0x55
    uint32_t timestamp_ms;      // 时间戳毫秒
    float quat_torso[4];        // 躯干四元数 w,x,y,z
    float quat_arm[4];          // 手臂四元数 w,x,y,z
    float torso_accel[3];       // 躯干加速度 ax,ay,az (g)
    float torso_gyro[3];        // 躯干角速度 gx,gy,gz (rad/s)
    float arm_accel[3];         // 手臂加速度 ax,ay,az (g)
    float arm_gyro[3];          // 手臂角速度 gx,gy,gz (rad/s)
    uint16_t crc16;             // CRC-16/MODBUS
} imu_data_frame_t;

// 环形缓冲区配置
constexpr size_t RING_BUFFER_SIZE = 64;  // 可容纳64帧

// 环形缓冲区
class RingBuffer {
private:
    imu_data_frame_t buffer[RING_BUFFER_SIZE];
    volatile size_t write_index = 0;
    volatile size_t read_index = 0;
    volatile size_t count = 0;
    SemaphoreHandle_t mutex;

public:
    RingBuffer() {
        mutex = xSemaphoreCreateMutex();
    }

    ~RingBuffer() {
        if (mutex) {
            vSemaphoreDelete(mutex);
        }
    }

    // 写入数据（覆盖旧数据）
    bool push(const imu_data_frame_t &frame) {
        if (xSemaphoreTake(mutex, 0) != pdTRUE) {  // 不等待，立即返回
            return false;
        }

        buffer[write_index] = frame;
        write_index = (write_index + 1) % RING_BUFFER_SIZE;

        if (count < RING_BUFFER_SIZE) {
            count++;
        } else {
            // 缓冲区满，覆盖旧数据，移动读指针
            read_index = (read_index + 1) % RING_BUFFER_SIZE;
        }

        xSemaphoreGive(mutex);
        return true;
    }

    // 读取数据
    bool pop(imu_data_frame_t *frame) {
        if (xSemaphoreTake(mutex, 0) != pdTRUE) {  // 不等待，立即返回
            return false;
        }

        if (count == 0) {
            xSemaphoreGive(mutex);
            return false;
        }

        *frame = buffer[read_index];
        read_index = (read_index + 1) % RING_BUFFER_SIZE;
        count--;

        xSemaphoreGive(mutex);
        return true;
    }

    size_t available() const {
        return count;
    }

    bool is_full() const {
        return count >= RING_BUFFER_SIZE;
    }
};

static RingBuffer usb_ring_buffer;

// 扩展MadgwickFilter以访问四元数
class MadgwickFilterExtended : public espp::MadgwickFilter {
public:
    explicit MadgwickFilterExtended(float beta = 0.1f)
        : espp::MadgwickFilter(beta) {}

    void get_quaternion_array(float quat[4]) const {
        quat[0] = q0;  // w
        quat[1] = q1;  // x
        quat[2] = q2;  // y
        quat[3] = q3;  // z
    }
};

static bool usb_cdc_initialized = false;

// CRC-16/MODBUS计算
uint16_t calculate_crc16(const uint8_t *data, size_t length) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < length; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x0001) {
                crc = (crc >> 1) ^ 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}

// USB CDC初始化
esp_err_t init_usb_cdc() {
    const tinyusb_config_t tusb_cfg = {
        .device_descriptor = NULL,
        .string_descriptor = NULL,
        .external_phy = false,
        .configuration_descriptor = NULL,
    };

    ESP_RETURN_ON_ERROR(tinyusb_driver_install(&tusb_cfg), TAG, "Failed to install TinyUSB driver");

    tinyusb_config_cdcacm_t acm_cfg = {
        .usb_dev = TINYUSB_USBDEV_0,
        .cdc_port = TINYUSB_CDC_ACM_0,
        .rx_unread_buf_sz = 64,
        .callback_rx = NULL,
        .callback_rx_wanted_char = NULL,
        .callback_line_state_changed = NULL,
        .callback_line_coding_changed = NULL,
    };

    ESP_RETURN_ON_ERROR(tusb_cdc_acm_init(&acm_cfg), TAG, "Failed to init CDC ACM");

    usb_cdc_initialized = true;
    ESP_LOGI(TAG, "USB CDC initialized successfully");
    return ESP_OK;
}

// 通过USB发送IMU数据
void send_imu_data_via_usb(
    const ImuSample &arm_sample,
    const ImuSample &torso_sample,
    const MadgwickFilterExtended &arm_filter,
    const MadgwickFilterExtended &torso_filter,
    uint32_t timestamp_ms
) {
    if (!usb_cdc_initialized) {
        return;
    }

    // 检查CDC连接状态（仅用于日志，不影响发送）
    static bool last_connected = false;
    bool connected = tud_cdc_n_connected(0);

    if (connected != last_connected) {
        if (connected) {
            ESP_LOGI(TAG, "USB CDC Host connected!");
        } else {
            ESP_LOGW(TAG, "USB CDC Host disconnected!");
        }
        last_connected = connected;
    }

    imu_data_frame_t frame;

    // 帧头
    frame.header[0] = 0xAA;
    frame.header[1] = 0x55;

    // 时间戳
    frame.timestamp_ms = timestamp_ms;

    // 四元数
    torso_filter.get_quaternion_array(frame.quat_torso);
    arm_filter.get_quaternion_array(frame.quat_arm);

    // 躯干原始数据
    frame.torso_accel[0] = torso_sample.ax;
    frame.torso_accel[1] = torso_sample.ay;
    frame.torso_accel[2] = torso_sample.az;
    frame.torso_gyro[0] = torso_sample.gx;
    frame.torso_gyro[1] = torso_sample.gy;
    frame.torso_gyro[2] = torso_sample.gz;

    // 手臂原始数据
    frame.arm_accel[0] = arm_sample.ax;
    frame.arm_accel[1] = arm_sample.ay;
    frame.arm_accel[2] = arm_sample.az;
    frame.arm_gyro[0] = arm_sample.gx;
    frame.arm_gyro[1] = arm_sample.gy;
    frame.arm_gyro[2] = arm_sample.gz;

    // CRC计算（不包括CRC字段本身）
    frame.crc16 = calculate_crc16((uint8_t*)&frame, sizeof(frame) - 2);

    // 写入环形缓冲区
    static uint32_t push_count = 0;
    static uint32_t overwrite_count = 0;

    bool was_full = usb_ring_buffer.is_full();
    if (usb_ring_buffer.push(frame)) {
        push_count++;
        if (was_full) {
            overwrite_count++;
            if (overwrite_count % 100 == 1) {
                ESP_LOGW(TAG, "Ring buffer full, overwriting old data (total overwrites: %lu)", overwrite_count);
            }
        }
        if (push_count % 100 == 0) {
            ESP_LOGI(TAG, "Pushed %lu frames to ring buffer (overwrites: %lu, available: %zu)",
                     push_count, overwrite_count, usb_ring_buffer.available());
        }
    }
}

// USB发送任务
void usb_send_task(void *arg) {
    ESP_LOGI(TAG, "USB send task started");

    uint32_t send_count = 0;
    uint32_t error_count = 0;
    imu_data_frame_t frame;

    while (true) {
        // 从环形缓冲区读取数据
        if (usb_ring_buffer.pop(&frame)) {
            // 尝试发送
            esp_err_t ret = tinyusb_cdcacm_write_queue(
                TINYUSB_CDC_ACM_0,
                (uint8_t*)&frame,
                sizeof(frame)
            );

            if (ret != ESP_OK) {
                error_count++;
                if (error_count % 100 == 1) {
                    ESP_LOGW(TAG, "USB send failed: %s (total errors: %lu)", esp_err_to_name(ret), error_count);
                }
                // 发送失败，等待一下再重试
                vTaskDelay(pdMS_TO_TICKS(10));
            } else {
                send_count++;
                if (send_count % 100 == 0) {
                    ESP_LOGI(TAG, "USB sent %lu frames (errors: %lu, buffer: %zu)",
                             send_count, error_count, usb_ring_buffer.available());
                }
                // 发送成功，短暂延迟让出CPU
                vTaskDelay(1);
            }
        } else {
            // 缓冲区空，等待新数据
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }
}

// RGB LED句柄
static led_strip_handle_t led_strip = nullptr;

// RGB LED初始化
esp_err_t init_rgb_led() {
    led_strip_config_t strip_config = {
        .strip_gpio_num = RGB_LED_GPIO,
        .max_leds = 1,
        .led_pixel_format = LED_PIXEL_FORMAT_GRB,
        .led_model = LED_MODEL_WS2812,
        .flags = {
            .invert_out = false,
        }
    };

    led_strip_rmt_config_t rmt_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = 10 * 1000 * 1000,  // 10MHz
        .flags = {
            .with_dma = false,
        }
    };

    return led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip);
}

// RGB LED闪烁任务
void rgb_led_blink_task(void *arg) {
    bool led_on = false;
    while (true) {
        if (led_on) {
            // 绿色表示系统正常运行
            led_strip_set_pixel(led_strip, 0, 0, 32, 0);  // R, G, B
        } else {
            // 关闭
            led_strip_set_pixel(led_strip, 0, 0, 0, 0);
        }
        led_strip_refresh(led_strip);
        led_on = !led_on;
        vTaskDelay(pdMS_TO_TICKS(1000));  // 1秒闪烁一次
    }
}

struct Icm42688Context {
    i2c_master_bus_handle_t bus = nullptr;
    icm42688_handle_t sensor = nullptr;
    float accel_sensitivity = 1.0f;
    float gyro_sensitivity = 1.0f;
};

struct Bmi160Context {
    bmi160_t sensor {};
};

void scan_i2c_bus(i2c_master_bus_handle_t bus, const char *bus_name) {
    ESP_LOGI(TAG, "Scanning %s...", bus_name);
    bool found = false;

    for (uint8_t addr = 0x08; addr <= 0x77; ++addr) {
        if (i2c_master_probe(bus, addr, 20) == ESP_OK) {
            ESP_LOGI(TAG, "%s found device at 0x%02X", bus_name, addr);
            found = true;
        }
    }

    if (!found) {
        ESP_LOGW(TAG, "%s no device found", bus_name);
    }
}

esp_err_t init_scan_bus(i2c_port_t port, gpio_num_t sda, gpio_num_t scl, i2c_master_bus_handle_t *bus) {
    const i2c_master_bus_config_t bus_config = {
        .i2c_port = port,
        .sda_io_num = sda,
        .scl_io_num = scl,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .intr_priority = 0,
        .trans_queue_depth = 0,
        .flags = {
            .enable_internal_pullup = true,
            .allow_pd = false,
        },
    };

    return i2c_new_master_bus(&bus_config, bus);
}

esp_err_t init_icm_bus(i2c_master_bus_handle_t *bus) {
    const i2c_master_bus_config_t bus_config = {
        .i2c_port = ICM_I2C_PORT,
        .sda_io_num = ICM_I2C_SDA,
        .scl_io_num = ICM_I2C_SCL,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .intr_priority = 0,
        .trans_queue_depth = 0,
        .flags = {
            .enable_internal_pullup = true,
            .allow_pd = false,
        },
    };

    return i2c_new_master_bus(&bus_config, bus);
}

esp_err_t init_icm42688(Icm42688Context *ctx) {
    ESP_RETURN_ON_ERROR(init_icm_bus(&ctx->bus), TAG, "failed to create ICM42688 I2C bus");
    ESP_RETURN_ON_ERROR(icm42688_create(ctx->bus, ICM_I2C_ADDRESS, &ctx->sensor), TAG, "failed to create ICM42688 device");

    const icm42688_cfg_t config = {
        .acce_fs = ACCE_FS_4G,
        .acce_odr = ACCE_ODR_100HZ,
        .gyro_fs = GYRO_FS_500DPS,
        .gyro_odr = GYRO_ODR_100HZ,
    };

    ESP_RETURN_ON_ERROR(icm42688_config(ctx->sensor, &config), TAG, "failed to configure ICM42688");
    ESP_RETURN_ON_ERROR(icm42688_set_power_mode(ctx->sensor, ACCE_PWR_ON, GYRO_PWR_ON, TEMP_PWR_ON), TAG,
                        "failed to power on ICM42688");
    ESP_RETURN_ON_ERROR(icm42688_get_acce_sensitivity(ctx->sensor, &ctx->accel_sensitivity), TAG,
                        "failed to get ICM42688 accel sensitivity");
    ESP_RETURN_ON_ERROR(icm42688_get_gyro_sensitivity(ctx->sensor, &ctx->gyro_sensitivity), TAG,
                        "failed to get ICM42688 gyro sensitivity");

    return ESP_OK;
}

esp_err_t init_bmi160(Bmi160Context *ctx) {
    ESP_RETURN_ON_ERROR(i2cdev_init(), TAG, "failed to init i2cdev");
    std::memset(&ctx->sensor, 0, sizeof(ctx->sensor));

    ESP_RETURN_ON_ERROR(bmi160_init(&ctx->sensor, BMI_I2C_ADDRESS, BMI_I2C_PORT, BMI_I2C_SDA, BMI_I2C_SCL), TAG,
                        "failed to init BMI160");

    // Software reset to ensure known state
    ESP_RETURN_ON_ERROR(bmi160_write_reg(&ctx->sensor, 0x7E, 0xB6), TAG,
                        "failed to soft-reset BMI160");
    vTaskDelay(pdMS_TO_TICKS(200));

    // Verify chip ID
    uint8_t chip_id = 0;
    ESP_RETURN_ON_ERROR(bmi160_read_reg(&ctx->sensor, 0x00, &chip_id), TAG,
                        "failed to read BMI160 chip ID");
    ESP_LOGI(TAG, "BMI160 chip ID: 0x%02X (expect 0xD1)", chip_id);
    if (chip_id != 0xD1) {
        ESP_LOGE(TAG, "BMI160 chip ID mismatch!");
        return ESP_ERR_INVALID_RESPONSE;
    }

    ESP_RETURN_ON_ERROR(bmi160_switch_mode(&ctx->sensor, BMI160_PMU_ACC_NORMAL, BMI160_PMU_GYR_NORMAL), TAG,
                        "failed to switch BMI160 mode");
    vTaskDelay(pdMS_TO_TICKS(50));

    ESP_RETURN_ON_ERROR(bmi160_set_acc_range(&ctx->sensor, BMI160_ACC_RANGE_4G), TAG,
                        "failed to set BMI160 accel range");
    ESP_RETURN_ON_ERROR(bmi160_set_gyr_range(&ctx->sensor, BMI160_GYR_RANGE_500DPS), TAG,
                        "failed to set BMI160 gyro range");
    ESP_RETURN_ON_ERROR(bmi160_set_acc_conf(&ctx->sensor, BMI160_ACC_ODR_100HZ, BMI160_ACC_LP_AVG_2, BMI160_ACC_US_OFF), TAG,
                        "failed to set BMI160 accel conf");
    ESP_RETURN_ON_ERROR(bmi160_set_gyr_odr(&ctx->sensor, BMI160_GYR_ODR_100HZ), TAG,
                        "failed to set BMI160 gyro odr");

    // Readback and verify configuration registers
    uint8_t acc_range_reg = 0, gyr_range_reg = 0, acc_conf_reg = 0, gyr_conf_reg = 0, pmu_status = 0;
    bmi160_read_reg(&ctx->sensor, 0x41, &acc_range_reg);
    bmi160_read_reg(&ctx->sensor, 0x43, &gyr_range_reg);
    bmi160_read_reg(&ctx->sensor, 0x40, &acc_conf_reg);
    bmi160_read_reg(&ctx->sensor, 0x42, &gyr_conf_reg);
    bmi160_read_reg(&ctx->sensor, 0x03, &pmu_status);
    ESP_LOGI(TAG, "BMI160 config readback: ACC_RANGE=0x%02X GYR_RANGE=0x%02X ACC_CONF=0x%02X GYR_CONF=0x%02X PMU=0x%02X",
             acc_range_reg, gyr_range_reg, acc_conf_reg, gyr_conf_reg, pmu_status);
    ESP_LOGI(TAG, "BMI160 aRes=%e gRes=%e", ctx->sensor.aRes, ctx->sensor.gRes);

    return ESP_OK;
}

float vector_norm3(float x, float y, float z) {
    return std::sqrt((x * x) + (y * y) + (z * z));
}

bool is_sample_still_for_calibration(const ImuSample &sample) {
    const float gyro_norm = vector_norm3(sample.gx, sample.gy, sample.gz);
    const float accel_norm = vector_norm3(sample.ax, sample.ay, sample.az);

    return gyro_norm <= CALIBRATION_MAX_GYRO_NORM &&
           accel_norm >= CALIBRATION_MIN_ACCEL_NORM &&
           accel_norm <= CALIBRATION_MAX_ACCEL_NORM;
}

esp_err_t read_icm42688_sample(const Icm42688Context &ctx, ImuSample *sample) {
    icm42688_raw_value_t accel_raw {};
    icm42688_raw_value_t gyro_raw {};

    esp_err_t ret = icm42688_get_acce_raw_value(ctx.sensor, &accel_raw);
    if (ret != ESP_OK) {
        return ret;
    }

    ret = icm42688_get_gyro_raw_value(ctx.sensor, &gyro_raw);
    if (ret != ESP_OK) {
        return ret;
    }

    *sample = {
        .ax = static_cast<float>(accel_raw.x) / ctx.accel_sensitivity,
        .ay = static_cast<float>(accel_raw.y) / ctx.accel_sensitivity,
        .az = static_cast<float>(accel_raw.z) / ctx.accel_sensitivity,
        .gx = (static_cast<float>(gyro_raw.x) / ctx.gyro_sensitivity) * DEG_TO_RAD,
        .gy = (static_cast<float>(gyro_raw.y) / ctx.gyro_sensitivity) * DEG_TO_RAD,
        .gz = (static_cast<float>(gyro_raw.z) / ctx.gyro_sensitivity) * DEG_TO_RAD,
    };
    return ESP_OK;
}

GyroBias calibrate_icm42688_gyro(const Icm42688Context &ctx) {
    GyroBias bias {};
    int valid_samples = 0;
    int read_errors = 0;
    int motion_rejects = 0;

    ESP_LOGI(TAG, "Calibrating ICM42688 gyro bias, keep device still");
    for (int i = 0; i < CALIBRATION_SAMPLES; ++i) {
        ImuSample sample {};
        const esp_err_t ret = read_icm42688_sample(ctx, &sample);
        if (ret != ESP_OK) {
            read_errors++;
            if (read_errors % 50 == 1) {
                ESP_LOGW(TAG, "ICM42688 read error: %s (total: %d)", esp_err_to_name(ret), read_errors);
            }
        } else if (is_sample_still_for_calibration(sample)) {
            bias.x += sample.gx;
            bias.y += sample.gy;
            bias.z += sample.gz;
            ++valid_samples;
            if (valid_samples % 50 == 0) {
                ESP_LOGI(TAG, "ICM42688 calibration progress: %d/%d valid samples", valid_samples, CALIBRATION_SAMPLES);
            }
        } else {
            motion_rejects++;
            if (motion_rejects % 50 == 1) {
                float gyro_norm = vector_norm3(sample.gx, sample.gy, sample.gz);
                float accel_norm = vector_norm3(sample.ax, sample.ay, sample.az);
                ESP_LOGW(TAG, "ICM42688 motion detected: gyro_norm=%.3f accel_norm=%.3f (rejects: %d)",
                         gyro_norm, accel_norm, motion_rejects);
            }
        }
        vTaskDelay(CALIBRATION_SAMPLE_DELAY_TICKS);
    }

    if (valid_samples > 0) {
        const float scale = 1.0f / static_cast<float>(valid_samples);
        bias.x *= scale;
        bias.y *= scale;
        bias.z *= scale;
    }

    ESP_LOGI(TAG, "ICM42688 gyro bias[rad/s]=[%+.5f, %+.5f, %+.5f] from %d still samples (errors: %d, motion: %d)",
             bias.x, bias.y, bias.z, valid_samples, read_errors, motion_rejects);
    return bias;
}

esp_err_t read_bmi160_sample(Bmi160Context *ctx, ImuSample *sample) {
    bmi160_result_t result {};
    esp_err_t ret = bmi160_read_data(&ctx->sensor, &result);
    if (ret != ESP_OK) {
        return ret;
    }

    *sample = {
        .ax = result.accX,
        .ay = result.accY,
        .az = result.accZ,
        .gx = result.gyroX * DEG_TO_RAD,
        .gy = result.gyroY * DEG_TO_RAD,
        .gz = result.gyroZ * DEG_TO_RAD,
    };
    return ESP_OK;
}

GyroBias calibrate_bmi160_gyro(Bmi160Context *ctx) {
    GyroBias bias {};
    int valid_samples = 0;

    ESP_LOGI(TAG, "Calibrating BMI160 gyro bias, keep device still");
    for (int i = 0; i < CALIBRATION_SAMPLES; ++i) {
        ImuSample sample {};
        if (read_bmi160_sample(ctx, &sample) != ESP_OK) {
            vTaskDelay(CALIBRATION_SAMPLE_DELAY_TICKS);
            continue;
        }
        if (is_sample_still_for_calibration(sample)) {
            bias.x += sample.gx;
            bias.y += sample.gy;
            bias.z += sample.gz;
            ++valid_samples;
        }
        vTaskDelay(CALIBRATION_SAMPLE_DELAY_TICKS);
    }

    if (valid_samples > 0) {
        const float scale = 1.0f / static_cast<float>(valid_samples);
        bias.x *= scale;
        bias.y *= scale;
        bias.z *= scale;
    }

    ESP_LOGI(TAG, "BMI160 gyro bias[rad/s]=[%+.5f, %+.5f, %+.5f] from %d still samples",
             bias.x, bias.y, bias.z, valid_samples);
    return bias;
}

void apply_gyro_bias(ImuSample *sample, const GyroBias &bias) {
    sample->gx -= bias.x;
    sample->gy -= bias.y;
    sample->gz -= bias.z;
}

void initialize_filter_from_accel(espp::MadgwickFilter *filter, const ImuSample &sample) {
    const float roll = std::atan2(sample.ay, sample.az);
    const float pitch = std::atan2(-sample.ax, std::sqrt((sample.ay * sample.ay) + (sample.az * sample.az)));
    const float yaw = 0.0f;

    const float cy = std::cos(yaw * 0.5f);
    const float sy = std::sin(yaw * 0.5f);
    const float cp = std::cos(pitch * 0.5f);
    const float sp = std::sin(pitch * 0.5f);
    const float cr = std::cos(roll * 0.5f);
    const float sr = std::sin(roll * 0.5f);

    const float q0 = cr * cp * cy + sr * sp * sy;
    const float q1 = sr * cp * cy - cr * sp * sy;
    const float q2 = cr * sp * cy + sr * cp * sy;
    const float q3 = cr * cp * sy - sr * sp * cy;

    filter->set_quaternion(q0, q1, q2, q3);
}

bool initialize_filter_from_icm(espp::MadgwickFilter *filter, const Icm42688Context &ctx, const GyroBias &bias) {
    ImuSample sample {};
    const esp_err_t ret = read_icm42688_sample(ctx, &sample);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "failed to initialize ICM42688 filter from accel: %s", esp_err_to_name(ret));
        return false;
    }

    apply_gyro_bias(&sample, bias);
    initialize_filter_from_accel(filter, sample);
    return true;
}

bool initialize_filter_from_bmi(espp::MadgwickFilter *filter, Bmi160Context *ctx, const GyroBias &bias) {
    ImuSample sample {};
    // Retry a few times in case data is not ready yet
    for (int i = 0; i < 10; ++i) {
        esp_err_t ret = read_bmi160_sample(ctx, &sample);
        if (ret == ESP_OK) {
            apply_gyro_bias(&sample, bias);
            initialize_filter_from_accel(filter, sample);
            return true;
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
    ESP_LOGW(TAG, "failed to initialize BMI160 filter from accel");
    return false;
}

void dump_bmi160_raw(Bmi160Context *ctx, int count) {
    ESP_LOGI(TAG, "=== BMI160 RAW DATA DUMP (%d samples) ===", count);
    for (int i = 0; i < count; ++i) {
        bmi160_result_t raw {};
        esp_err_t ret = bmi160_read_data(&ctx->sensor, &raw);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "  sample %d: read error %s", i, esp_err_to_name(ret));
        } else {
            float accel_norm = std::sqrt(raw.accX*raw.accX + raw.accY*raw.accY + raw.accZ*raw.accZ);
            float gyro_norm = std::sqrt(raw.gyroX*raw.gyroX + raw.gyroY*raw.gyroY + raw.gyroZ*raw.gyroZ);
            ESP_LOGI(TAG, "  sample %d: acc[g]=[%+.4f %+.4f %+.4f] |a|=%.4f  gyro[dps]=[%+.3f %+.3f %+.3f] |g|=%.3f",
                     i, raw.accX, raw.accY, raw.accZ, accel_norm,
                     raw.gyroX, raw.gyroY, raw.gyroZ, gyro_norm);
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
    ESP_LOGI(TAG, "=== END RAW DATA DUMP ===");
}

EulerAngles update_filter(espp::MadgwickFilter *filter, const ImuSample &sample, float dt_s) {
    filter->update(dt_s, sample.ax, sample.ay, sample.az, sample.gx, sample.gy, sample.gz);

    EulerAngles euler {};
    filter->get_euler(euler.pitch, euler.roll, euler.yaw);
    return euler;
}

void log_sample_and_attitude(const char *name, const ImuSample &sample, const EulerAngles &euler) {
    ESP_LOGI(TAG,
             "%s acc[g]=[%+.3f, %+.3f, %+.3f] gyro[rad/s]=[%+.3f, %+.3f, %+.3f] euler[deg]=[pitch=%+.2f roll=%+.2f yaw=%+.2f]",
             name, sample.ax, sample.ay, sample.az, sample.gx, sample.gy, sample.gz,
             euler.pitch, euler.roll, euler.yaw);
}

}  // namespace

extern "C" void app_main(void) {
    ESP_LOGI(TAG, "Initializing dual IMU application");

    // 初始化RGB LED
    ESP_ERROR_CHECK(init_rgb_led());
    xTaskCreate(rgb_led_blink_task, "rgb_blink", 2048, NULL, 1, NULL);
    ESP_LOGI(TAG, "RGB LED initialized on GPIO%d", RGB_LED_GPIO);

    i2c_master_bus_handle_t scan_bus0 = nullptr;
    i2c_master_bus_handle_t scan_bus1 = nullptr;

    if (init_scan_bus(static_cast<i2c_port_t>(ICM_I2C_PORT), ICM_I2C_SDA, ICM_I2C_SCL, &scan_bus0) == ESP_OK) {
        scan_i2c_bus(scan_bus0, "scan/icm_bus");
        i2c_del_master_bus(scan_bus0);
    } else {
        ESP_LOGW(TAG, "failed to create scan bus for ICM bus");
    }

    if (init_scan_bus(BMI_I2C_PORT, BMI_I2C_SDA, BMI_I2C_SCL, &scan_bus1) == ESP_OK) {
        scan_i2c_bus(scan_bus1, "scan/bmi_bus");
        i2c_del_master_bus(scan_bus1);
    } else {
        ESP_LOGW(TAG, "failed to create scan bus for BMI bus");
    }

    Icm42688Context icm {};
    Bmi160Context bmi {};

    ESP_ERROR_CHECK(init_icm42688(&icm));
    ESP_LOGI(TAG, "ICM-42688 init success on I2C%d SDA=%d SCL=%d", ICM_I2C_PORT, ICM_I2C_SDA, ICM_I2C_SCL);

    ESP_ERROR_CHECK(init_bmi160(&bmi));
    ESP_LOGI(TAG, "BMI160 init success on I2C%d SDA=%d SCL=%d", BMI_I2C_PORT, BMI_I2C_SDA, BMI_I2C_SCL);

    // Dump raw BMI160 data for diagnosis
    dump_bmi160_raw(&bmi, 20);

    MadgwickFilterExtended arm_filter(MADGWICK_BETA_RUN);
    MadgwickFilterExtended torso_filter(MADGWICK_BETA_RUN);

    ESP_LOGI(TAG, "Warm up IMUs for %lu ms before calibration, keep devices still",
             static_cast<unsigned long>(CALIBRATION_WARMUP_TICKS * portTICK_PERIOD_MS));
    vTaskDelay(CALIBRATION_WARMUP_TICKS);
    const GyroBias icm_gyro_bias = calibrate_icm42688_gyro(icm);
    const GyroBias bmi_gyro_bias = calibrate_bmi160_gyro(&bmi);
    initialize_filter_from_icm(&arm_filter, icm, icm_gyro_bias);
    initialize_filter_from_bmi(&torso_filter, &bmi, bmi_gyro_bias);
    ESP_LOGI(TAG, "Attitude initialization complete, entering normal output phase");

    // 在IMU校准完成后再初始化USB CDC
    ESP_LOGI(TAG, "Initializing USB CDC...");
    ESP_ERROR_CHECK(init_usb_cdc());

    // 启动USB发送任务
    xTaskCreate(usb_send_task, "usb_send", 4096, NULL, 5, NULL);
    ESP_LOGI(TAG, "USB send task created");

    int64_t last_tick_us = esp_timer_get_time();
    int64_t last_bmi_update_us = last_tick_us;

    while (true) {
        const int64_t now_us = esp_timer_get_time();
        float dt_s = static_cast<float>(now_us - last_tick_us) / 1000000.0f;
        last_tick_us = now_us;
        if (dt_s <= 0.0f) {
            dt_s = 0.01f;
        }

        ImuSample arm_sample {};
        const esp_err_t icm_ret = read_icm42688_sample(icm, &arm_sample);
        if (icm_ret != ESP_OK) {
            ESP_LOGW(TAG, "failed to read ICM42688 sample: %s", esp_err_to_name(icm_ret));
            vTaskDelay(SAMPLE_PERIOD_TICKS);
            continue;
        }
        ImuSample torso_sample {};
        const esp_err_t bmi_ret = read_bmi160_sample(&bmi, &torso_sample);
        apply_gyro_bias(&arm_sample, icm_gyro_bias);

        const EulerAngles arm_euler = update_filter(&arm_filter, arm_sample, dt_s);

        if (bmi_ret != ESP_OK) {
            ESP_LOGW(TAG, "failed to read BMI160 sample: %s", esp_err_to_name(bmi_ret));
        } else {
            // Use actual time since last BMI filter update for correct gyro integration
            float bmi_dt = static_cast<float>(now_us - last_bmi_update_us) / 1000000.0f;
            last_bmi_update_us = now_us;
            if (bmi_dt <= 0.0f || bmi_dt > 0.5f) {
                bmi_dt = 0.01f;
            }
            apply_gyro_bias(&torso_sample, bmi_gyro_bias);
            const EulerAngles torso_euler = update_filter(&torso_filter, torso_sample, bmi_dt);
            // log_sample_and_attitude("arm/icm42688", arm_sample, arm_euler);
            log_sample_and_attitude("torso/bmi160", torso_sample, torso_euler);

            // 通过USB发送IMU数据
            uint32_t timestamp_ms = static_cast<uint32_t>(now_us / 1000);
            send_imu_data_via_usb(arm_sample, torso_sample, arm_filter, torso_filter, timestamp_ms);
        }

        vTaskDelay(SAMPLE_PERIOD_TICKS);
    }
}
