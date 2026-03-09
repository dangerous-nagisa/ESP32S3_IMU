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
#include "freertos/event_groups.h"
#include "freertos/semphr.h"
#include "icm42688.h"
#include "i2cdev.h"
#include "led_strip.h"
#include "madgwick_filter.hpp"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "nvs_flash.h"
#include "lwip/sockets.h"
#include "lwip/netdb.h"
#include "lwip/inet.h"
#include <errno.h>
#include <cstdio>
#include <fcntl.h>

namespace {

static const char *TAG = "dual_imu";

constexpr gpio_num_t ICM_I2C_SDA = GPIO_NUM_8;
constexpr gpio_num_t ICM_I2C_SCL = GPIO_NUM_9;
constexpr gpio_num_t BMI_I2C_SDA = GPIO_NUM_16;
constexpr gpio_num_t BMI_I2C_SCL = GPIO_NUM_17;
constexpr gpio_num_t RGB_LED_GPIO = GPIO_NUM_48;  // RGB LED (WS2812)

constexpr i2c_port_num_t ICM_I2C_PORT = I2C_NUM_0;
constexpr i2c_port_t BMI_I2C_PORT = static_cast<i2c_port_t>(I2C_NUM_1);

constexpr uint32_t ICM_I2C_FREQ_HZ = 100000;  // 降低�?00kHz提高抗干�?
constexpr uint32_t BMI_I2C_FREQ_HZ = 400000;  // BMI160保持400kHz
constexpr TickType_t SAMPLE_PERIOD_TICKS = pdMS_TO_TICKS(20);  // 50Hz采样
constexpr TickType_t CALIBRATION_WARMUP_TICKS = pdMS_TO_TICKS(2000);
constexpr TickType_t CALIBRATION_SAMPLE_DELAY_TICKS = pdMS_TO_TICKS(10);  // 匹配100Hz ODR
constexpr int CALIBRATION_SAMPLES = 200;  // 增加样本�?
constexpr int MIN_VALID_SAMPLES = 100;  // 最低有效样本门�?
constexpr int MAX_CALIBRATION_TIME_MS = 8000;  // 最大校准时�?�?
constexpr int STARTUP_CALIBRATION_MAX_ATTEMPTS = 3;
constexpr TickType_t STARTUP_CALIBRATION_RETRY_DELAY_TICKS = pdMS_TO_TICKS(500);
constexpr int FILTER_INIT_RETRY_COUNT = 10;
constexpr TickType_t FILTER_INIT_RETRY_DELAY_TICKS = pdMS_TO_TICKS(20);
constexpr float CALIBRATION_MAX_GYRO_NORM = 0.10f;  // 收紧静止阈值到0.10 rad/s (�?.7 dps)
constexpr float CALIBRATION_MIN_ACCEL_NORM = 0.85f;
constexpr float CALIBRATION_MAX_ACCEL_NORM = 1.15f;
constexpr float MADGWICK_BETA_RUN = 0.3f;  // 增大Beta值提高响应速度（原0.05）
constexpr float DEG_TO_RAD = static_cast<float>(M_PI / 180.0);

constexpr uint8_t ICM_I2C_ADDRESS = ICM42688_I2C_ADDRESS;
constexpr uint8_t BMI_I2C_ADDRESS = BMI160_I2C_ADDRESS_VDD;
constexpr uint8_t ICM_DEVICE_ID_1 = 0x47;  // ICM42688 device ID variant 1
constexpr uint8_t ICM_DEVICE_ID_2 = 0x7B;  // ICM42688 device ID variant 2
constexpr int IMU_INIT_MAX_RETRIES = 5;
constexpr int IMU_RUNTIME_RECOVER_RETRIES = 5;
constexpr int64_t IMU_RECOVER_COOLDOWN_US = 300 * 1000;
constexpr int IMU_RECOVER_TRIGGER_CONSECUTIVE_ERRORS = 4;
constexpr TickType_t IMU_BUS_SETTLE_TICKS = pdMS_TO_TICKS(20);
constexpr int ICM_READ_RETRY_COUNT = 3;
constexpr TickType_t ICM_READ_RETRY_DELAY_TICKS = pdMS_TO_TICKS(2);

// WiFi配置
constexpr char WIFI_SSID[] = "ESP32_P4_AP";
constexpr char WIFI_PASS[] = "12345678";
constexpr char UDP_SERVER_IP[] = "192.168.4.1";
constexpr uint16_t UDP_SERVER_PORT = 8888;
constexpr int WIFI_MAXIMUM_RETRY = 5;
constexpr wifi_auth_mode_t WIFI_MIN_AUTH_MODE = WIFI_AUTH_WPA_PSK;
constexpr bool UDP_USE_WIFI_GATEWAY = true;

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

// IMU类型枚举
enum class ImuType : uint8_t {
    NONE = 0,
    ICM42688 = 1,  // arm (I2C0)
    BMI160 = 2     // torso (I2C1)
};

// 检测到的IMU信息
struct DetectedImu {
    ImuType type;
    i2c_port_t port;
    gpio_num_t sda;
    gpio_num_t scl;
};

// USB CDC数据帧结构（90字节）
typedef struct __attribute__((packed)) {
    uint8_t header[2];          // 0xAA 0x55
    uint8_t device_id;          // 1=arm(ICM42688), 2=torso(BMI160)
    uint8_t reserved;           // Reserved byte
    uint32_t timestamp_ms;      // 时间戳毫�?
    float quat_torso[4];        // 躯干四元�?w,x,y,z
    float quat_arm[4];          // 手臂四元�?w,x,y,z
    float acc_torso[3];         // 躯干加速度 ax,ay,az (g)
    float gyro_torso[3];        // 躯干角速度 gx,gy,gz (rad/s)
    float acc_arm[3];           // 手臂加速度 ax,ay,az (g)
    float gyro_arm[3];          // 手臂角速度 gx,gy,gz (rad/s)
    uint16_t crc16;             // CRC-16/MODBUS
} imu_data_frame_t;

// WiFi状态变量（使用EventGroup实现线程安全�?
static EventGroupHandle_t wifi_event_group;
constexpr int WIFI_CONNECTED_BIT = BIT0;
static int wifi_retry_count = 0;

// UDP Socket变量（使用互斥锁保护�?
static int udp_socket = -1;
static struct sockaddr_in dest_addr = {};
static SemaphoreHandle_t dest_addr_mutex = nullptr;

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

// WiFi事件处理（线程安全版本）
void wifi_event_handler(void* arg, esp_event_base_t event_base,
                        int32_t event_id, void* event_data) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
        ESP_LOGI(TAG, "WiFi connecting...");
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_CONNECTED) {
        // 使用静态IP时，连接成功就设置标志位
        ESP_LOGI(TAG, "WiFi connected (static IP mode)");
        wifi_retry_count = 0;

        // 先设置连接标志
        xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT);

        // 设置UDP目标地址为网关
        if (dest_addr_mutex && xSemaphoreTake(dest_addr_mutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
            dest_addr.sin_addr.s_addr = inet_addr("192.168.4.1");
            dest_addr.sin_family = AF_INET;
            dest_addr.sin_port = htons(UDP_SERVER_PORT);
            ESP_LOGI(TAG, "UDP target set to gateway: 192.168.4.1:%d", UDP_SERVER_PORT);
            xSemaphoreGive(dest_addr_mutex);
        }
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        xEventGroupClearBits(wifi_event_group, WIFI_CONNECTED_BIT);
        wifi_retry_count++;
        const int reason = event_data ? static_cast<int>(reinterpret_cast<wifi_event_sta_disconnected_t*>(event_data)->reason) : -1;
        if (wifi_retry_count <= WIFI_MAXIMUM_RETRY || (wifi_retry_count % 20) == 0) {
            ESP_LOGW(TAG, "WiFi disconnected, reason=%d, retry %d", reason, wifi_retry_count);
        }
        esp_wifi_connect();
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "WiFi connected, IP: " IPSTR, IP2STR(&event->ip_info.ip));
        wifi_retry_count = 0;

        // 先设置连接标志，确保发送函数能立即感知连接状�?
        xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT);

        // 使用互斥锁保护dest_addr更新（使用阻塞等待确保更新成功）
        if (dest_addr_mutex && xSemaphoreTake(dest_addr_mutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
            if (UDP_USE_WIFI_GATEWAY) {
                dest_addr.sin_addr.s_addr = event->ip_info.gw.addr;
                ESP_LOGI(TAG, "UDP target set to gateway: " IPSTR ":%d",
                         IP2STR(&event->ip_info.gw), UDP_SERVER_PORT);
            } else {
                dest_addr.sin_addr.s_addr = inet_addr(UDP_SERVER_IP);
            }
            dest_addr.sin_family = AF_INET;
            dest_addr.sin_port = htons(UDP_SERVER_PORT);
            xSemaphoreGive(dest_addr_mutex);
        } else {
            ESP_LOGE(TAG, "Failed to acquire mutex for dest_addr update, using fallback");
            // 即使获取锁失败，连接标志已设置，发送函数会使用默认地址
        }
    }
}

// WiFi Station初始化（兼容已初始化场景�?
esp_err_t init_wifi_sta() {
    // 创建EventGroup和互斥锁
    wifi_event_group = xEventGroupCreate();
    if (!wifi_event_group) {
        ESP_LOGE(TAG, "Failed to create wifi event group");
        return ESP_FAIL;
    }

    dest_addr_mutex = xSemaphoreCreateMutex();
    if (!dest_addr_mutex) {
        ESP_LOGE(TAG, "Failed to create dest_addr mutex");
        return ESP_FAIL;
    }

    // 初始化NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_RETURN_ON_ERROR(ret, TAG, "NVS init failed");

    // 初始化网络接口（容忍已初始化�?
    ret = esp_netif_init();
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "netif init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = esp_event_loop_create_default();
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "event loop create failed: %s", esp_err_to_name(ret));
        return ret;
    }

    esp_netif_t *sta_netif = esp_netif_create_default_wifi_sta();

    // 配置静态IP（如果P4没有DHCP服务器）
    esp_netif_dhcpc_stop(sta_netif);
    esp_netif_ip_info_t ip_info;
    IP4_ADDR(&ip_info.ip, 192, 168, 4, 2);       // S3的IP: 192.168.4.2
    IP4_ADDR(&ip_info.gw, 192, 168, 4, 1);       // 网关: 192.168.4.1 (P4)
    IP4_ADDR(&ip_info.netmask, 255, 255, 255, 0); // 子网掩码
    esp_netif_set_ip_info(sta_netif, &ip_info);
    ESP_LOGI(TAG, "Static IP configured: 192.168.4.2, Gateway: 192.168.4.1");

    // WiFi配置
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_RETURN_ON_ERROR(esp_wifi_init(&cfg), TAG, "WiFi init failed");

    // 注册事件处理
    ESP_RETURN_ON_ERROR(
        esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL),
        TAG, "WiFi event register failed");
    ESP_RETURN_ON_ERROR(
        esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL),
        TAG, "IP event register failed");

    // Station配置
    wifi_config_t wifi_config = {};
    strncpy((char*)wifi_config.sta.ssid, WIFI_SSID, sizeof(wifi_config.sta.ssid));
    strncpy((char*)wifi_config.sta.password, WIFI_PASS, sizeof(wifi_config.sta.password));
    wifi_config.sta.threshold.authmode = WIFI_MIN_AUTH_MODE;
    wifi_config.sta.pmf_cfg.capable = true;
    wifi_config.sta.pmf_cfg.required = false;

    ESP_RETURN_ON_ERROR(esp_wifi_set_mode(WIFI_MODE_STA), TAG, "set mode failed");
    ESP_RETURN_ON_ERROR(esp_wifi_set_config(WIFI_IF_STA, &wifi_config), TAG, "set config failed");
    ESP_RETURN_ON_ERROR(esp_wifi_start(), TAG, "WiFi start failed");

    ESP_LOGI(TAG, "WiFi Station initialized, connecting to %s", WIFI_SSID);
    return ESP_OK;
}

// UDP Client初始�?
esp_err_t init_udp_client() {
    // 创建UDP socket
    udp_socket = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (udp_socket < 0) {
        ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
        return ESP_FAIL;
    }

    // 初始化目标地址（仅在未设置时初始化，避免覆盖网关配置）
    if (dest_addr_mutex && xSemaphoreTake(dest_addr_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        // 检查是否已经由WiFi事件处理设置过（网关地址�?
        if (dest_addr.sin_family == 0) {
            // 未设置，使用默认配置
            dest_addr.sin_addr.s_addr = inet_addr(UDP_SERVER_IP);
            dest_addr.sin_family = AF_INET;
            dest_addr.sin_port = htons(UDP_SERVER_PORT);
            ESP_LOGI(TAG, "UDP client initialized with default target: %s:%d", UDP_SERVER_IP, UDP_SERVER_PORT);
        } else {
            ESP_LOGI(TAG, "UDP client initialized, target already set by WiFi event");
        }
        xSemaphoreGive(dest_addr_mutex);
    }

    // 设置socket为非阻塞
    int flags = fcntl(udp_socket, F_GETFL, 0);
    if (flags < 0 || fcntl(udp_socket, F_SETFL, flags | O_NONBLOCK) < 0) {
        ESP_LOGW(TAG, "Failed to set socket non-blocking");
    }

    // 设置发送超�?
    struct timeval timeout = {
        .tv_sec = 0,
        .tv_usec = 10000,  // 10ms
    };
    setsockopt(udp_socket, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout));

    return ESP_OK;
}

// 通过UDP发送单IMU数据（线程安全版本）
void send_single_imu_data_via_udp(
    ImuType imu_type,
    const ImuSample &sample,
    const MadgwickFilterExtended &filter,
    uint32_t timestamp_ms
) {
    // 使用EventGroup检查WiFi连接状态
    EventBits_t bits = xEventGroupGetBits(wifi_event_group);
    if (!(bits & WIFI_CONNECTED_BIT) || udp_socket < 0) {
        static uint32_t warn_count = 0;
        if (++warn_count % 100 == 1) {
            ESP_LOGW(TAG, "WiFi not connected or socket invalid (warnings: %lu)", warn_count);
        }
        return;
    }

    // 构建数据帧
    imu_data_frame_t frame;
    frame.header[0] = 0xAA;
    frame.header[1] = 0x55;
    frame.device_id = static_cast<uint8_t>(imu_type);
    frame.reserved = 0;
    frame.timestamp_ms = timestamp_ms;

    // 获取四元数
    float quat[4];
    filter.get_quaternion_array(quat);

    // 根据IMU类型填充数据
    if (imu_type == ImuType::ICM42688) {
        // 填充arm数据
        for (int i = 0; i < 4; i++) frame.quat_arm[i] = quat[i];
        frame.acc_arm[0] = sample.ax;
        frame.acc_arm[1] = sample.ay;
        frame.acc_arm[2] = sample.az;
        frame.gyro_arm[0] = sample.gx;
        frame.gyro_arm[1] = sample.gy;
        frame.gyro_arm[2] = sample.gz;
        // torso数据填零
        std::memset(frame.quat_torso, 0, sizeof(frame.quat_torso));
        std::memset(frame.acc_torso, 0, sizeof(frame.acc_torso));
        std::memset(frame.gyro_torso, 0, sizeof(frame.gyro_torso));
    } else if (imu_type == ImuType::BMI160) {
        // 填充torso数据
        for (int i = 0; i < 4; i++) frame.quat_torso[i] = quat[i];
        frame.acc_torso[0] = sample.ax;
        frame.acc_torso[1] = sample.ay;
        frame.acc_torso[2] = sample.az;
        frame.gyro_torso[0] = sample.gx;
        frame.gyro_torso[1] = sample.gy;
        frame.gyro_torso[2] = sample.gz;
        // arm数据填零
        std::memset(frame.quat_arm, 0, sizeof(frame.quat_arm));
        std::memset(frame.acc_arm, 0, sizeof(frame.acc_arm));
        std::memset(frame.gyro_arm, 0, sizeof(frame.gyro_arm));
    }

    // 计算CRC
    frame.crc16 = calculate_crc16((uint8_t*)&frame, sizeof(frame) - 2);

    // 拷贝目标地址到局部变量
    struct sockaddr_in local_dest_addr;
    if (dest_addr_mutex && xSemaphoreTake(dest_addr_mutex, 0) == pdTRUE) {
        local_dest_addr = dest_addr;
        xSemaphoreGive(dest_addr_mutex);
    } else {
        local_dest_addr.sin_addr.s_addr = inet_addr(UDP_SERVER_IP);
        local_dest_addr.sin_family = AF_INET;
        local_dest_addr.sin_port = htons(UDP_SERVER_PORT);
    }

    // UDP发送
    int sent = sendto(udp_socket, &frame, sizeof(frame), 0,
                      (struct sockaddr*)&local_dest_addr, sizeof(local_dest_addr));

    static uint32_t send_count = 0;
    static uint32_t error_count = 0;

    if (sent < 0) {
        error_count++;
        if (error_count % 100 == 1) {
            ESP_LOGW(TAG, "UDP send failed: errno %d (total errors: %lu)", errno, error_count);
        }
    } else if (sent != sizeof(frame)) {
        error_count++;
        ESP_LOGW(TAG, "UDP partial send: %d/%d bytes", sent, sizeof(frame));
    } else {
        send_count++;
        if (send_count % 100 == 0) {
            ESP_LOGI(TAG, "UDP sent %lu frames (errors: %lu)", send_count, error_count);
        }
    }
}

// 通过UDP发送IMU数据（线程安全版本，双IMU模式保留）
[[maybe_unused]] void send_imu_data_via_udp(
    const ImuSample &arm_sample,
    const ImuSample &torso_sample,
    const MadgwickFilterExtended &arm_filter,
    const MadgwickFilterExtended &torso_filter,
    uint32_t timestamp_ms
) {
    // 使用EventGroup检查WiFi连接状�?
    EventBits_t bits = xEventGroupGetBits(wifi_event_group);
    if (!(bits & WIFI_CONNECTED_BIT) || udp_socket < 0) {
        static uint32_t warn_count = 0;
        if (++warn_count % 100 == 1) {
            ESP_LOGW(TAG, "WiFi not connected or socket invalid (warnings: %lu)", warn_count);
        }
        return;
    }

    // 构建数据�?
    imu_data_frame_t frame;
    frame.header[0] = 0xAA;
    frame.header[1] = 0x55;
    frame.device_id = 0;  // 双IMU模式
    frame.reserved = 0;
    frame.timestamp_ms = timestamp_ms;

    // 使用临时数组避免packed结构未对齐访�?
    float torso_quat_temp[4];
    float arm_quat_temp[4];
    torso_filter.get_quaternion_array(torso_quat_temp);
    arm_filter.get_quaternion_array(arm_quat_temp);

    for (int i = 0; i < 4; i++) {
        frame.quat_torso[i] = torso_quat_temp[i];
        frame.quat_arm[i] = arm_quat_temp[i];
    }

    frame.acc_torso[0] = torso_sample.ax;
    frame.acc_torso[1] = torso_sample.ay;
    frame.acc_torso[2] = torso_sample.az;
    frame.gyro_torso[0] = torso_sample.gx;
    frame.gyro_torso[1] = torso_sample.gy;
    frame.gyro_torso[2] = torso_sample.gz;

    frame.acc_arm[0] = arm_sample.ax;
    frame.acc_arm[1] = arm_sample.ay;
    frame.acc_arm[2] = arm_sample.az;
    frame.gyro_arm[0] = arm_sample.gx;
    frame.gyro_arm[1] = arm_sample.gy;
    frame.gyro_arm[2] = arm_sample.gz;

    frame.crc16 = calculate_crc16((uint8_t*)&frame, sizeof(frame) - 2);

    // 拷贝目标地址到局部变量（避免发送过程中被修改）
    struct sockaddr_in local_dest_addr;
    if (dest_addr_mutex && xSemaphoreTake(dest_addr_mutex, 0) == pdTRUE) {
        local_dest_addr = dest_addr;
        xSemaphoreGive(dest_addr_mutex);
    } else {
        // 无法获取锁，使用默认地址
        local_dest_addr.sin_addr.s_addr = inet_addr(UDP_SERVER_IP);
        local_dest_addr.sin_family = AF_INET;
        local_dest_addr.sin_port = htons(UDP_SERVER_PORT);
    }

    // UDP发�?
    int sent = sendto(udp_socket, &frame, sizeof(frame), 0,
                      (struct sockaddr*)&local_dest_addr, sizeof(local_dest_addr));

    static uint32_t send_count = 0;
    static uint32_t error_count = 0;

    if (sent < 0) {
        error_count++;
        if (error_count % 100 == 1) {
            ESP_LOGW(TAG, "UDP send failed: errno %d (total errors: %lu)", errno, error_count);
        }
    } else if (sent != sizeof(frame)) {
        error_count++;
        ESP_LOGW(TAG, "UDP partial send: %d/%d bytes", sent, sizeof(frame));
    } else {
        send_count++;
        if (send_count % 100 == 0) {
            ESP_LOGI(TAG, "UDP sent %lu frames (errors: %lu)", send_count, error_count);
        }
    }
}
// RGB LED句柄
static led_strip_handle_t led_strip = nullptr;

// RGB LED初始�?
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
        .mem_block_symbols = 0,
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
        vTaskDelay(pdMS_TO_TICKS(1000));  // 1秒闪烁一�?
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

esp_err_t init_icm42688(Icm42688Context *ctx);
esp_err_t init_bmi160(Bmi160Context *ctx);

bool should_attempt_imu_recover(esp_err_t err) {
    return err == ESP_ERR_INVALID_STATE ||
           err == ESP_ERR_TIMEOUT ||
           err == ESP_ERR_INVALID_RESPONSE ||
           err == ESP_FAIL;
}

void deinit_icm42688(Icm42688Context *ctx) {
    if (ctx == nullptr) {
        return;
    }

    if (ctx->sensor != nullptr) {
        icm42688_delete(ctx->sensor);
        ctx->sensor = nullptr;
    }
    if (ctx->bus != nullptr) {
        i2c_del_master_bus(ctx->bus);
        ctx->bus = nullptr;
    }
    ctx->accel_sensitivity = 1.0f;
    ctx->gyro_sensitivity = 1.0f;
}

void deinit_bmi160(Bmi160Context *ctx) {
    if (ctx == nullptr) {
        return;
    }
    bmi160_free(&ctx->sensor);
    std::memset(&ctx->sensor, 0, sizeof(ctx->sensor));
}

esp_err_t verify_icm_device_id(icm42688_handle_t sensor) {
    if (sensor == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }
    uint8_t dev_id = 0;
    esp_err_t ret = icm42688_get_deviceid(sensor, &dev_id);
    if (ret != ESP_OK) {
        return ret;
    }
    // 支持两个有效的device ID
    if (dev_id != ICM_DEVICE_ID_1 && dev_id != ICM_DEVICE_ID_2) {
        ESP_LOGE(TAG, "ICM42688 device ID mismatch: 0x%02X (expected 0x%02X or 0x%02X)",
                 dev_id, ICM_DEVICE_ID_1, ICM_DEVICE_ID_2);
        return ESP_ERR_NOT_FOUND;
    }
    ESP_LOGI(TAG, "ICM42688 device ID verified: 0x%02X", dev_id);
    return ESP_OK;
}

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
    // 先配置GPIO驱动强度
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_INPUT_OUTPUT_OD;  // 开漏模�?
    io_conf.pin_bit_mask = (1ULL << ICM_I2C_SDA) | (1ULL << ICM_I2C_SCL);
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;  // 启用内部上拉
    gpio_config(&io_conf);

    // 设置强驱�?
    gpio_set_drive_capability(ICM_I2C_SDA, GPIO_DRIVE_CAP_3);  // 最强驱�?
    gpio_set_drive_capability(ICM_I2C_SCL, GPIO_DRIVE_CAP_3);

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

// IMU轻量级检测函数 - 只读chip id，不做完整初始化
DetectedImu detect_available_imu() {
    DetectedImu result = {ImuType::NONE, I2C_NUM_0, GPIO_NUM_8, GPIO_NUM_9};

    // 1. 尝试检测I2C0上的ICM42688
    ESP_LOGI(TAG, "Detecting IMU on I2C0 (GPIO8/9)...");
    i2c_master_bus_handle_t icm_bus = nullptr;
    if (init_icm_bus(&icm_bus) == ESP_OK) {
        icm42688_handle_t icm_sensor = nullptr;
        esp_err_t create_ret = icm42688_create(icm_bus, ICM_I2C_ADDRESS, &icm_sensor);
        if (create_ret == ESP_OK || (create_ret == ESP_ERR_INVALID_STATE && icm_sensor != nullptr)) {
            uint8_t dev_id = 0;
            if (icm42688_get_deviceid(icm_sensor, &dev_id) == ESP_OK &&
                (dev_id == ICM_DEVICE_ID_1 || dev_id == ICM_DEVICE_ID_2)) {
                ESP_LOGI(TAG, "Detected ICM42688 on I2C0 (chip_id=0x%02X)", dev_id);
                icm42688_delete(icm_sensor);  // 修复：传句柄本身，不是地址
                i2c_del_master_bus(icm_bus);
                result.type = ImuType::ICM42688;
                result.port = I2C_NUM_0;
                result.sda = GPIO_NUM_8;
                result.scl = GPIO_NUM_9;
                return result;
            }
            icm42688_delete(icm_sensor);  // 修复：传句柄本身，不是地址
        }
        i2c_del_master_bus(icm_bus);
    }
    ESP_LOGW(TAG, "No ICM42688 found on I2C0");

    // 2. 尝试检测I2C1上的BMI160 - 只使用0x69地址
    ESP_LOGI(TAG, "Detecting IMU on I2C1 (GPIO16/17)...");

    // 初始化i2cdev（BMI160依赖）
    if (i2cdev_init() == ESP_OK) {
        bmi160_t bmi_test = {};

        // 只尝试0x69地址（VDD）- 硬件确定使用此地址
        esp_err_t ret = bmi160_init(&bmi_test, BMI160_I2C_ADDRESS_VDD, BMI_I2C_PORT, BMI_I2C_SDA, BMI_I2C_SCL);
        if (ret == ESP_OK) {
            uint8_t chip_id = 0;
            if (bmi160_read_reg(&bmi_test, 0x00, &chip_id) == ESP_OK && chip_id == 0xD1) {
                ESP_LOGI(TAG, "Detected BMI160 on I2C1 at 0x69 (chip_id=0x%02X)", chip_id);
                bmi160_free(&bmi_test);
                result.type = ImuType::BMI160;
                result.port = static_cast<i2c_port_t>(I2C_NUM_1);
                result.sda = GPIO_NUM_16;
                result.scl = GPIO_NUM_17;
                return result;
            }
            bmi160_free(&bmi_test);
        }
    }
    ESP_LOGW(TAG, "No BMI160 found on I2C1");

    ESP_LOGE(TAG, "No IMU detected on either I2C bus!");
    return result;
}

esp_err_t init_icm42688(Icm42688Context *ctx) {
    if (ctx == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }
    deinit_icm42688(ctx);
    vTaskDelay(IMU_BUS_SETTLE_TICKS);
    ESP_RETURN_ON_ERROR(init_icm_bus(&ctx->bus), TAG, "failed to create ICM42688 I2C bus");

    esp_err_t create_ret = icm42688_create(ctx->bus, ICM_I2C_ADDRESS, &ctx->sensor);
    if (create_ret != ESP_OK) {
        // icm42688_create may return soft-reset related errors even when handle is valid.
        if (!(create_ret == ESP_ERR_INVALID_STATE && ctx->sensor != nullptr)) {
            ESP_RETURN_ON_ERROR(create_ret, TAG, "failed to create ICM42688 device");
        }
        ESP_LOGW(TAG, "ICM42688 create returned %s, continue with explicit ID verify",
                 esp_err_to_name(create_ret));
    }

    ESP_RETURN_ON_ERROR(verify_icm_device_id(ctx->sensor), TAG, "failed to verify ICM42688 ID");

    const icm42688_cfg_t config = {
        .acce_fs = ACCE_FS_4G,
        .acce_odr = ACCE_ODR_100HZ,
        .gyro_fs = GYRO_FS_500DPS,
        .gyro_odr = GYRO_ODR_100HZ,
    };

    ESP_RETURN_ON_ERROR(icm42688_config(ctx->sensor, &config), TAG, "failed to configure ICM42688");
    ESP_RETURN_ON_ERROR(icm42688_set_power_mode(ctx->sensor, ACCE_PWR_ON, GYRO_PWR_ON, TEMP_PWR_ON), TAG,
                        "failed to power on ICM42688");
    vTaskDelay(pdMS_TO_TICKS(10));
    ESP_RETURN_ON_ERROR(icm42688_get_acce_sensitivity(ctx->sensor, &ctx->accel_sensitivity), TAG,
                        "failed to get ICM42688 accel sensitivity");
    ESP_RETURN_ON_ERROR(icm42688_get_gyro_sensitivity(ctx->sensor, &ctx->gyro_sensitivity), TAG,
                        "failed to get ICM42688 gyro sensitivity");

    return ESP_OK;
}

esp_err_t init_bmi160_at_address(Bmi160Context *ctx, uint8_t address) {
    if (ctx == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }
    deinit_bmi160(ctx);
    ESP_RETURN_ON_ERROR(i2cdev_init(), TAG, "failed to init i2cdev");
    std::memset(&ctx->sensor, 0, sizeof(ctx->sensor));

    ESP_RETURN_ON_ERROR(bmi160_init(&ctx->sensor, address, BMI_I2C_PORT, BMI_I2C_SDA, BMI_I2C_SCL), TAG,
                        "failed to init BMI160");

    // Software reset to ensure known state
    ESP_LOGI(TAG, "BMI160 performing soft reset...");
    ESP_RETURN_ON_ERROR(bmi160_write_reg(&ctx->sensor, 0x7E, 0xB6), TAG,
                        "failed to soft-reset BMI160");
    vTaskDelay(pdMS_TO_TICKS(300));  // 增加到300ms，确保复位完成

    // Verify chip ID
    uint8_t chip_id = 0;
    ESP_RETURN_ON_ERROR(bmi160_read_reg(&ctx->sensor, 0x00, &chip_id), TAG,
                        "failed to read BMI160 chip ID");
    ESP_LOGI(TAG, "BMI160 chip ID: 0x%02X (expect 0xD1)", chip_id);
    if (chip_id != 0xD1) {
        ESP_LOGE(TAG, "BMI160 chip ID mismatch!");
        return ESP_ERR_INVALID_RESPONSE;
    }

    auto wait_bmi160_pmu = [&](uint8_t mask, uint8_t expected, uint8_t shift, const char *name) -> esp_err_t {
        for (int i = 0; i < 50; ++i) {  // 增加到50次，最多1000ms
            uint8_t pmu = 0;
            esp_err_t ret = bmi160_read_reg(&ctx->sensor, 0x03, &pmu);
            if (ret != ESP_OK) {
                ESP_LOGW(TAG, "BMI160 %s PMU read failed: %s", name, esp_err_to_name(ret));
                return ret;
            }
            uint8_t actual = (pmu & mask) >> shift;
            if (i % 5 == 0) {  // 每100ms打印一次
                ESP_LOGI(TAG, "BMI160 %s PMU polling: actual=0x%02X expect=0x%02X (raw=0x%02X)",
                         name, actual, expected, pmu);
            }
            if (actual == expected) {
                ESP_LOGI(TAG, "BMI160 %s PMU ready after %d ms", name, i * 20);
                return ESP_OK;
            }
            vTaskDelay(pdMS_TO_TICKS(20));
        }

        uint8_t pmu = 0;
        uint8_t err = 0;
        bmi160_read_reg(&ctx->sensor, 0x03, &pmu);  // PMU_STATUS
        bmi160_read_reg(&ctx->sensor, 0x02, &err);  // ERR_REG
        ESP_LOGE(TAG, "BMI160 %s PMU wait timeout: PMU=0x%02X ERR=0x%02X", name, pmu, err);
        return ESP_ERR_TIMEOUT;
    };

    // 使用显式命令+轮询，比库内一次性检查更稳健
    ESP_LOGI(TAG, "BMI160 setting accel to NORMAL mode...");
    ESP_RETURN_ON_ERROR(bmi160_write_reg(&ctx->sensor, 0x7E, BMI160_PMU_ACC_NORMAL), TAG,
                        "failed to set BMI160 accel normal mode");
    ESP_RETURN_ON_ERROR(wait_bmi160_pmu(0x30, 0x01, 4, "ACC"), TAG,
                        "failed to wait BMI160 accel PMU");

    // 尝试先设置陀螺仪为FAST_STARTUP，再切到NORMAL（某些芯片需要这个步骤）
    ESP_LOGI(TAG, "BMI160 setting gyro to FAST_STARTUP mode...");
    ESP_RETURN_ON_ERROR(bmi160_write_reg(&ctx->sensor, 0x7E, 0x17), TAG,  // FAST_STARTUP
                        "failed to set BMI160 gyro fast startup mode");
    vTaskDelay(pdMS_TO_TICKS(100));  // 等待FAST_STARTUP完成

    ESP_LOGI(TAG, "BMI160 setting gyro to NORMAL mode...");
    ESP_RETURN_ON_ERROR(bmi160_write_reg(&ctx->sensor, 0x7E, BMI160_PMU_GYR_NORMAL), TAG,
                        "failed to set BMI160 gyro normal mode");
    ESP_RETURN_ON_ERROR(wait_bmi160_pmu(0x0C, 0x01, 2, "GYR"), TAG,
                        "failed to wait BMI160 gyro PMU");
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
    ESP_LOGI(TAG, "BMI160 initialized at address 0x%02X", address);

    return ESP_OK;
}

esp_err_t init_bmi160(Bmi160Context *ctx) {
    if (ctx == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }

    // 初始化阶段仅使用工程已确认地址，避免在错误地址上反复重建句柄污染总线状态
    ESP_LOGI(TAG, "Trying BMI160 address 0x%02X on I2C%d (SDA=%d SCL=%d)",
             BMI_I2C_ADDRESS, BMI_I2C_PORT, BMI_I2C_SDA, BMI_I2C_SCL);
    esp_err_t ret = init_bmi160_at_address(ctx, BMI_I2C_ADDRESS);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "BMI160 init failed at 0x%02X: %s", BMI_I2C_ADDRESS, esp_err_to_name(ret));
    }
    return ret;
}

esp_err_t init_icm42688_with_retry(Icm42688Context *ctx, int max_retries) {
    esp_err_t last_err = ESP_FAIL;
    for (int attempt = 1; attempt <= max_retries; ++attempt) {
        last_err = init_icm42688(ctx);
        if (last_err == ESP_OK) {
            if (attempt > 1) {
                ESP_LOGW(TAG, "ICM42688 initialized after retry %d/%d", attempt, max_retries);
            }
            return ESP_OK;
        }
        ESP_LOGW(TAG, "ICM42688 init attempt %d/%d failed: %s",
                 attempt, max_retries, esp_err_to_name(last_err));
        vTaskDelay(pdMS_TO_TICKS(50 * attempt));
    }
    return last_err;
}

esp_err_t init_bmi160_with_retry(Bmi160Context *ctx, int max_retries) {
    esp_err_t last_err = ESP_FAIL;
    for (int attempt = 1; attempt <= max_retries; ++attempt) {
        last_err = init_bmi160(ctx);
        if (last_err == ESP_OK) {
            if (attempt > 1) {
                ESP_LOGW(TAG, "BMI160 initialized after retry %d/%d", attempt, max_retries);
            }
            return ESP_OK;
        }
        ESP_LOGW(TAG, "BMI160 init attempt %d/%d failed: %s",
                 attempt, max_retries, esp_err_to_name(last_err));
        vTaskDelay(pdMS_TO_TICKS(50 * attempt));
    }
    return last_err;
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

    esp_err_t ret = ESP_FAIL;
    for (int attempt = 1; attempt <= ICM_READ_RETRY_COUNT; ++attempt) {
        ret = icm42688_get_acce_raw_value(ctx.sensor, &accel_raw);
        if (ret == ESP_OK) {
            break;
        }
        if (attempt < ICM_READ_RETRY_COUNT) {
            vTaskDelay(ICM_READ_RETRY_DELAY_TICKS);
        }
    }
    if (ret != ESP_OK) {
        return ret;
    }

    for (int attempt = 1; attempt <= ICM_READ_RETRY_COUNT; ++attempt) {
        ret = icm42688_get_gyro_raw_value(ctx.sensor, &gyro_raw);
        if (ret == ESP_OK) {
            break;
        }
        if (attempt < ICM_READ_RETRY_COUNT) {
            vTaskDelay(ICM_READ_RETRY_DELAY_TICKS);
        }
    }
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

esp_err_t calibrate_icm42688_gyro(const Icm42688Context &ctx, GyroBias *bias_out) {
    if (bias_out == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }

    GyroBias bias {};
    int valid_samples = 0;
    int read_errors = 0;
    int motion_rejects = 0;
    int64_t start_time_ms = esp_timer_get_time() / 1000;

    ESP_LOGI(TAG, "Calibrating ICM42688 gyro bias, keep device still");
    for (int i = 0; i < CALIBRATION_SAMPLES; ++i) {
        int64_t elapsed_ms = (esp_timer_get_time() / 1000) - start_time_ms;
        if (elapsed_ms > MAX_CALIBRATION_TIME_MS) {
            ESP_LOGW(TAG, "ICM42688 calibration timeout after %lld ms", elapsed_ms);
            break;
        }

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

    if (valid_samples < MIN_VALID_SAMPLES) {
        ESP_LOGE(TAG, "ICM42688 calibration failed: only %d valid samples (minimum: %d)",
                 valid_samples, MIN_VALID_SAMPLES);
        ESP_LOGE(TAG, "ICM42688 calibration stats: errors=%d, motion_rejects=%d",
                 read_errors, motion_rejects);
        *bias_out = GyroBias{};
        return ESP_ERR_INVALID_STATE;
    }

    const float scale = 1.0f / static_cast<float>(valid_samples);
    bias.x *= scale;
    bias.y *= scale;
    bias.z *= scale;

    ESP_LOGI(TAG, "ICM42688 gyro bias[rad/s]=[%+.5f, %+.5f, %+.5f] from %d still samples (errors: %d, motion: %d)",
             bias.x, bias.y, bias.z, valid_samples, read_errors, motion_rejects);
    *bias_out = bias;
    return ESP_OK;
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

esp_err_t calibrate_bmi160_gyro(Bmi160Context *ctx, GyroBias *bias_out) {
    if (ctx == nullptr || bias_out == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }

    GyroBias bias {};
    int valid_samples = 0;
    int read_errors = 0;
    int motion_rejects = 0;
    int64_t start_time_ms = esp_timer_get_time() / 1000;

    ESP_LOGI(TAG, "Calibrating BMI160 gyro bias, keep device still");
    for (int i = 0; i < CALIBRATION_SAMPLES; ++i) {
        int64_t elapsed_ms = (esp_timer_get_time() / 1000) - start_time_ms;
        if (elapsed_ms > MAX_CALIBRATION_TIME_MS) {
            ESP_LOGW(TAG, "BMI160 calibration timeout after %lld ms", elapsed_ms);
            break;
        }

        ImuSample sample {};
        const esp_err_t ret = read_bmi160_sample(ctx, &sample);
        if (ret != ESP_OK) {
            read_errors++;
            if (read_errors % 50 == 1) {
                ESP_LOGW(TAG, "BMI160 read error: %s (total: %d)", esp_err_to_name(ret), read_errors);
            }
        } else if (is_sample_still_for_calibration(sample)) {
            bias.x += sample.gx;
            bias.y += sample.gy;
            bias.z += sample.gz;
            ++valid_samples;
            if (valid_samples % 50 == 0) {
                ESP_LOGI(TAG, "BMI160 calibration progress: %d/%d valid samples", valid_samples, CALIBRATION_SAMPLES);
            }
        } else {
            motion_rejects++;
            if (motion_rejects % 50 == 1) {
                float gyro_norm = vector_norm3(sample.gx, sample.gy, sample.gz);
                float accel_norm = vector_norm3(sample.ax, sample.ay, sample.az);
                ESP_LOGW(TAG, "BMI160 motion detected: gyro_norm=%.3f accel_norm=%.3f (rejects: %d)",
                         gyro_norm, accel_norm, motion_rejects);
            }
        }
        vTaskDelay(CALIBRATION_SAMPLE_DELAY_TICKS);
    }

    if (valid_samples < MIN_VALID_SAMPLES) {
        ESP_LOGE(TAG, "BMI160 calibration failed: only %d valid samples (minimum: %d)",
                 valid_samples, MIN_VALID_SAMPLES);
        ESP_LOGE(TAG, "BMI160 calibration stats: errors=%d, motion_rejects=%d",
                 read_errors, motion_rejects);
        *bias_out = GyroBias{};
        return ESP_ERR_INVALID_STATE;
    }

    const float scale = 1.0f / static_cast<float>(valid_samples);
    bias.x *= scale;
    bias.y *= scale;
    bias.z *= scale;

    ESP_LOGI(TAG, "BMI160 gyro bias[rad/s]=[%+.5f, %+.5f, %+.5f] from %d still samples (errors: %d, motion: %d)",
             bias.x, bias.y, bias.z, valid_samples, read_errors, motion_rejects);
    *bias_out = bias;
    return ESP_OK;
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
    for (int i = 0; i < FILTER_INIT_RETRY_COUNT; ++i) {
        const esp_err_t ret = read_icm42688_sample(ctx, &sample);
        if (ret == ESP_OK) {
            apply_gyro_bias(&sample, bias);
            initialize_filter_from_accel(filter, sample);
            return true;
        }
        vTaskDelay(FILTER_INIT_RETRY_DELAY_TICKS);
    }
    ESP_LOGW(TAG, "failed to initialize ICM42688 filter from accel");
    return false;
}

bool initialize_filter_from_bmi(espp::MadgwickFilter *filter, Bmi160Context *ctx, const GyroBias &bias) {
    ImuSample sample {};
    // Retry a few times in case data is not ready yet
    for (int i = 0; i < FILTER_INIT_RETRY_COUNT; ++i) {
        esp_err_t ret = read_bmi160_sample(ctx, &sample);
        if (ret == ESP_OK) {
            apply_gyro_bias(&sample, bias);
            initialize_filter_from_accel(filter, sample);
            return true;
        }
        vTaskDelay(FILTER_INIT_RETRY_DELAY_TICKS);
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

    // 注释掉启动扫描，避免无设备时产生大量超时干扰诊断
    // i2c_master_bus_handle_t scan_bus0 = nullptr;
    // i2c_master_bus_handle_t scan_bus1 = nullptr;
    //
    // if (init_scan_bus(static_cast<i2c_port_t>(ICM_I2C_PORT), ICM_I2C_SDA, ICM_I2C_SCL, &scan_bus0) == ESP_OK) {
    //     scan_i2c_bus(scan_bus0, "scan/icm_bus");
    //     i2c_del_master_bus(scan_bus0);
    //     vTaskDelay(IMU_BUS_SETTLE_TICKS);
    // } else {
    //     ESP_LOGW(TAG, "failed to create scan bus for ICM bus");
    // }
    //
    // if (init_scan_bus(BMI_I2C_PORT, BMI_I2C_SDA, BMI_I2C_SCL, &scan_bus1) == ESP_OK) {
    //     scan_i2c_bus(scan_bus1, "scan/bmi_bus");
    //     i2c_del_master_bus(scan_bus1);
    //     vTaskDelay(IMU_BUS_SETTLE_TICKS);
    // } else {
    //     ESP_LOGW(TAG, "failed to create scan bus for BMI bus");
    // }

    // 自动检测IMU
    DetectedImu detected = detect_available_imu();
    const bool imu_detected = (detected.type != ImuType::NONE);
    ImuType runtime_imu_type = ImuType::NONE;
    if (!imu_detected) {
        ESP_LOGE(TAG, "No IMU detected, continue in network-only degraded mode");
    }

    Icm42688Context icm {};
    Bmi160Context bmi {};
    MadgwickFilterExtended filter(MADGWICK_BETA_RUN);
    GyroBias gyro_bias {};

    if (imu_detected) {
        if (detected.type == ImuType::ICM42688) {
            ESP_LOGI(TAG, "Initializing ICM42688 system (device_id=1)");
            esp_err_t init_ret = init_icm42688_with_retry(&icm, IMU_INIT_MAX_RETRIES);
            if (init_ret == ESP_OK) {
                runtime_imu_type = ImuType::ICM42688;
                ESP_LOGI(TAG, "ICM42688 init success on I2C%d SDA=%d SCL=%d", ICM_I2C_PORT, ICM_I2C_SDA, ICM_I2C_SCL);
            } else {
                ESP_LOGE(TAG, "ICM42688 init failed after retries: %s, degrade to network-only",
                         esp_err_to_name(init_ret));
            }
        } else if (detected.type == ImuType::BMI160) {
            ESP_LOGI(TAG, "Initializing BMI160 system (device_id=2)");
            esp_err_t init_ret = init_bmi160_with_retry(&bmi, IMU_INIT_MAX_RETRIES);
            if (init_ret == ESP_OK) {
                runtime_imu_type = ImuType::BMI160;
                ESP_LOGI(TAG, "BMI160 init success on I2C%d SDA=%d SCL=%d", BMI_I2C_PORT, BMI_I2C_SDA, BMI_I2C_SCL);
                dump_bmi160_raw(&bmi, 20);
            } else {
                ESP_LOGE(TAG, "BMI160 init failed after retries: %s, degrade to network-only",
                         esp_err_to_name(init_ret));
            }
        }
    }

    const bool imu_runtime_ready = (runtime_imu_type != ImuType::NONE);

    if (imu_runtime_ready) {
        ESP_LOGI(TAG, "Warm up IMU for %lu ms before calibration, keep device still",
                 static_cast<unsigned long>(CALIBRATION_WARMUP_TICKS * portTICK_PERIOD_MS));
        vTaskDelay(CALIBRATION_WARMUP_TICKS);

        esp_err_t cal_ret = ESP_FAIL;
        bool filter_init = false;

        for (int attempt = 1; attempt <= STARTUP_CALIBRATION_MAX_ATTEMPTS; ++attempt) {
            ESP_LOGI(TAG, "Startup calibration attempt %d/%d", attempt, STARTUP_CALIBRATION_MAX_ATTEMPTS);
            filter_init = false;

            if (runtime_imu_type == ImuType::ICM42688) {
                cal_ret = calibrate_icm42688_gyro(icm, &gyro_bias);
                if (cal_ret == ESP_OK) {
                    filter_init = initialize_filter_from_icm(&filter, icm, gyro_bias);
                }
            } else if (runtime_imu_type == ImuType::BMI160) {
                cal_ret = calibrate_bmi160_gyro(&bmi, &gyro_bias);
                if (cal_ret == ESP_OK) {
                    filter_init = initialize_filter_from_bmi(&filter, &bmi, gyro_bias);
                }
            }

            if (filter_init) {
                break;
            }

            ESP_LOGW(TAG, "Startup calibration/init attempt %d failed: cal=%s filter=%d",
                     attempt, esp_err_to_name(cal_ret), filter_init ? 1 : 0);

            if (attempt < STARTUP_CALIBRATION_MAX_ATTEMPTS) {
                vTaskDelay(STARTUP_CALIBRATION_RETRY_DELAY_TICKS);
            }
        }

        if (!filter_init) {
            ESP_LOGE(TAG, "Startup calibration/filter init incomplete, continue in degraded mode");
            gyro_bias = GyroBias{};
            filter.set_quaternion(1.0f, 0.0f, 0.0f, 0.0f);
        }
    } else {
        // 无IMU时保持姿态为单位四元数，主循环仅维持联网与任务存活
        gyro_bias = GyroBias{};
        filter.set_quaternion(1.0f, 0.0f, 0.0f, 0.0f);
    }

    ESP_LOGI(TAG, "Attitude initialization complete, entering normal output phase");

    // 初始化WiFi Station
    ESP_LOGI(TAG, "Initializing WiFi Station...");
    ESP_ERROR_CHECK(init_wifi_sta());

    // 等待WiFi连接（使用EventGroup�?
    ESP_LOGI(TAG, "Waiting for WiFi connection...");
    EventBits_t bits = xEventGroupWaitBits(wifi_event_group,
                                            WIFI_CONNECTED_BIT,
                                            pdFALSE,
                                            pdFALSE,
                                            pdMS_TO_TICKS(10000));  // 10秒超�?

    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "WiFi connected successfully");
    } else {
        ESP_LOGW(TAG, "WiFi connection timeout, continuing anyway");
    }

    // 初始化UDP Client
    ESP_ERROR_CHECK(init_udp_client());

    int64_t last_update_us = esp_timer_get_time();
    int64_t last_recover_us = 0;
    int consecutive_failures = 0;
    uint32_t error_total = 0;

    while (true) {
        int64_t now_us = esp_timer_get_time();
        float dt_s = static_cast<float>(now_us - last_update_us) / 1000000.0f;
        last_update_us = now_us;
        if (dt_s <= 0.0f) {
            dt_s = 0.01f;
        }

        ImuSample sample {};
        EulerAngles euler {};
        bool read_success = false;
        esp_err_t read_ret = ESP_FAIL;

        // 根据检测到的IMU类型读取数据
        if (runtime_imu_type == ImuType::ICM42688) {
            read_ret = read_icm42688_sample(icm, &sample);
            if (read_ret == ESP_OK) {
                apply_gyro_bias(&sample, gyro_bias);
                euler = update_filter(&filter, sample, dt_s);
                read_success = true;
            }
        } else if (runtime_imu_type == ImuType::BMI160) {
            read_ret = read_bmi160_sample(&bmi, &sample);
            if (read_ret == ESP_OK) {
                apply_gyro_bias(&sample, gyro_bias);
                euler = update_filter(&filter, sample, dt_s);
                read_success = true;
            }
        } else {
            static uint32_t no_imu_log_counter = 0;
            if (++no_imu_log_counter % 200 == 1) {
                ESP_LOGW(TAG, "No IMU present, running network-only loop");
            }
        }

        // 错误处理和恢复逻辑
        if (!read_success) {
            if (imu_runtime_ready) {
                consecutive_failures++;
                error_total++;
                if (consecutive_failures == 1 || (consecutive_failures % 10) == 0) {
                    ESP_LOGW(TAG, "Failed to read IMU sample: %s (consecutive=%d total=%lu)",
                             esp_err_to_name(read_ret), consecutive_failures, error_total);
                }

                bool trigger_by_burst = consecutive_failures >= IMU_RECOVER_TRIGGER_CONSECUTIVE_ERRORS;
                bool trigger_by_cooldown = (now_us - last_recover_us) >= IMU_RECOVER_COOLDOWN_US;

                if (should_attempt_imu_recover(read_ret) && (trigger_by_burst || trigger_by_cooldown)) {
                    last_recover_us = now_us;
                    ESP_LOGW(TAG, "Attempting IMU runtime recover (consecutive=%d)...", consecutive_failures);

                    esp_err_t recover_ret = ESP_FAIL;
                    if (runtime_imu_type == ImuType::ICM42688) {
                        recover_ret = init_icm42688_with_retry(&icm, IMU_RUNTIME_RECOVER_RETRIES);
                    } else if (runtime_imu_type == ImuType::BMI160) {
                        recover_ret = init_bmi160_with_retry(&bmi, IMU_RUNTIME_RECOVER_RETRIES);
                    }

                    if (recover_ret == ESP_OK) {
                        ESP_LOGI(TAG, "IMU runtime recover success");
                        consecutive_failures = 0;
                    } else {
                        ESP_LOGE(TAG, "IMU runtime recover failed: %s", esp_err_to_name(recover_ret));
                    }
                }
            }
        } else {
            consecutive_failures = 0;

            // 定期打印姿态数据
            static uint32_t attitude_log_counter = 0;
            if (++attitude_log_counter % 25 == 0) {
                const char *imu_name = (runtime_imu_type == ImuType::ICM42688) ? "arm/icm42688" : "torso/bmi160";
                log_sample_and_attitude(imu_name, sample, euler);
            }
        }

        // 发送单IMU数据
        if (imu_runtime_ready) {
            uint32_t timestamp_ms = static_cast<uint32_t>(now_us / 1000);
            send_single_imu_data_via_udp(runtime_imu_type, sample, filter, timestamp_ms);
        }

        vTaskDelay(SAMPLE_PERIOD_TICKS);
    }
}
