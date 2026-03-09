# ESP-Hosted WiFi UDP传输方案书

## 项目背景

### 当前实现
- 单IMU自动识别系统，支持ICM42688和BMI160传感器
- 通过WiFi UDP发送90字节数据帧到ESP32-P4
- 自动检测传感器类型并设置对应的device_id

### 解决方案
采用ESP-Hosted + WiFi UDP传输，利用官方成熟方案

---

## 系统架构

### 硬件拓扑

```
┌─────────────────────────────────────────────────────────────┐
│                     ESP32-S3 (IMU Device)                    │
│                                                              │
│  ┌──────────────────────────────────────────────────────┐   │
│  │         自动检测IMU传感器                             │   │
│  │  ┌──────────┐         ┌──────────┐                  │   │
│  │  │ ICM42688 │  或     │ BMI160   │                  │   │
│  │  │ (I2C0)   │         │ (I2C1)   │                  │   │
│  │  │ GPIO8/9  │         │ GPIO16/17│                  │   │
│  │  └─────┬────┘         └─────┬────┘                  │   │
│  │        │                    │                        │   │
│  │        └────────┬───────────┘                        │   │
│  │                 │                                    │   │
│  │        ┌────────▼────────┐                           │   │
│  │        │  检测到ICM42688 │                           │   │
│  │        │  → device_id=1  │                           │   │
│  │        │  → 填充arm字段  │                           │   │
│  │        │                 │                           │   │
│  │        │  检测到BMI160   │                           │   │
│  │        │  → device_id=2  │                           │   │
│  │        │  → 填充torso字段│                           │   │
│  │        └────────┬────────┘                           │   │
│  └─────────────────┼────────────────────────────────────┘   │
│                    │                                         │
│         ┌──────────▼───────────┐                             │
│         │  Madgwick滤波器      │                             │
│         │  姿态解算            │                             │
│         │  数据打包(90字节)    │                             │
│         │  CRC校验             │                             │
│         └──────────┬───────────┘                             │
│                    │                                         │
│         ┌──────────▼───────────┐                             │
│         │   WiFi Station       │                             │
│         │   UDP Client         │                             │
│         │   静态IP: 192.168.4.2│                             │
│         │   目标: 192.168.4.1  │                             │
│         └──────────┬───────────┘                             │
└────────────────────┼─────────────────────────────────────────┘
                     │
                     │ WiFi 2.4GHz
                     │ UDP数据包 (90字节)
                     │
┌────────────────────▼─────────────────────────────────────────┐
│                     ESP32-P4 + C6系统                        │
│                                                              │
│  ┌──────────────────────────────────────────────────────┐   │
│  │              ESP32-C6 (WiFi协处理器)                  │   │
│  │                                                       │   │
│  │  ┌──────────────────┐    ┌──────────────────────┐   │   │
│  │  │  WiFi SoftAP     │◀───│  S3连接              │   │   │
│  │  │  192.168.4.1     │    │  192.168.4.2         │   │   │
│  │  └────────┬─────────┘    └──────────────────────┘   │   │
│  │           │                                          │   │
│  │  ┌────────▼─────────┐    ┌──────────────────────┐   │   │
│  │  │  UDP Server      │───▶│  ESP-Hosted          │   │   │
│  │  │  Port 8888       │    │  SDIO Slave          │   │   │
│  │  └──────────────────┘    └──────────┬───────────┘   │   │
│  └────────────────────────────────────┼─────────────────┘   │
│                                        │ SDIO接口           │
│                             ┌──────────▼───────────┐        │
│                             │  ESP32-P4主控        │        │
│                             │  SDIO Host           │        │
│                             │  lwIP TCP/IP栈       │        │
│                             │  UDP Client读取      │        │
│                             └──────────────────────┘        │
└─────────────────────────────────────────────────────────────┘
```

---

## 技术方案

### 方案A：C6作为SoftAP（推荐）

#### 优势
✓ 无需外部路由器
✓ 点对点连接，延迟最低
✓ 网络隔离，不受外界干扰
✓ 配置简单

#### 网络配置
```
C6 SoftAP:    192.168.4.1
S3 Station:   192.168.4.2 (静态IP)
UDP目标:      192.168.4.1:8888
P4通过C6:     192.168.4.1 (共享C6的IP)
```

---

### 方案B：连接同一路由器

#### 优势
✓ 可以同时连接其他设备
✓ 可以远程访问
✓ 便于调试

#### 网络配置
```
路由器:       192.168.1.1
C6 Station:   192.168.1.100 (DHCP)
S3 Station:   192.168.1.101 (DHCP)
P4通过C6:     192.168.1.100 (共享C6的IP)
```

---

## 详细实现

### 1. ESP32-S3端（UDP Client）

#### 1.1 WiFi Station配置（静态IP）
```c
#include "esp_wifi.h"
#include "esp_event.h"
#include "lwip/sockets.h"

// WiFi配置
#define WIFI_SSID      "ESP32_C6_AP"      // C6的SoftAP名称
#define WIFI_PASS      "12345678"
#define UDP_SERVER_IP  "192.168.4.1"      // C6的IP
#define UDP_SERVER_PORT 8888

// WiFi初始化（静态IP）
void wifi_init_sta() {
    esp_netif_init();
    esp_event_loop_create_default();

    // 创建Station接口
    esp_netif_t *sta_netif = esp_netif_create_default_wifi_sta();

    // 配置静态IP
    esp_netif_dhcpc_stop(sta_netif);  // 停止DHCP客户端

    esp_netif_ip_info_t ip_info;
    IP4_ADDR(&ip_info.ip, 192, 168, 4, 2);       // S3静态IP
    IP4_ADDR(&ip_info.gw, 192, 168, 4, 1);       // 网关(C6)
    IP4_ADDR(&ip_info.netmask, 255, 255, 255, 0); // 子网掩码
    esp_netif_set_ip_info(sta_netif, &ip_info);

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
        },
    };

    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
    esp_wifi_start();
    esp_wifi_connect();

    ESP_LOGI(TAG, "WiFi Station initialized with static IP: 192.168.4.2");
}
```

#### 1.2 UDP Socket创建
```c
int udp_socket = -1;
struct sockaddr_in dest_addr;

void udp_client_init() {
    // 创建UDP socket
    udp_socket = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (udp_socket < 0) {
        ESP_LOGE(TAG, "Unable to create socket");
        return;
    }

    // 配置目标地址
    dest_addr.sin_addr.s_addr = inet_addr(UDP_SERVER_IP);
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port = htons(UDP_SERVER_PORT);

    // 设置socket为非阻塞
    int flags = fcntl(udp_socket, F_GETFL, 0);
    fcntl(udp_socket, F_SETFL, flags | O_NONBLOCK);

    ESP_LOGI(TAG, "UDP client initialized, target: %s:%d",
             UDP_SERVER_IP, UDP_SERVER_PORT);
}
```

#### 1.3 数据帧结构定义
```c
// 90字节数据帧结构
typedef struct __attribute__((packed)) {
    uint8_t header[2];          // 0xAA 0x55
    uint8_t device_id;          // 1=arm(ICM42688), 2=torso(BMI160)
    uint8_t reserved;           // 保留字节
    uint32_t timestamp_ms;      // 时间戳毫秒
    float quat_torso[4];        // 躯干四元数 w,x,y,z
    float quat_arm[4];          // 手臂四元数 w,x,y,z
    float acc_torso[3];         // 躯干加速度 ax,ay,az (g)
    float gyro_torso[3];        // 躯干角速度 gx,gy,gz (rad/s)
    float acc_arm[3];           // 手臂加速度 ax,ay,az (g)
    float gyro_arm[3];          // 手臂角速度 gx,gy,gz (rad/s)
    uint16_t crc16;             // CRC-16/MODBUS
} imu_data_frame_t;  // 总计90字节
```

#### 1.4 单IMU数据发送
```c
void send_imu_data_via_udp(
    const ImuSample &sample,
    const MadgwickFilterExtended &filter,
    uint32_t timestamp_ms,
    uint8_t device_id  // 1=ICM42688(arm), 2=BMI160(torso)
) {
    if (udp_socket < 0) {
        return;  // Socket未初始化
    }

    // 构建数据帧（90字节）
    imu_data_frame_t frame;
    memset(&frame, 0, sizeof(frame));  // 先清零所有字段

    frame.header[0] = 0xAA;
    frame.header[1] = 0x55;
    frame.device_id = device_id;
    frame.reserved = 0;
    frame.timestamp_ms = timestamp_ms;

    // 根据device_id填充对应字段
    if (device_id == 1) {
        // ICM42688 -> arm字段
        filter.get_quaternion_array(frame.quat_arm);
        frame.acc_arm[0] = sample.ax;
        frame.acc_arm[1] = sample.ay;
        frame.acc_arm[2] = sample.az;
        frame.gyro_arm[0] = sample.gx;
        frame.gyro_arm[1] = sample.gy;
        frame.gyro_arm[2] = sample.gz;
        // torso字段保持为0
    } else if (device_id == 2) {
        // BMI160 -> torso字段
        filter.get_quaternion_array(frame.quat_torso);
        frame.acc_torso[0] = sample.ax;
        frame.acc_torso[1] = sample.ay;
        frame.acc_torso[2] = sample.az;
        frame.gyro_torso[0] = sample.gx;
        frame.gyro_torso[1] = sample.gy;
        frame.gyro_torso[2] = sample.gz;
        // arm字段保持为0
    }

    frame.crc16 = calculate_crc16((uint8_t*)&frame, sizeof(frame) - 2);

    // 发送UDP数据包
    int err = sendto(udp_socket, &frame, sizeof(frame), 0,
                     (struct sockaddr *)&dest_addr, sizeof(dest_addr));

    static uint32_t send_count = 0;
    static uint32_t error_count = 0;

    if (err < 0) {
        error_count++;
        if (error_count % 100 == 1) {
            ESP_LOGW(TAG, "UDP send failed: errno %d (total: %lu)",
                     errno, error_count);
        }
    } else {
        send_count++;
        if (send_count % 100 == 0) {
            ESP_LOGI(TAG, "UDP sent %lu packets (errors: %lu)",
                     send_count, error_count);
        }
    }
}
```

#### 1.5 主循环集成
```c
extern "C" void app_main(void) {
    // IMU初始化
    uint8_t device_id = 0;
    ImuSample sample;
    MadgwickFilterExtended filter(0.3f, 0.01f);  // Beta=0.3, dt=10ms

    // 自动检测IMU传感器
    if (icm42688_init() == ESP_OK) {
        device_id = 1;  // ICM42688 -> arm
        ESP_LOGI(TAG, "ICM42688 detected, device_id=1 (arm)");
    } else if (bmi160_init() == ESP_OK) {
        device_id = 2;  // BMI160 -> torso
        ESP_LOGI(TAG, "BMI160 detected, device_id=2 (torso)");
    } else {
        ESP_LOGE(TAG, "No IMU sensor detected!");
        return;
    }

    // WiFi初始化
    ESP_ERROR_CHECK(nvs_flash_init());
    wifi_init_sta();

    // 等待WiFi连接
    vTaskDelay(pdMS_TO_TICKS(5000));

    // UDP客户端初始化
    udp_client_init();

    // 主循环
    uint32_t timestamp_ms = 0;
    while (true) {
        // IMU采样
        if (device_id == 1) {
            icm42688_read_sample(&sample);
        } else {
            bmi160_read_sample(&sample);
        }

        // 姿态解算
        filter.update(sample.gx, sample.gy, sample.gz,
                     sample.ax, sample.ay, sample.az);

        // 通过UDP发送
        send_imu_data_via_udp(sample, filter, timestamp_ms, device_id);

        timestamp_ms += 10;
        vTaskDelay(pdMS_TO_TICKS(10));  // 10ms = 100Hz
    }
}
```

---

### 2. ESP32-C6端（ESP-Hosted + UDP Server）

#### 2.1 ESP-Hosted配置

C6运行ESP-Hosted固件，作为P4的WiFi网卡。

**编译ESP-Hosted固件：**
```bash
git clone --recursive https://github.com/espressif/esp-hosted.git
cd esp-hosted/esp_hosted_fg/esp/esp_driver/network_adapter

# 配置为SDIO模式
idf.py menuconfig
# -> Example Configuration
#    -> Transport layer (SDIO)
#    -> SDIO Configuration
#       -> SDIO Pin Configuration (根据P4连接配置)

# 编译并烧录到C6
idf.py build flash
```

**C6的SDIO引脚（需根据实际连接配置）：**
```
CLK:  GPIO某
CMD:  GPIO某
D0:   GPIO某
D1:   GPIO某
D2:   GPIO某
D3:   GPIO某
```

#### 2.2 SoftAP配置

在ESP-Hosted固件中配置SoftAP：
```c
// C6固件中的配置
#define SOFTAP_SSID     "ESP32_C6_AP"
#define SOFTAP_PASS     "12345678"
#define SOFTAP_CHANNEL  1
#define SOFTAP_MAX_CONN 4
```

#### 2.3 UDP Server（在C6上运行）

C6需要运行一个UDP Server接收S3的数据，然后通过ESP-Hosted转发给P4。

**方案1：C6固件内置UDP Server**
```c
// 在ESP-Hosted固件中添加UDP Server
void udp_server_task(void *pvParameters) {
    struct sockaddr_in dest_addr;
    dest_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port = htons(8888);

    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    bind(sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr));

    ESP_LOGI(TAG, "UDP Server listening on port 8888");

    while (1) {
        struct sockaddr_in source_addr;
        socklen_t socklen = sizeof(source_addr);
        uint8_t rx_buffer[128];

        int len = recvfrom(sock, rx_buffer, sizeof(rx_buffer), 0,
                          (struct sockaddr *)&source_addr, &socklen);

        if (len > 0) {
            // 数据会自动通过ESP-Hosted的SDIO转发给P4
            // P4的lwIP栈会接收到这个UDP包
            ESP_LOGI(TAG, "Received %d bytes from S3", len);
        }
    }
}
```

**方案2：P4直接接收（推荐）**
P4通过ESP-Hosted把C6当作网卡，P4上运行UDP Server直接接收S3的数据。

---

### 3. ESP32-P4端（UDP Server）

#### 3.1 ESP-Hosted Host配置

P4需要运行ESP-Hosted的Host驱动。

**添加ESP-Hosted组件：**
```bash
cd your_p4_project
# 添加ESP-Hosted作为组件
cp -r esp-hosted/esp_hosted_fg/host/linux/host_driver/esp32 components/esp_hosted
```

**P4的SDIO Host配置：**
```c
#include "esp_hosted.h"

// SDIO Host初始化
sdmmc_host_t host = SDMMC_HOST_DEFAULT();
sdmmc_slot_config_t slot_config = SDMMC_SLOT_CONFIG_DEFAULT();

// 配置SDIO引脚（根据实际连接）
slot_config.clk = GPIO_NUM_X;
slot_config.cmd = GPIO_NUM_X;
slot_config.d0 = GPIO_NUM_X;
slot_config.d1 = GPIO_NUM_X;
slot_config.d2 = GPIO_NUM_X;
slot_config.d3 = GPIO_NUM_X;

// 初始化ESP-Hosted
esp_hosted_init(&host, &slot_config);
```

#### 3.2 UDP Server接收
```c
#include "lwip/sockets.h"

#define UDP_PORT 8888

void udp_server_task(void *pvParameters) {
    struct sockaddr_in dest_addr;
    dest_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port = htons(UDP_PORT);

    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (sock < 0) {
        ESP_LOGE(TAG, "Unable to create socket");
        return;
    }

    int err = bind(sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
    if (err < 0) {
        ESP_LOGE(TAG, "Socket unable to bind");
        close(sock);
        return;
    }

    ESP_LOGI(TAG, "UDP Server listening on port %d", UDP_PORT);

    while (1) {
        struct sockaddr_in source_addr;
        socklen_t socklen = sizeof(source_addr);
        imu_data_frame_t frame;

        int len = recvfrom(sock, &frame, sizeof(frame), 0,
                          (struct sockaddr *)&source_addr, &socklen);

        if (len == sizeof(imu_data_frame_t)) {
            // CRC校验
            uint16_t calc_crc = calculate_crc16((uint8_t*)&frame,
                                                sizeof(frame) - 2);
            if (calc_crc != frame.crc16) {
                ESP_LOGW(TAG, "CRC mismatch");
                continue;
            }

            // 检查device_id并处理数据
            if (frame.device_id == 1) {
                ESP_LOGI(TAG, "Received arm data (ICM42688)");
            } else if (frame.device_id == 2) {
                ESP_LOGI(TAG, "Received torso data (BMI160)");
            }

            // 数据处理
            process_imu_data(&frame);

            static uint32_t recv_count = 0;
            if (++recv_count % 100 == 0) {
                ESP_LOGI(TAG, "Received %lu IMU packets", recv_count);
            }
        }
    }
}
```

#### 3.3 数据处理
```c
void process_imu_data(const imu_data_frame_t *frame) {
    // 根据device_id处理对应数据
    if (frame->device_id == 1) {
        // ICM42688数据在arm字段
        float quat_arm[4] = {
            frame->quat_arm[0],  // w
            frame->quat_arm[1],  // x
            frame->quat_arm[2],  // y
            frame->quat_arm[3]   // z
        };

        float arm_accel[3] = {
            frame->acc_arm[0],
            frame->acc_arm[1],
            frame->acc_arm[2]
        };

        float arm_gyro[3] = {
            frame->gyro_arm[0],
            frame->gyro_arm[1],
            frame->gyro_arm[2]
        };

        // 处理arm数据...
    } else if (frame->device_id == 2) {
        // BMI160数据在torso字段
        float quat_torso[4] = {
            frame->quat_torso[0],  // w
            frame->quat_torso[1],  // x
            frame->quat_torso[2],  // y
            frame->quat_torso[3]   // z
        };

        float torso_accel[3] = {
            frame->acc_torso[0],
            frame->acc_torso[1],
            frame->acc_torso[2]
        };

        float torso_gyro[3] = {
            frame->gyro_torso[0],
            frame->gyro_torso[1],
            frame->gyro_torso[2]
        };

        // 处理torso数据...
    }

    // 时间戳
    uint32_t timestamp = frame->timestamp_ms;
}
```

---

## 性能分析

### 带宽需求
```
数据帧大小：90字节
发送频率：100Hz
带宽需求：90 × 100 = 9 KB/s = 72 Kbps
```

### WiFi性能
```
WiFi 802.11n (2.4GHz)：
- 理论速率：72 Mbps (单天线)
- 实际吞吐：20-30 Mbps
- 余量：280倍以上
```

### 延迟分析
```
S3采样 → UDP发送 → WiFi传输 → C6接收 → SDIO传输 → P4接收
  0ms      <1ms       2-5ms      <1ms      1-2ms      <1ms
                    总延迟：5-10ms
```

### 丢包率
```
WiFi环境良好：<0.1%
WiFi环境一般：<1%
可通过重传机制进一步降低
```

---

## 优化建议

### 1. 降低延迟
```c
// S3端：设置WiFi省电模式为性能模式
esp_wifi_set_ps(WIFI_PS_NONE);

// 设置UDP socket优先级
int priority = 6;  // 高优先级
setsockopt(sock, SOL_SOCKET, SO_PRIORITY, &priority, sizeof(priority));

// 禁用Nagle算法（TCP才需要，UDP不需要）
```

### 2. 提高可靠性
```c
// 添加序列号检测丢包
struct imu_data_frame_t {
    uint8_t header[2];
    uint32_t sequence;      // 添加序列号
    uint32_t timestamp_ms;
    // ... 其他字段 ...
};

// P4端检测丢包
static uint32_t last_seq = 0;
if (frame->sequence != last_seq + 1) {
    ESP_LOGW(TAG, "Packet loss detected: expected %lu, got %lu",
             last_seq + 1, frame->sequence);
}
last_seq = frame->sequence;
```

### 3. 功耗优化
```c
// S3端：动态调整WiFi功率
esp_wifi_set_max_tx_power(40);  // 降低发射功率（范围8-84，单位0.25dBm）

// 使用WiFi省电模式（会增加延迟）
esp_wifi_set_ps(WIFI_PS_MIN_MODEM);  // 最小省电模式
```

---

## 实施步骤

### 阶段1：S3端改造（3-4小时）
1. 实现单IMU自动检测（ICM42688或BMI160）
2. 添加WiFi Station初始化（静态IP: 192.168.4.2）
3. 实现UDP Client发送（90字节数据帧）
4. 根据检测到的传感器设置device_id
5. 测试WiFi连接和数据发送

### 阶段2：C6固件烧录（1-2小时）
1. 下载ESP-Hosted源码
2. 配置SDIO引脚
3. 配置SoftAP参数
4. 编译并烧录固件

### 阶段3：P4端集成（4-5小时）
1. 集成ESP-Hosted Host驱动
2. 配置SDIO Host
3. 实现UDP Server接收（端口8888）
4. 根据device_id解析对应字段数据
5. 数据处理和应用集成

### 阶段4：系统联调（2-3小时）
1. 验证WiFi连接
2. 测试数据传输
3. 延迟和丢包率测试
4. 优化参数

### 阶段5：稳定性测试（2-3小时）
1. 长时间运行测试
2. 异常情况处理
3. 断线重连机制
4. 性能监控

---

## 故障排查

### 问题1：S3无法连接C6的SoftAP
```
检查项：
1. C6的SoftAP是否启动（查看C6日志）
2. SSID和密码是否正确
3. WiFi信道是否支持
4. S3的WiFi是否初始化成功

解决：
- 使用WiFi扫描确认AP可见
- 检查C6的SoftAP配置
- 尝试手机连接C6的AP验证
```

### 问题2：UDP数据无法到达P4
```
检查项：
1. P4的UDP Server是否启动
2. 网络路由是否正确
3. 防火墙是否阻止
4. SDIO连接是否正常

解决：
- 在C6上ping P4的IP
- 使用tcpdump抓包分析
- 检查ESP-Hosted的日志
```

### 问题3：延迟过高
```
检查项：
1. WiFi信号强度
2. WiFi省电模式设置
3. SDIO传输速率
4. P4的CPU负载

解决：
- 禁用WiFi省电模式
- 提高SDIO时钟频率
- 优化P4端的处理逻辑
```

### 问题4：丢包率高
```
检查项：
1. WiFi环境干扰
2. UDP缓冲区大小
3. P4处理速度

解决：
- 更换WiFi信道
- 增大socket接收缓冲区
- 添加重传机制
```

---

## 总结

### 方案优势
✓ **官方支持**：ESP-Hosted是Espressif官方方案
✓ **成熟稳定**：经过大量项目验证
✓ **单IMU灵活**：自动识别ICM42688或BMI160
✓ **无USB干扰**：完全避开USB问题
✓ **灵活扩展**：可以轻松添加其他网络功能
✓ **易于调试**：标准TCP/IP协议，工具丰富

### 性能指标
- **延迟**：5-10ms
- **带宽**：9 KB/s（实际可达MB/s级别）
- **丢包率**：<0.1%（良好环境）
- **采样率**：100Hz
- **数据帧**：90字节
- **功耗**：中等（可通过省电模式优化）

### 适用场景
✓ 单IMU自动识别系统
✓ 需要稳定可靠的数据传输
✓ 对延迟要求不是极致（<10ms可接受）
✓ 需要标准网络协议支持
✓ 未来可能需要扩展其他网络功能

### 数据帧说明
- **总大小**：90字节
- **device_id**：1=ICM42688(arm), 2=BMI160(torso)
- **单IMU模式**：未使用的字段用memset填0
- **ICM42688**：填充quat_arm、acc_arm、gyro_arm
- **BMI160**：填充quat_torso、acc_torso、gyro_torso

---

## 附录

### A. 完整的S3端代码框架
见后续实现

### B. C6 ESP-Hosted配置文件
见ESP-Hosted官方文档

### C. P4端UDP Server完整代码
见后续实现

### D. 性能测试脚本
见后续提供

---

**文档版本**：v1.0
**创建日期**：2025-01-XX
**作者**：哈雷酱 (Kiro AI Assistant)
**状态**：待实施
