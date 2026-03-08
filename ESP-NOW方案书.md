# ESP-NOW无线传输方案书

## 项目背景

### 当前问题
- ESP32-S3通过USB CDC连接ESP32-P4时，ICM42688传感器读取失败
- USB通信产生的电磁干扰影响I2C0总线
- 连接PC时正常，说明是P4的USB Host实现或供电问题

### 解决方案
采用ESP-NOW无线传输，完全避开USB干扰问题

---

## 系统架构

### 硬件拓扑

```
┌─────────────────────────────────────────────────────────────┐
│                     ESP32-S3 (Device端)                      │
│  ┌──────────┐    ┌──────────┐    ┌──────────────────────┐  │
│  │ ICM42688 │───▶│  I2C0    │    │                      │  │
│  │ (手臂)   │    │ GPIO8/9  │    │   Madgwick滤波器     │  │
│  └──────────┘    └──────────┘    │   姿态解算           │  │
│                                   │                      │  │
│  ┌──────────┐    ┌──────────┐    │                      │  │
│  │ BMI160   │───▶│  I2C1    │───▶│   数据打包           │  │
│  │ (躯干)   │    │ GPIO16/17│    │   CRC校验            │  │
│  └──────────┘    └──────────┘    │                      │  │
│                                   └──────────┬───────────┘  │
│                                              │              │
│                                   ┌──────────▼───────────┐  │
│                                   │   ESP-NOW发送        │  │
│                                   │   WiFi 2.4GHz        │  │
│                                   └──────────┬───────────┘  │
└────────────────────────────────────────────┼───────────────┘
                                              │
                                              │ 无线传输
                                              │ <10ms延迟
                                              │
┌────────────────────────────────────────────▼───────────────┐
│                     ESP32-P4 (Host端)                       │
│                                                             │
│  ┌──────────────────────────────────────────────────────┐  │
│  │              ESP32-C6 (WiFi协处理器)                  │  │
│  │  ┌──────────────────┐    ┌──────────────────────┐   │  │
│  │  │  ESP-NOW接收     │───▶│  数据解析            │   │  │
│  │  │  WiFi 2.4GHz     │    │  CRC校验             │   │  │
│  │  └──────────────────┘    └──────────┬───────────┘   │  │
│  │                                      │               │  │
│  │                           ┌──────────▼───────────┐   │  │
│  │                           │  SDIO接口传输        │   │  │
│  │                           │  到P4主控            │   │  │
│  │                           └──────────┬───────────┘   │  │
│  └────────────────────────────────────┼─────────────────┘  │
│                                        │                   │
│                             ┌──────────▼───────────┐       │
│                             │  ESP32-P4主控        │       │
│                             │  数据处理/显示       │       │
│                             └──────────────────────┘       │
└─────────────────────────────────────────────────────────────┘
```

---

## 技术方案

### 1. ESP32-S3端（发送端）

#### 1.1 数据帧格式（保持不变）
```c
struct imu_data_frame_t {
    uint8_t header[2];          // 0xAA 0x55
    uint32_t timestamp_ms;      // 时间戳
    float quat_torso[4];        // 躯干四元数 w,x,y,z
    float quat_arm[4];          // 手臂四元数 w,x,y,z
    float torso_accel[3];       // 躯干加速度 (g)
    float torso_gyro[3];        // 躯干角速度 (rad/s)
    float arm_accel[3];         // 手臂加速度 (g)
    float arm_gyro[3];          // 手臂角速度 (rad/s)
    uint16_t crc16;             // CRC-16/MODBUS
};
// 总大小：88字节
```

#### 1.2 ESP-NOW配置
```c
// WiFi初始化为Station模式
wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
esp_wifi_init(&cfg);
esp_wifi_set_mode(WIFI_MODE_STA);
esp_wifi_start();

// ESP-NOW初始化
esp_now_init();

// 添加对端MAC地址（C6的MAC）
esp_now_peer_info_t peer = {
    .peer_addr = {0xXX, 0xXX, 0xXX, 0xXX, 0xXX, 0xXX},  // C6的MAC
    .channel = 1,
    .ifidx = WIFI_IF_STA,
    .encrypt = false,  // 不加密，降低延迟
};
esp_now_add_peer(&peer);
```

#### 1.3 发送流程
```
IMU采样(50Hz) → 姿态解算 → 数据打包 → ESP-NOW发送
     ↓              ↓           ↓            ↓
  20ms周期      Madgwick    88字节帧    esp_now_send()
```

#### 1.4 关键代码
```c
// 发送函数
void send_imu_data_via_espnow(
    const ImuSample &arm_sample,
    const ImuSample &torso_sample,
    const MadgwickFilterExtended &arm_filter,
    const MadgwickFilterExtended &torso_filter,
    uint32_t timestamp_ms
) {
    imu_data_frame_t frame;
    // ... 填充数据 ...
    frame.crc16 = calculate_crc16((uint8_t*)&frame, sizeof(frame) - 2);

    esp_err_t ret = esp_now_send(peer_mac, (uint8_t*)&frame, sizeof(frame));
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "ESP-NOW send failed: %s", esp_err_to_name(ret));
    }
}
```

---

### 2. ESP32-C6端（接收端）

#### 2.1 ESP-NOW配置
```c
// WiFi初始化为Station模式
wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
esp_wifi_init(&cfg);
esp_wifi_set_mode(WIFI_MODE_STA);
esp_wifi_start();

// ESP-NOW初始化
esp_now_init();

// 注册接收回调
esp_now_register_recv_cb(espnow_recv_cb);
```

#### 2.2 接收回调
```c
void espnow_recv_cb(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
    if (len != sizeof(imu_data_frame_t)) {
        return;  // 长度不匹配
    }

    imu_data_frame_t *frame = (imu_data_frame_t*)data;

    // CRC校验
    uint16_t calc_crc = calculate_crc16(data, len - 2);
    if (calc_crc != frame->crc16) {
        ESP_LOGW(TAG, "CRC mismatch");
        return;
    }

    // 通过SDIO发送到P4
    send_to_p4_via_sdio(frame);
}
```

#### 2.3 SDIO传输到P4
```c
// C6通过SDIO Slave模式连接P4
// P4作为SDIO Host读取数据

// C6端：SDIO Slave发送
esp_err_t send_to_p4_via_sdio(const imu_data_frame_t *frame) {
    // 将数据写入SDIO缓冲区
    sdio_slave_transmit(frame, sizeof(imu_data_frame_t));
}
```

---

### 3. ESP32-P4端（主控）

#### 3.1 SDIO Host配置
```c
// P4作为SDIO Host，从C6读取数据
sdmmc_host_t host = SDMMC_HOST_DEFAULT();
sdmmc_slot_config_t slot_config = SDMMC_SLOT_CONFIG_DEFAULT();

// 初始化SDIO
sdmmc_card_t *card;
sdmmc_host_init();
sdmmc_host_init_slot(SDMMC_HOST_SLOT_1, &slot_config);
```

#### 3.2 数据接收
```c
// P4从C6读取IMU数据
void read_imu_data_from_c6() {
    imu_data_frame_t frame;

    // 从SDIO读取
    sdmmc_read_sectors(card, &frame, sector, 1);

    // CRC校验
    uint16_t calc_crc = calculate_crc16((uint8_t*)&frame, sizeof(frame) - 2);
    if (calc_crc != frame->crc16) {
        return;
    }

    // 处理数据
    process_imu_data(&frame);
}
```

---

## 性能指标

### 延迟分析
| 环节 | 延迟 | 说明 |
|------|------|------|
| S3采样 | 20ms | 50Hz采样周期 |
| 姿态解算 | <1ms | Madgwick滤波 |
| ESP-NOW传输 | 2-5ms | 无线传输 |
| C6接收处理 | <1ms | 回调处理 |
| SDIO传输 | 1-2ms | C6→P4 |
| **总延迟** | **<30ms** | 端到端 |

### 带宽需求
- 数据帧大小：88字节
- 发送频率：50Hz
- 带宽需求：88 × 50 = 4.4 KB/s
- ESP-NOW最大带宽：~250 KB/s
- **余量充足**：56倍余量

### 可靠性
- ESP-NOW丢包率：<1%（近距离）
- CRC校验：检测传输错误
- 重传机制：可选实现
- 距离范围：室内10-50米

---

## 实现步骤

### 阶段1：ESP32-S3端修改（1-2小时）
1. ✓ 移除USB CDC相关代码
2. ✓ 添加WiFi和ESP-NOW初始化
3. ✓ 修改发送函数为ESP-NOW
4. ✓ 配置对端MAC地址
5. ✓ 测试发送功能

### 阶段2：ESP32-C6端开发（2-3小时）
1. ✓ 创建C6项目
2. ✓ 实现ESP-NOW接收
3. ✓ 实现SDIO Slave
4. ✓ 数据转发逻辑
5. ✓ 测试接收和转发

### 阶段3：ESP32-P4端修改（1-2小时）
1. ✓ 实现SDIO Host
2. ✓ 从C6读取数据
3. ✓ 数据解析和处理
4. ✓ 集成到现有应用

### 阶段4：联调测试（2-3小时）
1. ✓ S3→C6无线传输测试
2. ✓ C6→P4 SDIO传输测试
3. ✓ 端到端延迟测试
4. ✓ 可靠性和丢包率测试

**总预计时间：6-10小时**

---

## 优势对比

| 特性 | USB CDC | ESP-NOW |
|------|---------|---------|
| 延迟 | <5ms | <10ms |
| 可靠性 | 高 | 中高 |
| 抗干扰 | ❌ 受USB干扰 | ✓ 无干扰 |
| 布线 | 需要USB线 | 无线 |
| 灵活性 | 受限 | 高 |
| 功耗 | 低 | 中 |
| 实现复杂度 | 简单 | 中等 |

---

## 风险与对策

### 风险1：ESP-NOW丢包
**对策**：
- 添加序列号检测丢包
- 实现重传机制
- 降低发送频率到25Hz

### 风险2：C6 SDIO性能不足
**对策**：
- 使用DMA传输
- 优化缓冲区管理
- 降低数据率

### 风险3：P4 SDIO驱动问题
**对策**：
- 参考ESP-IDF官方示例
- 使用轮询模式替代中断
- 简化协议

---

## 配置清单

### ESP32-S3 (Device)
- WiFi: Station模式
- ESP-NOW: 发送端
- 功耗: ~100mA (WiFi开启)

### ESP32-C6 (Bridge)
- WiFi: Station模式
- ESP-NOW: 接收端
- SDIO: Slave模式
- 功耗: ~80mA

### ESP32-P4 (Host)
- SDIO: Host模式
- 数据处理和显示

---

## 下一步行动

1. **立即开始**：修改S3端代码，实现ESP-NOW发送
2. **并行开发**：C6端ESP-NOW接收和SDIO转发
3. **最后集成**：P4端SDIO接收

笨蛋，准备好开始了吗？本小姐会一步步指导你完成！(￣▽￣)ノ
