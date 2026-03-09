# ESP32-S3 单IMU姿态解算系统

基于ESP32-S3的单IMU姿态融合系统，支持ICM42688和BMI160自动识别，使用Madgwick算法实现高精度姿态解算，通过WiFi UDP发送数据到ESP32-P4。

## 📋 项目简介

本项目实现了ESP32-S3单IMU姿态追踪系统，支持自动检测并初始化ICM42688或BMI160传感器。通过ESP-Hosted网络方案，将姿态数据通过UDP发送到ESP32-P4进行姿态评估和CNN推理。

## 🔧 硬件配置

### 支持的传感器

**ICM-42688**（手臂IMU，device_id=1）
- 6轴加速度计 + 陀螺仪
- 量程：±4g / ±500dps
- 采样率：100Hz
- I2C地址：0x68
- 支持device ID：0x47 / 0x7B

**BMI160**（躯干IMU，device_id=2）
- 6轴加速度计 + 陀螺仪
- 量程：±4g / ±500dps
- 采样率：100Hz
- I2C地址：0x69

### 引脚连接

| 传感器 | I2C总线 | SDA | SCL | 地址 |
|--------|---------|-----|-----|------|
| ICM-42688 | I2C0 | GPIO 8 | GPIO 9 | 0x68 |
| BMI160 | I2C1 | GPIO 16 | GPIO 17 | 0x69 |

### 网络配置

- WiFi SSID：ESP32_P4_AP
- 静态IP：192.168.4.2
- 网关：192.168.4.1
- UDP目标：192.168.4.1:8888

## ✨ 功能特性

- ✅ 单IMU自动检测（ICM42688/BMI160）
- ✅ 轻量级chip id检测，避免完整初始化
- ✅ Madgwick姿态融合算法（Beta=0.3）
- ✅ 陀螺仪零偏自动校准
- ✅ 加速度计姿态初始化
- ✅ 实时欧拉角输出（Pitch/Roll/Yaw）
- ✅ 运行时错误恢复
- ✅ WiFi UDP数据传输
- ✅ 90字节数据帧格式（含device_id）
- ✅ BMI160陀螺仪FAST_STARTUP支持

## 🚀 快速开始

### 环境要求

- ESP-IDF v5.5.3
- Python 3.8+
- Git

### 克隆项目

```bash
git clone https://github.com/dangerous-nagisa/ESP32S3_IMU.git
cd ESP32S3_IMU
```

### 配置项目

```bash
idf.py set-target esp32s3
idf.py menuconfig
```

### 编译和烧录

```bash
idf.py build
idf.py -p COM_PORT flash monitor
```

## 📦 依赖组件

项目使用ESP-IDF组件管理器自动管理依赖：

- `nickchen110/icm42688` ^1.0.3 - ICM-42688驱动
- `esp-idf-lib/bmi160` ^1.0.3 - BMI160驱动
- `esp-idf-lib/i2cdev` ^1.0.3 - I2C设备管理（已修复ref_count bug）
- `espp/filters` ^1.0.35 - Madgwick滤波器
- `espressif/led_strip` ^2.5.5 - RGB LED驱动

## 📊 输出示例

```
I (1538) dual_imu: Detecting IMU on I2C0 (GPIO8/9)...
W (1538) dual_imu: No ICM42688 found on I2C0
I (1538) dual_imu: Detecting IMU on I2C1 (GPIO16/17)...
I (1748) dual_imu: Detected BMI160 on I2C1 at 0x69 (chip_id=0xD1)
I (1748) dual_imu: Initializing BMI160 system (device_id=2)
I (2058) dual_imu: BMI160 performing soft reset...
I (2368) dual_imu: BMI160 chip ID: 0xD1 (expect 0xD1)
I (2368) dual_imu: BMI160 setting accel to NORMAL mode...
I (2388) dual_imu: BMI160 ACC PMU ready after 20 ms
I (2388) dual_imu: BMI160 setting gyro to FAST_STARTUP mode...
I (2488) dual_imu: BMI160 setting gyro to NORMAL mode...
I (2608) dual_imu: BMI160 GYR PMU ready after 100 ms
I (2658) dual_imu: BMI160 init success on I2C1 SDA=16 SCL=17
I (4868) dual_imu: BMI160 gyro bias[rad/s]=[+0.00156, -0.00112, +0.00034] from 197 still samples
I (4868) dual_imu: Attitude initialization complete, entering normal output phase
I (4878) dual_imu: WiFi connected successfully
I (4978) dual_imu: torso/bmi160 acc[g]=[+0.008, -0.021, +1.002] gyro[rad/s]=[+0.000, -0.001, +0.001] euler[deg]=[pitch=-0.89 roll=0.45 yaw=0.00]
```

## 🔬 技术细节

### 姿态解算流程

1. **IMU检测阶段**：轻量级chip id检测，确定IMU类型
2. **预热阶段**（2秒）：等待传感器稳定
3. **校准阶段**（最多3次尝试）：采集静止状态下的陀螺仪零偏
4. **初始化阶段**：使用加速度计计算初始姿态四元数
5. **运行阶段**：10ms周期采样，Madgwick融合更新

### 数据帧格式（90字节）

```c
typedef struct __attribute__((packed)) {
    uint8_t header[2];          // 0xAA 0x55
    uint8_t device_id;          // 1=arm(ICM42688), 2=torso(BMI160)
    uint8_t reserved;           // Reserved byte
    uint32_t timestamp_ms;      // 时间戳毫秒
    float quat_torso[4];        // 躯干四元数 w,x,y,z
    float quat_arm[4];          // 手臂四元数 w,x,y,z
    float acc_torso[3];         // 躯干加速度 ax,ay,az (g)
    float gyro_torso[3];        // 躯干角速度 gx,gy,gz (rad/s)
    float acc_arm[3];           // 手臂加速度 ax,ay,az (g)
    float gyro_arm[3];          // 手臂角速度 gx,gy,gz (rad/s)
    uint16_t crc16;             // CRC-16/MODBUS
} imu_data_frame_t;
```

单IMU模式下，未使用的字段填0。

### Madgwick参数

- **Beta**：0.3（提高响应速度）
- **采样周期**：10ms（100Hz）

## 🛠️ 故障排查

### I2C设备未找到

1. 检查引脚连接是否正确
2. 确认传感器供电正常（3.3V）
3. 检查I2C地址是否匹配

### BMI160陀螺仪PMU超时

- 已添加FAST_STARTUP步骤解决
- 如仍失败，检查供电和焊接质量

### WiFi连接失败

1. 确认ESP32-P4的AP已启动
2. 检查SSID和密码配置
3. 验证静态IP配置

### 编译错误

```bash
# 清理构建缓存
idf.py fullclean
# 重新拉取依赖
rm -rf managed_components
idf.py reconfigure
```

## 📝 已知问题

- 无磁力计，Yaw轴存在累积漂移
- 动态加速度会影响Pitch/Roll精度
- BMI160某些批次需要FAST_STARTUP步骤

## 🔧 关键修复

- ✅ 轻量级IMU检测，避免完整初始化导致误判
- ✅ i2cdev ref_count递减bug修复
- ✅ icm42688_delete参数类型错误修复
- ✅ BMI160陀螺仪FAST_STARTUP步骤
- ✅ 支持ICM42688两个device ID (0x47/0x7B)

## 🤝 贡献

欢迎提交Issue和Pull Request！

## 📄 许可证

MIT License

## 👤 作者

dangerous-nagisa

---

⭐ 如果这个项目对你有帮助，请给个Star！
