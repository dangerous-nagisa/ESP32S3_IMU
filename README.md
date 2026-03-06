# ESP32-S3 双 IMU 姿态解算系统

基于 ESP32-S3 的双 IMU 姿态融合系统，使用 Madgwick 算法实现高精度姿态解算。

## 📋 项目简介

本项目实现了在 ESP32-S3 上同时接入两颗 IMU 传感器，分别用于手臂和躯干的姿态追踪。通过独立的 I2C 总线和 Madgwick 姿态融合算法，实现实时、高精度的三维姿态解算。

## 🔧 硬件配置

### 传感器

- **ICM-42688**：手臂 IMU
  - 6 轴加速度计 + 陀螺仪
  - 量程：±4g / ±500dps
  - 采样率：100Hz
  
- **BMI160**：躯干 IMU
  - 6 轴加速度计 + 陀螺仪
  - 量程：±4g / ±500dps
  - 采样率：100Hz

### 引脚连接

| 传感器 | I2C 总线 | SDA | SCL | 地址 |
|--------|----------|-----|-----|------|
| ICM-42688 | I2C0 | GPIO 8 | GPIO 9 | 0x68 |
| BMI160 | I2C1 | GPIO 16 | GPIO 17 | 0x69 |

## ✨ 功能特性

- ✅ 双 IMU 独立 I2C 总线驱动
- ✅ Madgwick 姿态融合算法
- ✅ 陀螺仪零偏自动校准
- ✅ 加速度计姿态初始化
- ✅ 坐标系自动映射（X/Y 轴交换）
- ✅ 实时欧拉角输出（Pitch/Roll/Yaw）
- ✅ I2C 总线扫描诊断
- ✅ 运行时错误恢复

## 🚀 快速开始

### 环境要求

- ESP-IDF v5.0 或更高版本
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

项目使用 ESP-IDF 组件管理器自动管理依赖：

- `nickchen110/icm42688` ^1.0.3 - ICM-42688 驱动（已修复寄存器映射）
- `esp-idf-lib/bmi160` ^1.0.3 - BMI160 驱动
- `espp/filters` ^1.0.35 - Madgwick 滤波器

## 📊 输出示例

```
I (2000) dual_imu: Warm up IMUs for 2000 ms before calibration, keep devices still
I (4205) dual_imu: ICM42688 gyro bias[rad/s]=[+0.00123, -0.00087, +0.00045] from 198 still samples
I (5230) dual_imu: BMI160 gyro bias[rad/s]=[+0.00156, -0.00112, +0.00034] from 197 still samples
I (5230) dual_imu: Attitude initialization complete, entering normal output phase
I (5240) dual_imu: arm/icm42688 acc[g]=[+0.012, -0.034, +0.998] gyro[rad/s]=[+0.001, -0.002, +0.000] euler[deg]=[pitch=-1.23 roll=0.67 yaw=0.00]
I (5350) dual_imu: torso/bmi160 acc[g]=[+0.008, -0.021, +1.002] gyro[rad/s]=[+0.000, -0.001, +0.001] euler[deg]=[pitch=-0.89 roll=0.45 yaw=0.00]
```

## 🔬 技术细节

### 姿态解算流程

1. **预热阶段**（2秒）：等待传感器稳定
2. **校准阶段**（1秒）：采集静止状态下的陀螺仪零偏
3. **初始化阶段**：使用加速度计计算初始姿态四元数
4. **运行阶段**：10ms 周期采样，Madgwick 融合更新

### 坐标系说明

- 传感器原始坐标系经过 X/Y 轴交换映射
- 加速度计单位：g（重力加速度）
- 陀螺仪单位：rad/s（弧度/秒）
- 欧拉角单位：度（°）

### Madgwick 参数

- **Beta**：0.02（运行阶段滤波器增益）
- **采样周期**：10ms（100Hz）

## 🛠️ 故障排查

### I2C 设备未找到

1. 检查引脚连接是否正确
2. 确认传感器供电正常（3.3V）
3. 查看启动日志中的 I2C 扫描结果

### 姿态数据异常

1. 确保校准时设备完全静止
2. 检查传感器安装方向
3. 验证坐标系映射是否正确

### 编译错误

```bash
# 清理构建缓存
idf.py fullclean
# 重新拉取依赖
rm -rf managed_components
idf.py reconfigure
```

## 📝 已知问题

- 无磁力计，Yaw 轴存在累积漂移
- 动态加速度会影响 Pitch/Roll 精度
- 需要外部上拉电阻（I2C 总线）

## 🤝 贡献

欢迎提交 Issue 和 Pull Request！

## 📄 许可证

MIT License

## 👤 作者

dangerous-nagisa

---

⭐ 如果这个项目对你有帮助，请给个 Star！
