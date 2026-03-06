# ESP32-S3 双 IMU 库使用教程（ICM42688 + BMI160）

本教程基于当前工程实际实现整理，目标是让你直接在本项目里同时驱动两个 IMU：
- `nickchen110/icm42688`（ICM42688）
- `esp-idf-lib/bmi160`（BMI160）

对应参考代码：`main/main.cpp`

## 1. 工程依赖与总体结构

`main/idf_component.yml` 已声明依赖：
- `nickchen110/icm42688: ^1.0.3`
- `esp-idf-lib/bmi160: ^1.0.3`
- `espp/filters: ^1.0.35`（Madgwick 姿态融合）

当前工程采用“双总线双 IMU”结构：
- ICM42688：`I2C_NUM_0`，SDA=`GPIO8`，SCL=`GPIO9`
- BMI160：`I2C_NUM_1`，SDA=`GPIO16`，SCL=`GPIO17`

采样周期为 `10ms`（100Hz），两颗 IMU 读数后分别喂给各自的 Madgwick 滤波器。

## 2. 两个库的实现差异（先理解再用）

### 2.1 ICM42688 库（nickchen110/icm42688）

关键点（来自 `managed_components/nickchen110__icm42688/icm42688.c`）：
- `icm42688_create()` 内部会：
  1) 通过 `i2c_master_bus_add_device` 绑定设备
  2) 读取 `WHO_AM_I` 校验芯片 ID（期望 `0x6A`）
  3) 执行软复位 `icm42688_soft_reset()`
- `icm42688_config()` 写入加速度/陀螺仪量程与 ODR，并配置中断寄存器。
- `icm42688_set_power_mode()` 上电流程是先写 `0x0E` 再写目标值，并带延时。
- `icm42688_get_acce_raw_value()` / `icm42688_get_gyro_raw_value()` 返回原始 `int16`。
- `icm42688_get_acce_value()` / `icm42688_get_gyro_value()` 内部按灵敏度换算后返回工程值：
  - 加速度：单位 `g`
  - 角速度：单位 `dps`
- 温度换算：`T = raw / 512 + 23`。

### 2.2 BMI160 库（esp-idf-lib/bmi160）

关键点（来自 `managed_components/esp-idf-lib__bmi160/bmi160.c`）：
- 该库依赖 `i2cdev`，使用前要先 `i2cdev_init()`。
- `bmi160_init()` 只做设备描述符和互斥体初始化，不等于“完整启动”。
- 完整启动有两种方式：
  - 方式A（本工程正在用）：手动调用 `bmi160_set_*` + `bmi160_switch_mode()`。
  - 方式B：调用 `bmi160_start(dev, conf)` 让库按配置统一启动。
- `bmi160_read_data()` 返回工程值：
  - `accX/Y/Z`：单位 `g`
  - `gyroX/Y/Z`：单位 `dps`

## 3. 接线与地址

### 3.1 ICM42688
- 地址：`ICM42688_I2C_ADDRESS`(0x68) 或 `ICM42688_I2C_ADDRESS_1`(0x69)
- 本工程使用：`0x68`

### 3.2 BMI160
- 地址：`BMI160_I2C_ADDRESS_GND`(0x68) 或 `BMI160_I2C_ADDRESS_VDD`(0x69)
- 本工程使用：`0x68`

说明：两颗传感器都可以是 `0x68`，因为它们挂在不同 I2C 端口，不冲突。

## 4. ICM42688 使用步骤（按本工程）

### 4.1 创建 I2C master bus
示例流程（`main/main.cpp::init_icm_bus`）：
1. 配置 `i2c_master_bus_config_t`（端口、SDA/SCL、上拉、时钟源）
2. 调用 `i2c_new_master_bus()`

### 4.2 创建设备并配置
调用顺序（`main/main.cpp::init_icm42688`）：
1. `icm42688_create(bus, ICM42688_I2C_ADDRESS, &sensor)`
2. `icm42688_config(sensor, &config)`
3. `icm42688_set_power_mode(sensor, ACCE_PWR_ON, GYRO_PWR_ON, TEMP_PWR_ON)`
4. `icm42688_get_acce_sensitivity()` / `icm42688_get_gyro_sensitivity()`

推荐与工程一致的配置：
- 加速度：`ACCE_FS_4G`, `ACCE_ODR_100HZ`
- 陀螺仪：`GYRO_FS_500DPS`, `GYRO_ODR_100HZ`

### 4.3 读取与单位换算
本工程使用原始值读取后自行换算（`read_icm42688_sample`）：
- `acc_g = raw_acc / accel_sensitivity`
- `gyro_dps = raw_gyro / gyro_sensitivity`
- `gyro_rad_s = gyro_dps * PI / 180`

可直接调用库的 `icm42688_get_acce_value` / `icm42688_get_gyro_value`，但如果你要严格控制转换链路（如统一滤波输入单位），建议像工程里一样用 raw + sensitivity。

## 5. BMI160 使用步骤（按本工程）

### 5.1 初始化 i2cdev
在首次使用 BMI160 前调用：
1. `i2cdev_init()`
2. `memset(&bmi, 0, sizeof(bmi))`
3. `bmi160_init(&bmi, BMI160_I2C_ADDRESS_GND, I2C_NUM_1, GPIO16, GPIO17)`

### 5.2 配置量程、ODR、工作模式
本工程调用顺序（`main/main.cpp::init_bmi160`）：
1. `bmi160_set_acc_range(..., BMI160_ACC_RANGE_4G)`
2. `bmi160_set_gyr_range(..., BMI160_GYR_RANGE_500DPS)`
3. `bmi160_set_acc_conf(..., BMI160_ACC_ODR_100HZ, BMI160_ACC_LP_AVG_2, BMI160_ACC_US_OFF)`
4. `bmi160_set_gyr_odr(..., BMI160_GYR_ODR_100HZ)`
5. `bmi160_switch_mode(..., BMI160_PMU_ACC_NORMAL, BMI160_PMU_GYR_NORMAL)`

### 5.3 读取数据
调用 `bmi160_read_data(&bmi, &result)` 后：
- `result.accX/Y/Z` 为 `g`
- `result.gyroX/Y/Z` 为 `dps`

本工程后续将角速度转换为 `rad/s` 再送融合算法。

## 6. 两库统一到姿态融合（本工程做法）

当前融合用的是 `espp::MadgwickFilter`，输入要求：
- 加速度：`g`
- 角速度：`rad/s`
- 时间步长：`dt_s`

代码路径（`main/main.cpp`）：
1. 计算 `dt_s = (now_us - last_tick_us) / 1e6`
2. 分别读取 ICM 与 BMI 的 `ImuSample`
3. `filter.update(dt_s, ax, ay, az, gx, gy, gz)`
4. `get_euler(pitch, roll, yaw)` 输出欧拉角

如果你后续替换滤波算法，建议保留这套统一单位接口，避免不同 IMU 驱动输出单位不一致导致姿态漂移。

## 7. 常见问题与排查

### 7.1 ICM42688 初始化失败（设备 ID 错误）
- 检查地址是否与硬件 SDO 拉电平一致（`0x68/0x69`）
- 检查 I2C 端口和引脚是否与实际接线一致
- 确认供电和地线

### 7.2 BMI160 有读数但噪声大/漂移大
- 确认已设置量程和 ODR 后再切正常模式
- 上电静止时可调用 `bmi160_calibrate()` 估计偏置
- 确认融合输入角速度单位是 `rad/s` 而不是 `dps`

### 7.3 两颗传感器地址都为 0x68 是否冲突
- 不冲突，只要它们在不同 I2C 控制器（如本工程 `I2C_NUM_0` 与 `I2C_NUM_1`）

## 8. 推荐最小调用模板

### ICM42688 最小模板
1. `i2c_new_master_bus`
2. `icm42688_create`
3. `icm42688_config`
4. `icm42688_set_power_mode`
5. 循环：`icm42688_get_*_raw_value` + sensitivity 换算

### BMI160 最小模板
1. `i2cdev_init`
2. `bmi160_init`
3. `bmi160_set_acc_range`
4. `bmi160_set_gyr_range`
5. `bmi160_set_acc_conf`
6. `bmi160_set_gyr_odr`
7. `bmi160_switch_mode`
8. 循环：`bmi160_read_data`

---

如需我继续，我可以再给你补一版“单总线挂双 IMU（地址错开）”的接线与代码模板，便于后续节省 I2C 端口。
