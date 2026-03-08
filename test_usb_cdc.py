#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ESP32-S3 IMU USB CDC数据接收测试脚本
"""

import serial
import struct
import sys

def verify_crc16(data):
    """验证CRC-16/MODBUS"""
    crc = 0xFFFF
    for byte in data[:-2]:
        crc ^= byte
        for _ in range(8):
            if crc & 0x0001:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    received_crc = struct.unpack('<H', data[-2:])[0]
    return crc == received_crc

def parse_frame(data):
    """解析88字节数据帧"""
    if len(data) != 88:
        return None

    # 解包数据
    header = data[0:2]
    timestamp = struct.unpack('<I', data[2:6])[0]
    quat_torso = struct.unpack('<4f', data[6:22])
    quat_arm = struct.unpack('<4f', data[22:38])
    torso_accel = struct.unpack('<3f', data[38:50])
    torso_gyro = struct.unpack('<3f', data[50:62])
    arm_accel = struct.unpack('<3f', data[62:74])
    arm_gyro = struct.unpack('<3f', data[74:86])
    crc = struct.unpack('<H', data[86:88])[0]

    return {
        'timestamp': timestamp,
        'quat_torso': quat_torso,
        'quat_arm': quat_arm,
        'torso_accel': torso_accel,
        'torso_gyro': torso_gyro,
        'arm_accel': arm_accel,
        'arm_gyro': arm_gyro,
        'crc': crc
    }

def main():
    if len(sys.argv) < 2:
        print("用法: python test_usb_cdc.py COM端口")
        print("例如: python test_usb_cdc.py COM3")
        sys.exit(1)

    port = sys.argv[1]

    print(f"正在打开 {port}...")
    try:
        # 注意：波特率对USB CDC无意义，但serial库需要指定
        ser = serial.Serial(port, 115200, timeout=1)
    except Exception as e:
        print(f"错误：无法打开串口 - {e}")
        sys.exit(1)

    print(f"已连接到 {port}")
    print("等待数据帧...")

    frame_count = 0
    error_count = 0

    while True:
        try:
            # 查找帧头 0xAA 0x55
            byte1 = ser.read(1)
            if not byte1:
                continue

            if byte1[0] == 0xAA:
                byte2 = ser.read(1)
                if byte2 and byte2[0] == 0x55:
                    # 读取剩余86字节
                    remaining = ser.read(86)
                    if len(remaining) == 86:
                        data = byte1 + byte2 + remaining

                        # 验证CRC
                        if verify_crc16(data):
                            frame_count += 1
                            frame = parse_frame(data)

                            if frame_count % 10 == 0:  # 每10帧输出一次
                                print(f"\n帧 #{frame_count}:")
                                print(f"  时间戳: {frame['timestamp']} ms")
                                print(f"  躯干四元数: w={frame['quat_torso'][0]:.3f} x={frame['quat_torso'][1]:.3f} y={frame['quat_torso'][2]:.3f} z={frame['quat_torso'][3]:.3f}")
                                print(f"  手臂四元数: w={frame['quat_arm'][0]:.3f} x={frame['quat_arm'][1]:.3f} y={frame['quat_arm'][2]:.3f} z={frame['quat_arm'][3]:.3f}")
                                print(f"  躯干加速度: {frame['torso_accel']}")
                                print(f"  错误率: {error_count}/{frame_count} ({100*error_count/(frame_count+error_count):.2f}%)")
                        else:
                            error_count += 1
                            print(f"CRC错误! (总错误: {error_count})")

        except KeyboardInterrupt:
            print(f"\n\n统计:")
            print(f"  成功接收: {frame_count} 帧")
            print(f"  CRC错误: {error_count} 帧")
            print(f"  错误率: {100*error_count/(frame_count+error_count) if (frame_count+error_count) > 0 else 0:.2f}%")
            break
        except Exception as e:
            print(f"错误: {e}")
            break

    ser.close()

if __name__ == '__main__':
    main()
