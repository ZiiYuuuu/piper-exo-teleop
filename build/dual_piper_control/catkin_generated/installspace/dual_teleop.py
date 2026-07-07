#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import serial
import time
import math
from sensor_msgs.msg import JointState

# ===================== 串口 =====================
SERIAL_PORT = "/dev/ttyACM0"
BAUD = 1000000
TIMEOUT = 0.01

# ===================== 舵机 =====================
servo_ids = list(range(1, 17))
last_valid = [2048] * 16

CMD_READ_POS = 0x38
READ_LEN = 0x02

# ===================== ⭐ 关键：零点（现在你要改的就是它） =====================
# 遥操臂“自然下垂”为 0 的补偿（核心修正）
MASTER_ZERO = [2048] * 16   # 后面我们再标定，这里先全0等价

# Piper 左臂零点（来自你旧代码）
PIPER_LEFT_ZERO = [0, 2.5, -3, 0, 0, 0, 0]

# Piper 右臂零点（先假设对称）
PIPER_RIGHT_ZERO = [0, 0, -3, 0, 0, 0, 0]

# ===================== 映射 =====================
LEFT_MAP  = [0, 1, 3, 4, 5, 6, 7]
RIGHT_MAP = [10, 11, 13, 14, 15, 8, 9]

LEFT_DIR  = [-1, -1, 1, 1, 1, 1, 1]
RIGHT_DIR = [1, 1, 1, 1, 1, 1, 1]

# ===================== 工具 =====================
def checksum(data):
    return (~sum(data)) & 0xFF


def read_servo(ser, sid):
    cmd = bytearray([0xFF, 0xFF, sid, 0x04, 0x02, CMD_READ_POS, READ_LEN, 0])
    cmd[7] = checksum(cmd[2:7])

    ser.reset_input_buffer()
    ser.write(cmd)

    start = time.time()
    while ser.in_waiting < 8:
        if time.time() - start > TIMEOUT:
            return -1

    resp = ser.read(8)
    if len(resp) != 8:
        return -1

    if resp[2] != sid:
        return -1

    if resp[7] != checksum(resp[2:7]):
        return -1

    return (resp[6] << 8) | resp[5]


def raw_to_rad(raw):
    return (raw - 2048) * 2 * math.pi / 4095


# ===================== 主程序 =====================
def main():

    rospy.init_node("dual_teleop")

    left_pub = rospy.Publisher("/left_arm/joint_states", JointState, queue_size=10)
    right_pub = rospy.Publisher("/right_arm/joint_states", JointState, queue_size=10)

    ser = serial.Serial(SERIAL_PORT, BAUD, timeout=0.01)

    rate = rospy.Rate(30)

    while not rospy.is_shutdown():

        raw = [2048] * 16

        # ================= 读取16舵机 =================
        for i in range(16):
            v = read_servo(ser, i + 1)
            if v > 0:
                raw[i] = v
                last_valid[i] = v
            else:
                raw[i] = last_valid[i]

        # ================= 左臂 =================
        left = []
        for i, idx in enumerate(LEFT_MAP):

            rad = raw_to_rad(raw[idx])

            # ⭐ 核心：减去遥操零点
            rad -= raw_to_rad(MASTER_ZERO[idx])

            # ⭐ Piper零点补偿
            rad = rad * LEFT_DIR[i] + PIPER_LEFT_ZERO[i]

            left.append(rad)

        # ================= 右臂 =================
        right = []
        for i, idx in enumerate(RIGHT_MAP):

            rad = raw_to_rad(raw[idx])
            rad -= raw_to_rad(MASTER_ZERO[idx])

            rad = rad * RIGHT_DIR[i] + PIPER_RIGHT_ZERO[i]

            right.append(rad)

        # ================= 发布 =================
        msg_l = JointState()
        msg_r = JointState()

        msg_l.header.stamp = rospy.Time.now()
        msg_r.header.stamp = rospy.Time.now()

        names = [
            "joint1","joint2","joint3",
            "joint4","joint5","joint6","endeffector"
        ]

        msg_l.name = names
        msg_r.name = names

        msg_l.position = left
        msg_r.position = right

        left_pub.publish(msg_l)
        right_pub.publish(msg_r)

        rate.sleep()


if __name__ == "__main__":
    main()
