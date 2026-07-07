#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import serial
import time
import math
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool

# ===================== 串口配置 =====================
SERIAL_PORT = "/dev/ttyACM0"
SERVO_BAUD = 1000000
READ_TIMEOUT_MS = 20

servo_ids = list(range(1, 17))

CMD_READ_POS = 0x38
READ_POS_DATA_LEN = 0x02

joint_raw = [-1] * 16
last_valid = [2048] * 16

# ===================== 简单映射（先跑通） =====================
# 你可以后面再改精细mapping
MAP = [0, 1, 2, 3, 4, 5]

# ===================== 工具函数 =====================
def checksum(data):
    return (~sum(data)) & 0xFF


def raw_to_rad(raw):
    if raw < 0:
        return 0.0
    deg = (raw - 2048) * 360.0 / 4095.0
    return deg * math.pi / 180.0


def read_servo(ser, sid):
    cmd = bytearray([0xFF, 0xFF, sid, 0x04, 0x02, CMD_READ_POS, READ_POS_DATA_LEN, 0x00])
    cmd[7] = checksum(cmd[2:7])

    ser.reset_input_buffer()
    ser.write(cmd)
    ser.flush()

    start = time.time()
    while ser.in_waiting < 8:
        if (time.time() - start) * 1000 > READ_TIMEOUT_MS:
            return -1

    resp = ser.read(8)
    if len(resp) != 8:
        return -1
    if resp[0] != 0xFF or resp[1] != 0xFF:
        return -1
    if resp[2] != sid:
        return -1
    if resp[7] != checksum(resp[2:7]):
        return -1

    return (resp[6] << 8) | resp[5]


# ===================== 主程序 =====================
def main():

    rospy.init_node("teleop_to_piper")

    pub = rospy.Publisher("/joint_states", JointState, queue_size=10)

    rospy.loginfo("等待Piper启动...")
    time.sleep(2)

    try:
        ser = serial.Serial(SERIAL_PORT, SERVO_BAUD, timeout=0.01)
    except Exception as e:
        rospy.logerr(f"串口失败: {e}")
        return

    rate = rospy.Rate(30)

    while not rospy.is_shutdown():

        # ===== 读取16路遥测 =====
        for i, sid in enumerate(servo_ids):
            v = read_servo(ser, sid)
            joint_raw[i] = v
            if v >= 0:
                last_valid[i] = v

        # ===== 转换为Piper 6关节 =====
        joints = []

        for i in MAP:
            rad = raw_to_rad(last_valid[i])
            joints.append(rad)

        # 补 gripper
        joints.append(0.0)

        # ===== 发布 =====
        msg = JointState()
        msg.header.stamp = rospy.Time.now()

        msg.name = [
            "joint1",
            "joint2",
            "joint3",
            "joint4",
            "joint5",
            "joint6",
            "gripper"
        ]

        msg.position = joints

        pub.publish(msg)

        rate.sleep()


if __name__ == "__main__":
    main()
