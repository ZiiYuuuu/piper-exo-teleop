#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import serial
import time
import math
from sensor_msgs.msg import JointState

# ===================== 串口配置 =====================
SERIAL_PORT = "/dev/ttyACM0"
SERVO_BAUD = 1000000
READ_TIMEOUT_MS = 20

servo_ids = list(range(1, 9))  # 只用前8个有效舵机

CMD_READ_POS = 0x38
READ_POS_DATA_LEN = 0x02

# ===================== 你的已知映射 =====================
# servo → piper joint
JOINT_MAP = {
    0: 0,  # servo1 → joint1
    1: 1,  # servo2 → joint2
    3: 2,  # servo4 → joint3
    4: 3,  # servo5 → joint4
    5: 4,  # servo6 → joint5
    6: 5,  # servo7 → joint6
    7: 6   # servo8 → gripper
}

# ===================== 标定参数 =====================
SIGN = [-1, -1, -1, -1, -1, 1, 1]       # 方向修正
OFFSET = [0, 0, -3, 0, 0, 0, 0]     # 零点修正

# ===================== 数据缓存 =====================
last_valid = [2048] * 8

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

    rospy.init_node("teleop_to_piper_fixed")

    pub = rospy.Publisher("/joint_states", JointState, queue_size=10)

    rospy.loginfo("启动遥操作系统...等待Piper连接")

    time.sleep(2)

    try:
        ser = serial.Serial(SERIAL_PORT, SERVO_BAUD, timeout=0.01)
    except Exception as e:
        rospy.logerr(f"串口打开失败: {e}")
        return

    rate = rospy.Rate(30)

    while not rospy.is_shutdown():

        # ===================== 读取舵机 =====================
        raw_data = [2048] * 8

        for i in range(8):
            v = read_servo(ser, i + 1)
            if v >= 0:
                raw_data[i] = v
                last_valid[i] = v
            else:
                raw_data[i] = last_valid[i]

        # ===================== 构造Piper关节 =====================
        joints = [0.0] * 7

        for servo_idx, joint_idx in JOINT_MAP.items():

            rad = raw_to_rad(raw_data[servo_idx])

            # 标定修正
            rad = SIGN[joint_idx] * rad + OFFSET[joint_idx]

            joints[joint_idx] = rad

        # ===================== 发布 =====================
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
