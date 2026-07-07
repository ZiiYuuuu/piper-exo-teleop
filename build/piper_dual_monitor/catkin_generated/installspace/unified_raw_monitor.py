#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
unified_raw_monitor.py

联合监视：遥测臂(16路舵机原始脉冲) + PIPER 双臂(真实关节弧度)

== 关键修复说明 ==
官方 piper_ros 驱动里有两个名字很像、但作用完全相反的话题：

  /joint_states         —— 【输入】控制话题。你往这个话题发布
                            sensor_msgs/JointState，机械臂才会动。
                            驱动节点本身只订阅它，从不在它上面发布反馈。
  /joint_states_single   —— 【输出】反馈话题。驱动节点把真实关节角度
                            实时发布在这里。

原脚本订阅的是 .../joint_states，根本没有人往这个话题发消息，
所以永远收不到数据 —— 这不是 CAN 没接好，也不是话题没启动，
只是订阅错了话题名。本脚本已改为订阅 .../joint_states_single。

== 用法 ==
作为 rospy 节点运行，可通过 rosparam / launch 文件配置：
  ~serial_port    (string, default: /dev/ttyACM0)
  ~servo_baud     (int,    default: 1000000)
  ~read_timeout_ms(int,    default: 30)
  ~left_topic     (string, default: /left_arm/joint_states_single)
  ~right_topic    (string, default: /right_arm/joint_states_single)
  ~refresh_rate   (float,  default: 30.0)   # Hz
"""

import os
import time

import rospy
import serial
from sensor_msgs.msg import JointState

# ===================== 舵机协议配置（与原脚本一致，未改动） =====================
servo_ids = list(range(1, 17))
servo_count = len(servo_ids)
CMD_READ_POS = 0x38
READ_POS_DATA_LEN = 0x02

joint_raw_pos = [-1] * servo_count
piper_left_pos = []
piper_right_pos = []


def piper_left_callback(msg):
    global piper_left_pos
    piper_left_pos = list(msg.position)


def piper_right_callback(msg):
    global piper_right_pos
    piper_right_pos = list(msg.position)


def md_servo_checksum(data):
    total = sum(data)
    return (~total) & 0xFF


def read_single_servo(ser, sid, timeout_ms):
    cmd = bytearray([0xFF, 0xFF, sid, 0x04, 0x02, CMD_READ_POS, READ_POS_DATA_LEN, 0x00])
    cmd[7] = md_servo_checksum(cmd[2:7])
    try:
        ser.reset_input_buffer()
        ser.write(cmd)
        ser.flush()
        start = time.time()
        while ser.in_waiting < 8:
            if (time.time() - start) * 1000 > timeout_ms:
                return -1
        resp = ser.read(8)
        if len(resp) != 8 or resp[0] != 0xFF or resp[1] != 0xFF or resp[2] != sid:
            return -1
        if resp[7] != md_servo_checksum(resp[2:7]):
            return -1
        return (resp[6] << 8) | resp[5]
    except Exception:
        return -1


def main():
    rospy.init_node('unified_raw_monitor', anonymous=False)

    serial_port = rospy.get_param('~serial_port', '/dev/ttyACM0')
    servo_baud = rospy.get_param('~servo_baud', 1000000)
    read_timeout_ms = rospy.get_param('~read_timeout_ms', 30)
    left_topic = rospy.get_param('~left_topic', '/left_arm/joint_states_single')
    right_topic = rospy.get_param('~right_topic', '/right_arm/joint_states_single')
    refresh_rate = rospy.get_param('~refresh_rate', 30.0)

    rospy.Subscriber(left_topic, JointState, piper_left_callback)
    rospy.Subscriber(right_topic, JointState, piper_right_callback)

    try:
        ser = serial.Serial(
            port=serial_port, baudrate=servo_baud,
            bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE, timeout=0.03
        )
    except Exception as e:
        rospy.logerr("无法打开串口 %s: %s", serial_port, e)
        return

    rospy.loginfo("正在启动联合监视器 (左臂反馈话题: %s, 右臂反馈话题: %s)",
                   left_topic, right_topic)

    rate = rospy.Rate(refresh_rate)
    while not rospy.is_shutdown():
        for idx, sid in enumerate(servo_ids):
            joint_raw_pos[idx] = read_single_servo(ser, sid, read_timeout_ms)

        os.system('clear')
        print("=" * 80)
        print("   遥测臂(原始脉冲) 与 PIPER 机器人(即时弧度) 联合监视器  [ROS 版]")
        print("=" * 80)

        print("【 PIPER 真实即时位置 】")
        if piper_left_pos:
            print(f"  左 Piper 弧度: {[f'{j:.3f}' for j in piper_left_pos]}")
        else:
            print(f"  左 Piper 弧度: [无数据] 正在等待话题 {left_topic} 发送...")

        if piper_right_pos:
            print(f"  右 Piper 弧度: {[f'{j:.3f}' for j in piper_right_pos]}")
        else:
            print(f"  右 Piper 弧度: [无数据] 正在等待话题 {right_topic} 发送...")

        print("-" * 80)
        print("【 遥测臂 16 路底层物理脉冲 】")
        left_strs = [f"ID{servo_ids[i]}:{joint_raw_pos[i] if joint_raw_pos[i] >= 0 else '离线'}"
                     for i in range(8)]
        right_strs = [f"ID{servo_ids[i]}:{joint_raw_pos[i] if joint_raw_pos[i] >= 0 else '离线'}"
                      for i in range(8, 16)]
        print(f"  左手侧原始: {', '.join(left_strs)}")
        print(f"  右手侧原始: {', '.join(right_strs)}")
        print("=" * 80)

        rate.sleep()

    ser.close()
    rospy.loginfo("已安全退出。")


if __name__ == "__main__":
    main()
