#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import serial
import time
import os
import sys

try:
    import rospy
    from sensor_msgs.msg import JointState
    HAS_ROS = True
except ImportError:
    HAS_ROS = False

# ===================== 配置参数 =====================
SERIAL_PORT = "/dev/ttyACM0"
SERVO_BAUD = 1000000
READ_TIMEOUT_MS = 20

# 脉冲转角度函数 (假设 12位编码器 0~4095 对应 0~360°)
def pulse_to_degree(pulse):
    if pulse < 0:
        return "离线"
    degree = (pulse / 4095.0) * 360.0
    return f"{degree:.1f}°"

servo_ids = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16]
servo_count = len(servo_ids)
CMD_READ_POS = 0x38
READ_POS_DATA_LEN = 0x02

joint_raw_pos = [-1] * servo_count
piper_left_pos = []
piper_right_pos = []

# ROS 回调函数
def piper_left_callback(msg):
    global piper_left_pos
    if msg.position:
        piper_left_pos = list(msg.position)

def piper_right_callback(msg):
    global piper_right_pos
    if msg.position:
        piper_right_pos = list(msg.position)

def md_servo_checksum(data):
    total = sum(data)
    return (~total) & 0xFF

def read_single_servo(ser, sid):
    cmd = bytearray([0xFF, 0xFF, sid, 0x04, 0x02, CMD_READ_POS, READ_POS_DATA_LEN, 0x00])
    ck = md_servo_checksum(cmd[2:7])
    cmd[7] = ck
    try:
        ser.reset_input_buffer()
        ser.write(cmd)
        ser.flush()
        start = time.time()
        while ser.in_waiting < 8:
            if (time.time() - start) * 1000 > READ_TIMEOUT_MS:
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
    global piper_left_pos, piper_right_pos
    
    if HAS_ROS:
        try:
            rospy.init_node('unified_raw_monitor', anonymous=True)
            # 维持之前的多路全网监听，确保 Piper 弧度正常读取
            rospy.Subscriber('/joint_states', JointState, piper_left_callback) 
            rospy.Subscriber('/left_arm/joint_ctrl_single', JointState, piper_left_callback)
            rospy.Subscriber('/right_arm/joint_ctrl_single', JointState, piper_right_callback)
            rospy.Subscriber('/left_arm/joint_states', JointState, piper_left_callback)
            rospy.Subscriber('/right_arm/joint_states', JointState, piper_right_callback)
            rospy.Subscriber('/left_arm/joint_states_single', JointState, piper_left_callback)
            rospy.Subscriber('/right_arm/joint_states_single', JointState, piper_right_callback)
        except Exception:
            pass

    try:
        ser = serial.Serial(
            port=SERIAL_PORT, baudrate=SERVO_BAUD, 
            bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE, 
            stopbits=serial.STOPBITS_ONE, timeout=0.02
        )
    except Exception as e:
        print(f"❌ 无法打开串口 {SERIAL_PORT}: {e}")
        return

    while not (rospy.is_shutdown() if HAS_ROS else False):
        try:
            # 1. 串口轮询遥测臂
            for idx, sid in enumerate(servo_ids):
                joint_raw_pos[idx] = read_single_servo(ser, sid)
            
            # 2. 动态清屏刷新
            os.system('clear')
            print("=" * 100)
            print("          遥测臂(原始脉冲/角度) 与 PIPER 机器人(即时弧度) 联合监视器 (增强版)")
            print("=" * 100)
            
            print("【 PIPER 真实即时位置 】")
            if piper_left_pos:
                print(f"  左 Piper 弧度: {[f'{j:.3f}' for j in piper_left_pos]}")
            else:
                print("  左 Piper 弧度: [无数据] 正在等待话题...")
                
            if piper_right_pos:
                print(f"  右 Piper 弧度: {[f'{j:.3f}' for j in piper_right_pos]}")
            else:
                print("  右 Piper 弧度: [无数据] 正在等待话题...")
                
            print("-" * 100)
            print("【 遥测臂 16 路底层物理脉冲 与 实时角度 】")
            
            # 排版优化：将原先的长整行拆成两路，并附带角度输出
            print("  左手侧:")
            for i in range(8):
                p = joint_raw_pos[i]
                p_str = f"{p:<4}" if p >= 0 else "离线"
                deg_str = f"({pulse_to_degree(p)})" if p >= 0 else ""
                print(f"    ID{servo_ids[i]:<2}: {p_str} {deg_str:<8}", end="")
                if (i + 1) % 4 == 0: print() # 每 4 个换一行，防止太挤
                
            print("  右手侧:")
            for i in range(8, 16):
                p = joint_raw_pos[i]
                p_str = f"{p:<4}" if p >= 0 else "离线"
                deg_str = f"({pulse_to_degree(p)})" if p >= 0 else ""
                print(f"    ID{servo_ids[i]:<2}: {p_str} {deg_str:<8}", end="")
                if (i + 1) % 4 == 0: print()
                
            print("=" * 100)
            
            time.sleep(0.01)
            
        except KeyboardInterrupt:
            break
        except Exception:
            pass

    if ser and ser.is_open:
        ser.close()

if __name__ == "__main__":
    main()
