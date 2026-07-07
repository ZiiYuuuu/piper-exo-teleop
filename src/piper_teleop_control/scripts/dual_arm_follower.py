#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import serial
import time
import math
import os
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

# ===================== 配置参数 =====================
SERIAL_PORT = "/dev/ttyACM0"
SERVO_BAUD = 1000000
READ_TIMEOUT_MS = 15

servo_ids = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16]
servo_count = len(servo_ids)
CMD_READ_POS = 0x38
READ_POS_DATA_LEN = 0x02

joint_raw_pos = [-1] * servo_count
last_valid_raw = [2048] * servo_count 

# ===================== 🛠️ 校准参数（严格对齐你之前的版本） =====================
TELEOP_DROOP = [-3.067176417350914, -3.113207078941973, -0.04296195081832196, 1.5880578248915438, 0.06751163700022023, 0.0, 0.0]
PIPER_LEFT_DROOP = [3.011939623441643, 3.062573351191808, 0.15036682786412686, -1.6463633295735522, -0.1258171416822286, 0.04142759543195332, -0.8147427101617485]
PIPER_RIGHT_DROOP = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

# 方向修正：先全给 1.0。如果后续发现某关节反向，再改 -1.0
LEFT_DIR =  [1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0]
RIGHT_DIR = [1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0]

LEFT_MAPPING = [0, 1, 3, 4, 5, 6, 7]      
RIGHT_MAPPING = [10, 11, 13, 14, 15, 8, 9] 

def md_servo_checksum(data):
    return (~sum(data)) & 0xFF

def pos_to_angle(raw_pos):
    if raw_pos < 0: return -1.0
    return (raw_pos - 2048) * 360.0 / 4095.0

def pulse_to_degree_str(pulse):
    if pulse < 0: return "离线"
    return f"{(pulse / 4095.0) * 360.0:.1f}°"

def read_single_servo(ser, sid):
    cmd = bytearray([0xFF, 0xFF, sid, 0x04, 0x02, CMD_READ_POS, READ_POS_DATA_LEN, 0x00])
    cmd[7] = md_servo_checksum(cmd[2:7])
    try:
        ser.reset_input_buffer()
        ser.write(cmd)
        ser.flush()
        start = time.time()
        while ser.in_waiting < 8:
            if (time.time() - start) * 1000 > READ_TIMEOUT_MS: return -1
        resp = ser.read(8)
        if len(resp) != 8 or resp[0] != 0xFF or resp[1] != 0xFF or resp[2] != sid: return -1
        if resp[7] != md_servo_checksum(resp[2:7]): return -1
        return (resp[6] << 8) | resp[5]
    except Exception:
        return -1

def main():
    rospy.init_node('teleop_integrated_controller', anonymous=True)
    
    # 核心修复 1：完全对齐你旧代码能动的话题
    pub_arm1 = rospy.Publisher('/left_arm/joint_states', JointState, queue_size=10)
    pub_arm2 = rospy.Publisher('/right_arm/joint_states', JointState, queue_size=10)
    
    rate = rospy.Rate(40) # 保持平滑度

    try:
        ser = serial.Serial(port=SERIAL_PORT, baudrate=SERVO_BAUD, timeout=0.01)
        rospy.loginfo(f"成功打开遥测臂核心串口 {SERIAL_PORT}")
    except Exception as e:
        rospy.logerr(f"串口打开失败: {e}")
        return

    while not rospy.is_shutdown():
        try:
            for idx, sid in enumerate(servo_ids):
                raw_val = read_single_servo(ser, sid)
                joint_raw_pos[idx] = raw_val
                if raw_val >= 0:
                    last_valid_raw[idx] = raw_val

            # ==================== 双臂解算 ====================
            left_radians = []
            for count, idx in enumerate(LEFT_MAPPING):
                deg = pos_to_angle(last_valid_raw[idx])
                raw_rad = deg * (math.pi / 180.0)
                # 融入方向算子
                left_radians.append((raw_rad - TELEOP_DROOP[count]) * LEFT_DIR[count] + PIPER_LEFT_DROOP[count])

            right_radians = []
            for count, idx in enumerate(RIGHT_MAPPING):
                deg = pos_to_angle(last_valid_raw[idx])
                raw_rad = deg * (math.pi / 180.0)
                right_radians.append((raw_rad - TELEOP_DROOP[count]) * RIGHT_DIR[count] + PIPER_RIGHT_DROOP[count])

            # ==================== 发送与组装 ====================
            stamp = rospy.Time.now()
            # 核心修复 2：名字叫 endeffector
            joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6', 'endeffector']
            
            pub_arm1.publish(JointState(header=Header(stamp=stamp), name=joint_names, position=left_radians))
            pub_arm2.publish(JointState(header=Header(stamp=stamp), name=joint_names, position=right_radians))

            # UI 监视保持
            os.system('clear')
            print("=" * 100)
            print("          【稳定不掉线版】 遥测臂主从跟随控制终端 & 实时状态监视器")
            print("=" * 100)
            print("【 PIPER 发送位置 】")
            print(f"  左臂弧度: {[f'{j:.3f}' for j in left_radians]}")
            print(f"  右臂弧度: {[f'{j:.3f}' for j in right_radians]}")
            print("-" * 100)
            print("【 遥测臂底层物理输入 】")
            print("  左手侧:")
            for i in range(8):
                p = joint_raw_pos[i]
                print(f"    ID{servo_ids[i]:<2}: {f'{p:<4}' if p>=0 else '离线'} {f'({pulse_to_degree_str(p)})':<8}", end="")
                if (i + 1) % 4 == 0: print()
            print("  右手侧:")
            for i in range(8, 16):
                p = joint_raw_pos[i]
                print(f"    ID{servo_ids[i]:<2}: {f'{p:<4}' if p>=0 else '离线'} {f'({pulse_to_degree_str(p)})':<8}", end="")
                if (i + 1) % 4 == 0: print()
            print("=" * 100)

        except Exception as main_err:
            rospy.logerr(f"核心循环异常: {main_err}")
        rate.sleep()

    ser.close()

if __name__ == "__main__":
    try: main()
    except rospy.ROSInterruptException: pass
