#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import serial
import time
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import math

# ===================== 配置参数 =====================
SERIAL_PORT = "/dev/ttyACM0"
SERVO_BAUD = 1000000
READ_TIMEOUT_MS = 50

# 保持 16 路舵机顺序排列
servo_ids = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16]
servo_count = len(servo_ids)

CMD_READ_POS = 0x38
READ_POS_DATA_LEN = 0x02

joint_raw_pos = [0] * servo_count

# ===================== 终极校准参数 =====================
# 1. 遥测臂处于自然垂落姿态时的物理弧度读取值
TELEOP_DROOP = [
    -3.067176417350914, 
    -3.113207078941973, 
    -0.04296195081832196, 
    1.5880578248915438, 
    0.06751163700022023, 
    0.0, 
    0.0
]

# 2. Piper 左机械臂处于自然垂落姿态时的真实目标弧度值
PIPER_LEFT_DROOP = [
    3.011939623441643, 
    3.062573351191808, 
    0.15036682786412686, 
    -1.6463633295735522, 
    -0.1258171416822286, 
    0.04142759543195332, 
    -0.8147427101617485
]

# 3. Piper 右机械臂处于自然垂落姿态时的真实目标弧度值
PIPER_RIGHT_DROOP = [
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
]

def md_servo_checksum(data):
    total = sum(data)
    return (~total) & 0xFF

def pos_to_angle(raw_pos):
    if raw_pos < 0:
        return -1.0
    return (raw_pos - 2048) * 360.0 / 4095.0

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
    except Exception as e:
        rospy.logwarn_throttle(2.0, f"串口读取异常 (ID {sid}): {e}")
        return -1

def read_all_servos(ser):
    for idx, sid in enumerate(servo_ids):
        raw_val = read_single_servo(ser, sid)
        if raw_val >= 0:
            joint_raw_pos[idx] = raw_val
        else:
            joint_raw_pos[idx] = -1
    return joint_raw_pos

def main():
    rospy.init_node('teleop_joint_publisher', anonymous=True)
    
    pub_arm1 = rospy.Publisher('/left_arm/joint_states', JointState, queue_size=10)
    pub_arm2 = rospy.Publisher('/right_arm/joint_states', JointState, queue_size=10)
    
    rate = rospy.Rate(100)

    ser = None
    while not rospy.is_shutdown() and ser is None:
        try:
            ser = serial.Serial(port=SERIAL_PORT, baudrate=SERVO_BAUD, bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE, timeout=0.05)
            rospy.loginfo(f"成功打开遥测臂串口 {SERIAL_PORT}")
        except Exception as e:
            rospy.logerr_throttle(5.0, f"串口打开失败: {e}")
            rospy.sleep(1.0)

    while not rospy.is_shutdown():
        try:
            raw_list = read_all_servos(ser)
            
            # ==================== 1. 解析并解算左臂 ====================
            left_radians = []
            left_mapping_indices = [0, 1, 3, 4, 5, 6, 7]
            for count, idx in enumerate(left_mapping_indices):
                deg = pos_to_angle(raw_list[idx])
                raw_rad = deg * (math.pi / 180.0) if raw_list[idx] >= 0 else 0.0
                
                # 算法公式：遥测臂相对运动增量 + Piper机械臂的自然垂落初始值
                calibrated_rad = (raw_rad - TELEOP_DROOP[count]) + PIPER_LEFT_DROOP[count]
                left_radians.append(calibrated_rad)
                
            # ==================== 2. 解析并解算右臂 ====================
            right_radians = []
            right_mapping_indices = [10, 11, 13, 14, 15, 8, 9]
            for count, idx in enumerate(right_mapping_indices):
                deg = pos_to_angle(raw_list[idx])
                raw_rad = deg * (math.pi / 180.0) if raw_list[idx] >= 0 else 0.0
                
                # 算法公式：遥测臂相对运动增量 + Piper右臂的初始值(当前全为0)
                calibrated_rad = (raw_rad - TELEOP_DROOP[count]) + PIPER_RIGHT_DROOP[count]
                right_radians.append(calibrated_rad)
                
            # ==================== 3. 组装左臂消息 ====================
            left_msg = JointState()
            left_msg.header = Header()
            left_msg.header.stamp = rospy.Time.now()
            left_msg.name = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6', 'endeffector']
            left_msg.position = left_radians
            pub_arm1.publish(left_msg)
            
            # ==================== 4. 组装右臂消息 ====================
            right_msg = JointState()
            right_msg.header = Header()
            right_msg.header.stamp = rospy.Time.now()
            right_msg.name = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6', 'endeffector']
            right_msg.position = right_radians
            pub_arm2.publish(right_msg)
            
        except Exception as main_err:
            rospy.logerr(f"主循环异常: {main_err}")
        rate.sleep()

    if ser and ser.is_open:
        ser.close()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
