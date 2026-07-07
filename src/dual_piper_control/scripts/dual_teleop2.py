#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import time
import rospy
import serial

from sensor_msgs.msg import JointState

try:
    from piper_msgs.srv import Enable
except Exception:
    Enable = None


# ============================================================
# Serial servo settings
# ============================================================

SERIAL_PORT = rospy.get_param("~serial_port", "/dev/ttyACM0")
BAUD = int(rospy.get_param("~baud", 1000000))
TIMEOUT = float(rospy.get_param("~timeout", 0.015))

CMD_READ_POS = 0x38
READ_LEN = 0x02


# ============================================================
# Safety settings
# ============================================================

RATE_HZ = float(rospy.get_param("~rate_hz", 25.0))

# Conservative inward margin from both ends of each Piper joint range.
# 0.03 rad ≈ 1.7 degrees.
# Set to 0.0 if you want to use the exact table endpoints.
PIPER_MARGIN_RAD = float(rospy.get_param("~piper_margin_rad", 0.03))

# Max change per loop.
# At 25 Hz, 0.025 rad/cycle ≈ 36 deg/sec.
MAX_STEP_RAD = float(rospy.get_param("~max_step_rad", 0.025))

# If true, this node will:
# 1. read current Piper feedback
# 2. publish hold pose
# 3. call enable services
# 4. begin teleop slowly
ENABLE_ON_START = bool(rospy.get_param("~enable_on_start", False))


# ============================================================
# Piper mapping table
# ============================================================

# Format:
# servo_id, servo_start_deg, servo_end_deg, piper_start_rad, piper_end_rad

LEFT_TABLE = [
    # Piper J1: 0 -> -1.50, Servo ID1: 180 -> 270
    (1, 180.0, 270.0, 0.00, -1.50),

    # Piper J2: 3.00 -> 1.50, Servo ID2: 180 -> 103
    (2, 180.0, 103.0, 3.00, 1.50),

    # Piper J3: -2.80 -> -1.30, Servo ID4: 180 -> 270
    (4, 180.0, 270.0, -2.80, -1.30),

    # Piper J4: 0 -> 1.50, Servo ID5: 180 -> 90
    (5, 180.0, 90.0, 0.00, 1.50),

    # Piper J5: 0 -> 1.30, Servo ID6: 180 -> 270
    (6, 180.0, 270.0, 0.00, 1.30),
]

RIGHT_TABLE = [
    # Piper J1: 0 -> 1.62, Servo ID11: 180 -> 90
    (11, 180.0, 90.0, 0.00, 1.62),

    # Piper J2: 3.10 -> 1.60, Servo ID12: 180 -> 270
    (12, 180.0, 270.0, 3.10, 1.60),

    # Piper J3: -2.80 -> -1.30, Servo ID14: 180 -> 90
    (14, 180.0, 90.0, -2.80, -1.30),

    # Piper J4: 0 -> -1.50, Servo ID15: 180 -> 270
    (15, 180.0, 270.0, 0.00, -1.50),

    # Piper J5 corrected: 0 -> +1.20, Servo ID16: 180 -> 90
    (16, 180.0, 90.0, 0.00, 1.20),
]

# Fixed wrist joints
LEFT_J6_FIXED = 2.70
RIGHT_J6_FIXED = -2.70

JOINT_NAMES = [
    "joint1", "joint2", "joint3",
    "joint4", "joint5", "joint6"
]


# ============================================================
# Servo helpers
# ============================================================

last_valid = {sid: 2048 for sid in range(1, 17)}


def checksum(data):
    return (~sum(data)) & 0xFF


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


def raw_to_deg(raw):
    """
    Servo raw 0..4095 -> 0..360 deg.
    raw 2048 is approximately 180 deg.
    """
    return raw * 360.0 / 4095.0


def read_servo_raw(ser, sid):
    cmd = bytearray([
        0xFF, 0xFF,
        sid,
        0x04,
        0x02,
        CMD_READ_POS,
        READ_LEN,
        0
    ])

    cmd[7] = checksum(cmd[2:7])

    ser.reset_input_buffer()
    ser.write(cmd)

    start = time.time()
    while ser.in_waiting < 8:
        if time.time() - start > TIMEOUT:
            return last_valid[sid]

    resp = ser.read(8)

    if len(resp) != 8:
        return last_valid[sid]

    if resp[0] != 0xFF or resp[1] != 0xFF:
        return last_valid[sid]

    if resp[2] != sid:
        return last_valid[sid]

    if resp[7] != checksum(resp[2:7]):
        return last_valid[sid]

    raw = (resp[6] << 8) | resp[5]

    if 0 <= raw <= 4095:
        last_valid[sid] = raw

    return last_valid[sid]


def read_servo_deg(ser, sid):
    return raw_to_deg(read_servo_raw(ser, sid))


# ============================================================
# Mapping helpers
# ============================================================

def map_one_joint(servo_deg, servo_a, servo_b, piper_a, piper_b):
    """
    Linear map:
        servo_a -> piper_a
        servo_b -> piper_b

    Works even when input or output range is reversed.
    """

    # Clamp servo input first
    s_lo = min(servo_a, servo_b)
    s_hi = max(servo_a, servo_b)
    servo_deg = clamp(servo_deg, s_lo, s_hi)

    # Linear interpolation
    t = (servo_deg - servo_a) / (servo_b - servo_a)
    piper = piper_a + t * (piper_b - piper_a)

    # Clamp Piper output to table range, with conservative margin
    p_lo = min(piper_a, piper_b)
    p_hi = max(piper_a, piper_b)

    span = p_hi - p_lo
    margin = min(max(PIPER_MARGIN_RAD, 0.0), span * 0.25)

    return clamp(piper, p_lo + margin, p_hi - margin)


def build_arm_target(ser, table, fixed_j6):
    joints = []

    for sid, servo_a, servo_b, piper_a, piper_b in table:
        deg = read_servo_deg(ser, sid)
        rad = map_one_joint(
            deg,
            servo_a,
            servo_b,
            piper_a,
            piper_b
        )
        joints.append(rad)

    joints.append(fixed_j6)
    return joints


def slew_limit(prev, target):
    if prev is None:
        return list(target)

    out = []

    for old, new in zip(prev, target):
        delta = new - old

        if abs(delta) > MAX_STEP_RAD:
            new = old + math.copysign(MAX_STEP_RAD, delta)

        out.append(new)

    return out


# ============================================================
# ROS helpers
# ============================================================

left_feedback = None
right_feedback = None


def left_feedback_cb(msg):
    global left_feedback
    if len(msg.position) >= 6:
        left_feedback = list(msg.position[:6])


def right_feedback_cb(msg):
    global right_feedback
    if len(msg.position) >= 6:
        right_feedback = list(msg.position[:6])


def publish_joint(pub, q):
    msg = JointState()
    msg.header.stamp = rospy.Time.now()
    msg.name = JOINT_NAMES
    msg.position = q

    # Important:
    # Do not send position[6], because piper_ctrl_single_node.py treats
    # position[6] as gripper command.
    pub.publish(msg)


def wait_for_feedback(timeout_sec=3.0):
    start = time.time()
    rate = rospy.Rate(50)

    while not rospy.is_shutdown():
        if left_feedback is not None and right_feedback is not None:
            return True

        if time.time() - start > timeout_sec:
            rospy.logwarn("Timed out waiting for Piper feedback.")
            return False

        rate.sleep()


def enable_arm(ns, enable=True):
    if Enable is None:
        rospy.logwarn("piper_msgs/Enable not available.")
        return

    service_name = f"/{ns}/enable_srv"

    rospy.loginfo("Waiting for %s", service_name)
    rospy.wait_for_service(service_name)

    proxy = rospy.ServiceProxy(service_name, Enable)
    resp = proxy(enable)

    rospy.loginfo("%s returned: %s", service_name, resp)


def safe_enable_sequence(left_pub, right_pub):
    """
    Hold current Piper pose first, then enable.
    This reduces the chance of jumping when motors are enabled.
    """

    wait_for_feedback(timeout_sec=3.0)

    hold_rate = rospy.Rate(20)

    for _ in range(10):
        if rospy.is_shutdown():
            return

        if left_feedback is not None:
            publish_joint(left_pub, left_feedback)

        if right_feedback is not None:
            publish_joint(right_pub, right_feedback)

        hold_rate.sleep()

    enable_arm("left_arm", True)
    enable_arm("right_arm", True)

    for _ in range(10):
        if rospy.is_shutdown():
            return

        if left_feedback is not None:
            publish_joint(left_pub, left_feedback)

        if right_feedback is not None:
            publish_joint(right_pub, right_feedback)

        hold_rate.sleep()


# ============================================================
# Main
# ============================================================

def main():
    rospy.init_node("dual_piper_servo_teleop_safe")

    left_pub = rospy.Publisher(
        "/left_arm/joint_ctrl_single",
        JointState,
        queue_size=1
    )

    right_pub = rospy.Publisher(
        "/right_arm/joint_ctrl_single",
        JointState,
        queue_size=1
    )

    rospy.Subscriber(
        "/left_arm/joint_states_single",
        JointState,
        left_feedback_cb,
        queue_size=1
    )

    rospy.Subscriber(
        "/right_arm/joint_states_single",
        JointState,
        right_feedback_cb,
        queue_size=1
    )

    ser = serial.Serial(
        SERIAL_PORT,
        BAUD,
        timeout=TIMEOUT
    )

    rospy.loginfo("Opened servo serial port: %s", SERIAL_PORT)
    rospy.loginfo("Publishing left commands to /left_arm/joint_ctrl_single")
    rospy.loginfo("Publishing right commands to /right_arm/joint_ctrl_single")

    if ENABLE_ON_START:
        safe_enable_sequence(left_pub, right_pub)

    prev_left = None
    prev_right = None

    rate = rospy.Rate(RATE_HZ)

    while not rospy.is_shutdown():

        left_target = build_arm_target(
            ser,
            LEFT_TABLE,
            LEFT_J6_FIXED
        )

        right_target = build_arm_target(
            ser,
            RIGHT_TABLE,
            RIGHT_J6_FIXED
        )

        # Initialize command from feedback if available.
        # This avoids an immediate jump at startup.
        if prev_left is None:
            if left_feedback is not None:
                prev_left = list(left_feedback)
            else:
                prev_left = list(left_target)

        if prev_right is None:
            if right_feedback is not None:
                prev_right = list(right_feedback)
            else:
                prev_right = list(right_target)

        left_cmd = slew_limit(prev_left, left_target)
        right_cmd = slew_limit(prev_right, right_target)

        prev_left = left_cmd
        prev_right = right_cmd

        publish_joint(left_pub, left_cmd)
        publish_joint(right_pub, right_cmd)

        rospy.loginfo_throttle(
            1.0,
            "Left: %s | Right: %s",
            [round(x, 3) for x in left_cmd],
            [round(x, 3) for x in right_cmd]
        )

        rate.sleep()


if __name__ == "__main__":
    main()
