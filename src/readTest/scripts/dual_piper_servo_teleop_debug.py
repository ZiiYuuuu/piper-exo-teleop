#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import time
import rospy
import serial
from serial.serialutil import SerialException

from sensor_msgs.msg import JointState
from std_msgs.msg import Bool

try:
    from piper_msgs.srv import Enable
except Exception:
    Enable = None


CMD_READ_POS = 0x38
READ_LEN = 0x02

JOINT_NAMES = [
    "joint1", "joint2", "joint3",
    "joint4", "joint5", "joint6"
]


# ============================================================
# Mapping table
# servo_id, servo_start_deg, servo_end_deg, piper_start_rad, piper_end_rad
# ============================================================

LEFT_TABLE = [
    (1, 180.0, 270.0, 0.00, -1.50),
    (2, 180.0, 103.0, 3.00, 1.50),
    (4, 180.0, 270.0, -2.80, -1.30),
    (5, 180.0, 90.0, 0.00, 1.50),
    (6, 180.0, 270.0, 0.00, 1.30),
]

RIGHT_TABLE = [
    (11, 180.0, 90.0, 0.00, 1.62),
    (12, 180.0, 270.0, 3.10, 1.60),
    (14, 180.0, 90.0, -2.80, -1.30),
    (15, 180.0, 270.0, 0.00, -1.50),
    (16, 180.0, 90.0, 0.00, 1.20),
]

LEFT_J6_FIXED = 2.70
RIGHT_J6_FIXED = -2.70


# ============================================================
# Feedback globals
# ============================================================

left_feedback = None
right_feedback = None
left_feedback_time = None
right_feedback_time = None


def left_feedback_cb(msg):
    global left_feedback, left_feedback_time
    if len(msg.position) >= 6:
        left_feedback = list(msg.position[:6])
        left_feedback_time = time.time()


def right_feedback_cb(msg):
    global right_feedback, right_feedback_time
    if len(msg.position) >= 6:
        right_feedback = list(msg.position[:6])
        right_feedback_time = time.time()


# ============================================================
# Utility
# ============================================================

def get_bool_param(name, default=False):
    value = rospy.get_param(name, default)

    if isinstance(value, bool):
        return value

    if isinstance(value, str):
        return value.strip().lower() in ["true", "1", "yes", "y", "on"]

    return bool(value)


def checksum(data):
    return (~sum(data)) & 0xFF


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


def raw_to_deg(raw):
    return raw * 360.0 / 4095.0


def fmt(vals, ndigits=3):
    if vals is None:
        return "None"
    return "[" + ", ".join([str(round(v, ndigits)) for v in vals]) + "]"


def fmt_dict(d, ndigits=1):
    parts = []

    for k in sorted(d.keys()):
        v = d[k]

        if isinstance(v, float):
            parts.append("{}:{:.{}f}".format(k, v, ndigits))
        else:
            parts.append("{}:{}".format(k, v))

    return "{" + ", ".join(parts) + "}"


def map_one_joint(servo_deg, servo_a, servo_b, piper_a, piper_b, margin_rad):
    servo_deg = clamp(
        servo_deg,
        min(servo_a, servo_b),
        max(servo_a, servo_b)
    )

    t = (servo_deg - servo_a) / (servo_b - servo_a)
    piper = piper_a + t * (piper_b - piper_a)

    p_lo = min(piper_a, piper_b)
    p_hi = max(piper_a, piper_b)

    span = p_hi - p_lo
    margin = min(max(margin_rad, 0.0), span * 0.25)

    return clamp(piper, p_lo + margin, p_hi - margin)


def slew_limit(prev, target, max_step_rad):
    if prev is None:
        return list(target)

    out = []

    for old, new in zip(prev, target):
        delta = new - old

        if abs(delta) > max_step_rad:
            new = old + math.copysign(max_step_rad, delta)

        out.append(new)

    return out


# ============================================================
# Servo reader
# ============================================================

class ServoBusReader:
    def __init__(self, port, baud, timeout):
        self.port = port
        self.baud = baud
        self.timeout = timeout
        self.ser = None
        self.last_valid = {sid: 2048 for sid in range(1, 17)}
        self.open()

    def open(self):
        self.close()

        try:
            self.ser = serial.Serial(
                self.port,
                self.baud,
                timeout=self.timeout,
                write_timeout=self.timeout
            )

            time.sleep(0.05)
            rospy.loginfo("Opened servo serial port: %s", self.port)
            return True

        except (SerialException, OSError) as e:
            self.ser = None
            rospy.logerr_throttle(
                2.0,
                "Cannot open servo serial port %s: %s",
                self.port,
                str(e)
            )
            return False

    def close(self):
        try:
            if self.ser is not None and self.ser.is_open:
                self.ser.close()
        except Exception:
            pass

        self.ser = None

    def reconnect(self):
        self.close()
        time.sleep(0.2)
        return self.open()

    def read_raw(self, sid):
        if self.ser is None or not self.ser.is_open:
            self.reconnect()
            return self.last_valid[sid]

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

        try:
            self.ser.reset_input_buffer()
            self.ser.write(cmd)
            self.ser.flush()

            resp = self.ser.read(8)

            if len(resp) != 8:
                return self.last_valid[sid]

            if resp[0] != 0xFF or resp[1] != 0xFF:
                return self.last_valid[sid]

            if resp[2] != sid:
                return self.last_valid[sid]

            if resp[7] != checksum(resp[2:7]):
                return self.last_valid[sid]

            raw = (resp[6] << 8) | resp[5]

            if 0 <= raw <= 4095:
                self.last_valid[sid] = raw

            return self.last_valid[sid]

        except (SerialException, OSError) as e:
            rospy.logwarn_throttle(
                1.0,
                "Servo serial read failed on ID%d: %s. "
                "Using last valid value. Check duplicate access to %s.",
                sid,
                str(e),
                self.port
            )

            self.reconnect()
            return self.last_valid[sid]

    def read_deg(self, sid):
        raw = self.read_raw(sid)
        deg = raw_to_deg(raw)
        return raw, deg


# ============================================================
# Mapping and publishing
# ============================================================

def build_arm_target(servo_reader, table, fixed_j6, margin_rad):
    joints = []
    raw_by_id = {}
    deg_by_id = {}

    for sid, servo_a, servo_b, piper_a, piper_b in table:
        raw, deg = servo_reader.read_deg(sid)

        raw_by_id[sid] = raw
        deg_by_id[sid] = deg

        rad = map_one_joint(
            deg,
            servo_a,
            servo_b,
            piper_a,
            piper_b,
            margin_rad
        )

        joints.append(rad)

    joints.append(fixed_j6)

    return joints, raw_by_id, deg_by_id


def publish_joint(pub, q):
    msg = JointState()
    msg.header.stamp = rospy.Time.now()
    msg.name = JOINT_NAMES
    msg.position = list(q)

    # Do not send position[6].
    # In piper_ctrl_single_node.py, position[6] is interpreted as gripper.
    pub.publish(msg)


# ============================================================
# Enable helpers
# ============================================================

def wait_for_feedback(timeout_sec=3.0):
    start = time.time()
    rate = rospy.Rate(50)

    while not rospy.is_shutdown():
        if left_feedback is not None and right_feedback is not None:
            return True

        if time.time() - start > timeout_sec:
            rospy.logwarn(
                "Timed out waiting for Piper feedback. "
                "Check /left_arm/joint_states_single and /right_arm/joint_states_single."
            )
            return False

        rate.sleep()


def enable_arm_service(ns, enable=True):
    if Enable is None:
        rospy.logwarn(
            "Cannot import piper_msgs.srv.Enable. "
            "Service enable will be skipped."
        )
        return False

    service_name = "/{}/enable_srv".format(ns)

    try:
        rospy.loginfo("Waiting for %s", service_name)
        rospy.wait_for_service(service_name, timeout=6.0)

        proxy = rospy.ServiceProxy(service_name, Enable)
        resp = proxy(enable)

        rospy.loginfo("%s response: %s", service_name, str(resp))
        return True

    except Exception as e:
        rospy.logerr("Failed to call %s: %s", service_name, str(e))
        return False


def publish_enable_flag(left_enable_pub, right_enable_pub, enable=True, seconds=1.5):
    msg = Bool()
    msg.data = enable

    rate = rospy.Rate(10)
    end_time = time.time() + seconds

    while not rospy.is_shutdown() and time.time() < end_time:
        left_enable_pub.publish(msg)
        right_enable_pub.publish(msg)
        rate.sleep()


def safe_enable_sequence(left_pub, right_pub, left_enable_pub, right_enable_pub):
    rospy.loginfo("Starting safe enable sequence.")

    got_feedback = wait_for_feedback(timeout_sec=3.0)
    hold_rate = rospy.Rate(20)

    if got_feedback:
        rospy.loginfo("Holding current Piper feedback before enabling.")

        for _ in range(10):
            if rospy.is_shutdown():
                return

            if left_feedback is not None:
                publish_joint(left_pub, left_feedback)

            if right_feedback is not None:
                publish_joint(right_pub, right_feedback)

            hold_rate.sleep()
    else:
        rospy.logwarn(
            "No Piper feedback. Enable will still be attempted, "
            "but hold-pose is unavailable."
        )

    left_ok = enable_arm_service("left_arm", True)
    right_ok = enable_arm_service("right_arm", True)

    rospy.loginfo(
        "Enable service call results: left=%s, right=%s",
        left_ok,
        right_ok
    )

    # Extra fallback: the Piper wrapper also listens to enable_flag.
    rospy.loginfo(
        "Publishing /left_arm/enable_flag and /right_arm/enable_flag "
        "as extra enable signal."
    )

    publish_enable_flag(
        left_enable_pub,
        right_enable_pub,
        True,
        seconds=1.5
    )

    if got_feedback:
        rospy.loginfo("Continuing hold command after enabling.")

        for _ in range(10):
            if rospy.is_shutdown():
                return

            if left_feedback is not None:
                publish_joint(left_pub, left_feedback)

            if right_feedback is not None:
                publish_joint(right_pub, right_feedback)

            hold_rate.sleep()

    rospy.loginfo("Safe enable sequence finished.")


# ============================================================
# Main
# ============================================================

def main():
    rospy.init_node("dual_piper_servo_teleop_debug")

    # Important:
    # Read ROS params AFTER init_node.
    serial_port = rospy.get_param("~serial_port", "/dev/ttyACM0")
    baud = int(rospy.get_param("~baud", 1000000))
    timeout = float(rospy.get_param("~timeout", 0.015))
    rate_hz = float(rospy.get_param("~rate_hz", 25.0))

    piper_margin_rad = float(rospy.get_param("~piper_margin_rad", 0.08))
    max_step_rad = float(rospy.get_param("~max_step_rad", 0.01))

    enable_on_start = get_bool_param("~enable_on_start", False)

    debug_period = float(rospy.get_param("~debug_period", 0.5))

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

    left_enable_pub = rospy.Publisher(
        "/left_arm/enable_flag",
        Bool,
        queue_size=1
    )

    right_enable_pub = rospy.Publisher(
        "/right_arm/enable_flag",
        Bool,
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

    rospy.loginfo("======================================================")
    rospy.loginfo("dual_piper_servo_teleop_debug started")
    rospy.loginfo("serial_port: %s", serial_port)
    rospy.loginfo("baud: %d", baud)
    rospy.loginfo("timeout: %.4f", timeout)
    rospy.loginfo("rate_hz: %.2f", rate_hz)
    rospy.loginfo("piper_margin_rad: %.4f", piper_margin_rad)
    rospy.loginfo("max_step_rad: %.4f", max_step_rad)
    rospy.loginfo("enable_on_start: %s", enable_on_start)
    rospy.loginfo("debug_period: %.2f", debug_period)
    rospy.loginfo("Command topics:")
    rospy.loginfo("  /left_arm/joint_ctrl_single")
    rospy.loginfo("  /right_arm/joint_ctrl_single")
    rospy.loginfo("Feedback topics:")
    rospy.loginfo("  /left_arm/joint_states_single")
    rospy.loginfo("  /right_arm/joint_states_single")
    rospy.loginfo("======================================================")

    time.sleep(0.5)

    servo_reader = ServoBusReader(
        serial_port,
        baud,
        timeout
    )

    if enable_on_start:
        safe_enable_sequence(
            left_pub,
            right_pub,
            left_enable_pub,
            right_enable_pub
        )
    else:
        rospy.loginfo(
            "enable_on_start is false. "
            "This node will NOT enable the Piper arms."
        )

    prev_left = None
    prev_right = None
    last_debug_print = 0.0

    rate = rospy.Rate(rate_hz)

    while not rospy.is_shutdown():
        left_target, left_raw, left_deg = build_arm_target(
            servo_reader,
            LEFT_TABLE,
            LEFT_J6_FIXED,
            piper_margin_rad
        )

        right_target, right_raw, right_deg = build_arm_target(
            servo_reader,
            RIGHT_TABLE,
            RIGHT_J6_FIXED,
            piper_margin_rad
        )

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

        left_cmd = slew_limit(
            prev_left,
            left_target,
            max_step_rad
        )

        right_cmd = slew_limit(
            prev_right,
            right_target,
            max_step_rad
        )

        prev_left = left_cmd
        prev_right = right_cmd

        publish_joint(left_pub, left_cmd)
        publish_joint(right_pub, right_cmd)

        now = time.time()

        if now - last_debug_print >= debug_period:
            last_debug_print = now

            left_subs = left_pub.get_num_connections()
            right_subs = right_pub.get_num_connections()

            if left_feedback_time is None:
                left_fb_age = None
            else:
                left_fb_age = now - left_feedback_time

            if right_feedback_time is None:
                right_fb_age = None
            else:
                right_fb_age = now - right_feedback_time

            rospy.loginfo("--------------- TELEOP DEBUG ---------------")
            rospy.loginfo(
                "Command topic subscribers: left=%d right=%d",
                left_subs,
                right_subs
            )

            if left_subs == 0:
                rospy.logwarn(
                    "No subscriber on /left_arm/joint_ctrl_single. "
                    "Piper left node may not be running or namespace is different."
                )

            if right_subs == 0:
                rospy.logwarn(
                    "No subscriber on /right_arm/joint_ctrl_single. "
                    "Piper right node may not be running or namespace is different."
                )

            rospy.loginfo("LEFT servo raw: %s", fmt_dict(left_raw, 0))
            rospy.loginfo("LEFT servo deg: %s", fmt_dict(left_deg, 1))
            rospy.loginfo("LEFT piper target(rad): %s", fmt(left_target, 3))
            rospy.loginfo("LEFT piper command(rad): %s", fmt(left_cmd, 3))
            rospy.loginfo(
                "LEFT piper feedback(rad): %s age=%s",
                fmt(left_feedback, 3),
                "None" if left_fb_age is None else "{:.2f}s".format(left_fb_age)
            )

            rospy.loginfo("RIGHT servo raw: %s", fmt_dict(right_raw, 0))
            rospy.loginfo("RIGHT servo deg: %s", fmt_dict(right_deg, 1))
            rospy.loginfo("RIGHT piper target(rad): %s", fmt(right_target, 3))
            rospy.loginfo("RIGHT piper command(rad): %s", fmt(right_cmd, 3))
            rospy.loginfo(
                "RIGHT piper feedback(rad): %s age=%s",
                fmt(right_feedback, 3),
                "None" if right_fb_age is None else "{:.2f}s".format(right_fb_age)
            )

            rospy.loginfo("--------------------------------------------")

        rate.sleep()


if __name__ == "__main__":
    main()
