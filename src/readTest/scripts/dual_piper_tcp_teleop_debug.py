#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import time
import socket
import rospy

from sensor_msgs.msg import JointState
from std_msgs.msg import Bool

try:
    from piper_msgs.srv import Enable
except Exception:
    Enable = None


# ============================================================
# Piper JointState:
#   position[0] = joint1
#   position[1] = joint2
#   position[2] = joint3
#   position[3] = joint4
#   position[4] = joint5
#   position[5] = joint6
#   position[6] = gripper
# ============================================================

JOINT_NAMES = [
    "joint1", "joint2", "joint3",
    "joint4", "joint5", "joint6",
    "gripper"
]


# ============================================================
# TCP channel order from exo mini arm
#
# 原始 TCP 16 路:
#
#   ch1  = 原始右臂 joint1
#   ch2  = 原始右臂 joint2
#   ch3  = 原始右臂 joint3
#   ch4  = 原始右臂 joint4
#   ch5  = 原始右臂 joint5
#   ch6  = 原始右臂 joint6
#   ch7  = 原始右臂 joint7
#   ch8  = 原始右臂 gripper
#
#   ch9  = 原始左臂 joint1
#   ch10 = 原始左臂 joint2
#   ch11 = 原始左臂 joint3
#   ch12 = 原始左臂 joint4
#   ch13 = 原始左臂 joint5
#   ch14 = 原始左臂 joint6
#   ch15 = 原始左臂 joint7
#   ch16 = 原始左臂 gripper
#
# 你说实际控制左右手反了，所以现在:
#
#   Piper 左臂使用 ch1, ch2, ch4, ch5, ch6, ch7, ch8
#   Piper 右臂使用 ch9, ch10, ch12, ch13, ch14, ch15, ch16
#
# 前五个关节映射保持你现在已经调好的逻辑。
# 新增:
#   左 Piper joint6 = ch7
#   右 Piper joint6 = ch15
#   左 Piper gripper = ch8
#   右 Piper gripper = ch16
# ============================================================


# ============================================================
# 前 5 个关节映射表
#
# table item:
#   channel, input_start_deg, input_end_deg, piper_start_rad, piper_end_rad
#
# 现在从臂每次使用前校 0:
#   旧 180 -> 新 0
#   旧 270 -> 新 90
#   旧 90  -> 新 -90
#   旧 103 -> 新 -77
# ============================================================

LEFT_TABLE = [
    (1, 0.0, 90.0, 0.00, -1.50),
    (2, 0.0, -77.0, 3.00, 1.50),
    (4, 0.0, 90.0, -2.80, -1.30),
    (5, 0.0, -90.0, 0.00, 1.50),
    (6, 0.0, 90.0, 0.00, 1.30),
]

RIGHT_TABLE = [
    (9, 0.0, -90.0, 0.00, 1.62),
    (10, 0.0, 90.0, 3.10, 1.60),
    (12, 0.0, -90.0, -2.80, -1.30),
    (13, 0.0, 90.0, 0.00, -1.50),
    (14, 0.0, -90.0, 0.00, 1.20),
]


# ============================================================
# Feedback globals
# ============================================================

left_feedback = None
right_feedback = None
left_feedback_time = None
right_feedback_time = None


def pad_joint_command(q, default_gripper=0.04):
    """
    保证 JointState position 一定是 7 个值:
      joint1~joint6 + gripper

    如果 Piper feedback 只有 6 个关节，就补一个 gripper 默认值。
    """
    if q is None:
        return None

    out = list(q)

    if len(out) >= 7:
        return out[:7]

    while len(out) < 6:
        out.append(0.0)

    if len(out) == 6:
        out.append(default_gripper)

    return out[:7]


def left_feedback_cb(msg):
    global left_feedback, left_feedback_time

    if len(msg.position) >= 6:
        left_feedback = pad_joint_command(msg.position)
        left_feedback_time = time.time()


def right_feedback_cb(msg):
    global right_feedback, right_feedback_time

    if len(msg.position) >= 6:
        right_feedback = pad_joint_command(msg.position)
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


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


def fmt(vals, ndigits=3):
    if vals is None:
        return "None"

    return "[" + ", ".join([str(round(v, ndigits)) for v in vals]) + "]"


def fmt_dict(d, ndigits=1):
    parts = []

    for k in sorted(d.keys(), key=lambda x: str(x)):
        v = d[k]

        if isinstance(v, float):
            parts.append("{}:{:.{}f}".format(k, v, ndigits))
        else:
            parts.append("{}:{}".format(k, v))

    return "{" + ", ".join(parts) + "}"


def map_one_joint(input_deg, input_a, input_b, piper_a, piper_b, margin_rad):
    input_deg = clamp(
        input_deg,
        min(input_a, input_b),
        max(input_a, input_b)
    )

    if abs(input_b - input_a) < 1e-6:
        return piper_a

    t = (input_deg - input_a) / (input_b - input_a)
    piper = piper_a + t * (piper_b - piper_a)

    p_lo = min(piper_a, piper_b)
    p_hi = max(piper_a, piper_b)

    span = p_hi - p_lo
    margin = min(max(margin_rad, 0.0), span * 0.25)

    return clamp(piper, p_lo + margin, p_hi - margin)


def map_gripper(
    gripper_deg,
    input_open_deg,
    input_close_deg,
    output_open,
    output_close
):
    """
    从臂夹爪角度 -> Piper 夹爪命令。

    公式:
      input_open_deg  -> output_open
      input_close_deg -> output_close

    例如:
      input_open_deg  = 0
      input_close_deg = -15
      output_open     = 0.0
      output_close    = 0.09

    意思:
      ch8/ch16 = 0 度    -> Piper gripper = 0.0
      ch8/ch16 = -15 度  -> Piper gripper = 0.09

    注意:
      这里会 clamp。
      如果输入方向/范围不对，输出会一直卡在 open 或 close。
    """
    gripper_deg = clamp(
        gripper_deg,
        min(input_open_deg, input_close_deg),
        max(input_open_deg, input_close_deg)
    )

    if abs(input_close_deg - input_open_deg) < 1e-6:
        return output_open

    t = (gripper_deg - input_open_deg) / (input_close_deg - input_open_deg)
    cmd = output_open + t * (output_close - output_open)

    return clamp(
        cmd,
        min(output_open, output_close),
        max(output_open, output_close)
    )


def slew_limit(prev, target, max_step_rad):
    if prev is None:
        return list(target)

    if len(prev) != len(target):
        return list(target)

    out = []

    for old, new in zip(prev, target):
        delta = new - old

        if abs(delta) > max_step_rad:
            new = old + math.copysign(max_step_rad, delta)

        out.append(new)

    return out


# ============================================================
# TCP reader
# ============================================================

class ExoMiniArmTcpReader:
    def __init__(self, ip, port, socket_timeout, frame_timeout):
        self.ip = ip
        self.port = int(port)
        self.socket_timeout = float(socket_timeout)
        self.frame_timeout = float(frame_timeout)

        self.client = None

        # 16 路缓存，ch1~ch16
        self.last_raw_codes = [2048] * 16
        self.last_angles_deg = [0.0] * 16

        # 读取 16 路关节角度的命令
        self.read_cmd = bytes([0xFD, 0xFD, 0x02, 0x01, 0xFE])

        self.open()

    def open(self):
        self.close()

        try:
            self.client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.client.settimeout(self.socket_timeout)
            self.client.connect((self.ip, self.port))

            # 连接成功后，把 recv timeout 改短，避免主循环卡太久。
            self.client.settimeout(self.frame_timeout)

            time.sleep(0.01)

            rospy.loginfo(
                "Connected to exo mini arm TCP: %s:%d",
                self.ip,
                self.port
            )

            return True

        except Exception as e:
            rospy.logerr_throttle(
                2.0,
                "Cannot connect to exo mini arm TCP %s:%d: %s",
                self.ip,
                self.port,
                str(e)
            )

            self.close()
            return False

    def close(self):
        try:
            if self.client is not None:
                self.client.close()
        except Exception:
            pass

        self.client = None

    def reconnect(self):
        self.close()
        time.sleep(0.2)
        return self.open()

    def _recv_exact(self, n):
        buf = b""
        start = time.time()

        while len(buf) < n:
            if time.time() - start > self.frame_timeout:
                return None

            try:
                chunk = self.client.recv(n - len(buf))
            except socket.timeout:
                return None

            if not chunk:
                return None

            buf += chunk

        return buf

    def _read_frame_37(self):
        if self.client is None:
            if not self.reconnect():
                return None

        try:
            self.client.sendall(self.read_cmd)

            frame = self._recv_exact(37)

            if frame is None:
                rospy.logwarn_throttle(
                    1.0,
                    "TCP read timeout: did not receive complete 37-byte frame."
                )
                return None

            if len(frame) != 37:
                rospy.logwarn_throttle(
                    1.0,
                    "Invalid frame length: %d",
                    len(frame)
                )
                return None

            if frame[0] != 0xFD or frame[1] != 0xFD:
                rospy.logwarn_throttle(
                    1.0,
                    "Invalid frame header: %s",
                    frame.hex(" ")
                )
                return None

            if frame[-1] != 0xFE:
                rospy.logwarn_throttle(
                    1.0,
                    "Invalid frame tail: %s",
                    frame.hex(" ")
                )
                return None

            return frame

        except Exception as e:
            rospy.logwarn_throttle(
                1.0,
                "TCP read failed: %s. Reconnecting.",
                str(e)
            )
            self.reconnect()
            return None

    def update(self):
        """
        读取一次 37 字节帧，并更新 ch1~ch16。
        失败时继续使用 last valid。
        """
        frame = self._read_frame_37()

        if frame is None:
            return False

        # 37 字节格式:
        #   FD FD ?? ?? + 32 字节 payload + FE
        #
        # 你原始代码:
        #   read_hex = frame.hex()
        #   payload_hex = read_hex[8:-2]
        #
        # 等价于:
        payload = frame[4:-1]

        if len(payload) != 32:
            rospy.logwarn_throttle(
                1.0,
                "Invalid payload length: %d",
                len(payload)
            )
            return False

        raw_codes = []
        angles_deg = []

        for i in range(16):
            lo = payload[i * 2]
            hi = payload[i * 2 + 1]

            raw_code = lo | (hi << 8)

            # 和你的 TCP 读取代码一致:
            # raw 2048 -> 0 deg
            angle = 180.0 * (raw_code - 2048) / 2048.0

            raw_codes.append(raw_code)
            angles_deg.append(round(angle, 2))

        self.last_raw_codes = raw_codes
        self.last_angles_deg = angles_deg

        return True

    def read_channel(self, ch):
        """
        直接读取 TCP channel。
        ch 是 1~16。
        返回:
            raw_code, angle_deg
        """
        idx = int(ch) - 1

        if idx < 0 or idx >= 16:
            rospy.logwarn_throttle(
                1.0,
                "Invalid TCP channel: ch%d",
                ch
            )
            return 2048, 0.0

        return self.last_raw_codes[idx], self.last_angles_deg[idx]

    def get_all_angles_deg(self):
        return list(self.last_angles_deg)


# ============================================================
# Mapping and publishing
# ============================================================

def build_arm_target(
    reader,
    table,
    margin_rad,
    j6_channel,
    j6_input_a_deg,
    j6_input_b_deg,
    j6_output_a_rad,
    j6_output_b_rad,
    gripper_channel,
    gripper_input_open_deg,
    gripper_input_close_deg,
    gripper_output_open,
    gripper_output_close
):
    joints = []
    raw_by_ch = {}
    deg_by_ch = {}

    # 前 5 个关节：保持你现在已经调好的映射
    for ch, input_a, input_b, piper_a, piper_b in table:
        raw, deg = reader.read_channel(ch)

        raw_by_ch["ch{}".format(ch)] = raw
        deg_by_ch["ch{}".format(ch)] = deg

        rad = map_one_joint(
            deg,
            input_a,
            input_b,
            piper_a,
            piper_b,
            margin_rad
        )

        joints.append(rad)

    # 第 6 个关节：ch7 / ch15
    j6_raw, j6_deg = reader.read_channel(j6_channel)

    raw_by_ch["ch{}".format(j6_channel)] = j6_raw
    deg_by_ch["ch{}".format(j6_channel)] = j6_deg

    j6_rad = map_one_joint(
        j6_deg,
        j6_input_a_deg,
        j6_input_b_deg,
        j6_output_a_rad,
        j6_output_b_rad,
        margin_rad
    )

    joints.append(j6_rad)

    # 夹爪：ch8 / ch16
    grip_raw, grip_deg = reader.read_channel(gripper_channel)

    raw_by_ch["ch{}".format(gripper_channel)] = grip_raw
    deg_by_ch["ch{}".format(gripper_channel)] = grip_deg

    gripper_cmd = map_gripper(
        grip_deg,
        gripper_input_open_deg,
        gripper_input_close_deg,
        gripper_output_open,
        gripper_output_close
    )

    joints.append(gripper_cmd)

    return joints, raw_by_ch, deg_by_ch


def publish_joint(pub, q, gripper_effort=1.0):
    msg = JointState()
    msg.header.stamp = rospy.Time.now()
    msg.name = JOINT_NAMES
    msg.position = pad_joint_command(q)

    # 如果 Piper 控制节点读取 effort[6]，这个值可能影响夹爪力度。
    # 如果控制节点忽略 effort，则不会有副作用。
    msg.effort = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, float(gripper_effort)]

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


def safe_enable_sequence(
    left_pub,
    right_pub,
    left_enable_pub,
    right_enable_pub,
    gripper_effort
):
    rospy.loginfo("Starting safe enable sequence.")

    got_feedback = wait_for_feedback(timeout_sec=3.0)
    hold_rate = rospy.Rate(20)

    if got_feedback:
        rospy.loginfo("Holding current Piper feedback before enabling.")

        for _ in range(10):
            if rospy.is_shutdown():
                return

            if left_feedback is not None:
                publish_joint(left_pub, left_feedback, gripper_effort)

            if right_feedback is not None:
                publish_joint(right_pub, right_feedback, gripper_effort)

            hold_rate.sleep()
    else:
        rospy.logwarn(
            "No Piper feedback. Enable will still be attempted, "
            "but hold-pose is unavailable."
        )

    left_ok = enable_arm_service("left_arm", True)
    right_ok = enable_arm_service("right_arm", True)

    rospy.loginfo(
        "Enable service call results: left=%s right=%s",
        left_ok,
        right_ok
    )

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
                publish_joint(left_pub, left_feedback, gripper_effort)

            if right_feedback is not None:
                publish_joint(right_pub, right_feedback, gripper_effort)

            hold_rate.sleep()

    rospy.loginfo("Safe enable sequence finished.")


# ============================================================
# Main
# ============================================================

def main():
    rospy.init_node("dual_piper_tcp_teleop_debug")

    tcp_ip = rospy.get_param("~tcp_ip", "192.168.4.1")
    tcp_port = int(rospy.get_param("~tcp_port", 10000))
    socket_timeout = float(rospy.get_param("~socket_timeout", 2.0))
    frame_timeout = float(rospy.get_param("~frame_timeout", 0.05))

    rate_hz = float(rospy.get_param("~rate_hz", 25.0))

    piper_margin_rad = float(rospy.get_param("~piper_margin_rad", 0.08))
    max_step_rad = float(rospy.get_param("~max_step_rad", 0.01))

    enable_on_start = get_bool_param("~enable_on_start", False)
    debug_period = float(rospy.get_param("~debug_period", 0.5))

    # ========================================================
    # 第 6 关节参数
    #
    # 保持你现在已经调好的默认值，不动。
    # ========================================================

    left_j6_channel = int(rospy.get_param("~left_j6_channel", 7))
    right_j6_channel = int(rospy.get_param("~right_j6_channel", 15))

    left_j6_input_a_deg = float(rospy.get_param("~left_j6_input_a_deg", 0.0))
    left_j6_input_b_deg = float(rospy.get_param("~left_j6_input_b_deg", 90.0))
    left_j6_output_a_rad = float(rospy.get_param("~left_j6_output_a_rad", 2.70))
    left_j6_output_b_rad = float(rospy.get_param("~left_j6_output_b_rad", 1.20))

    right_j6_input_a_deg = float(rospy.get_param("~right_j6_input_a_deg", 0.0))
    right_j6_input_b_deg = float(rospy.get_param("~right_j6_input_b_deg", 90.0))
    right_j6_output_a_rad = float(rospy.get_param("~right_j6_output_a_rad", -2.70))
    right_j6_output_b_rad = float(rospy.get_param("~right_j6_output_b_rad", -1.20))

    # ========================================================
    # Gripper params
    #
    # 兼容旧参数:
    #   _gripper_input_open_deg
    #   _gripper_input_close_deg
    #
    # 新增左右独立参数:
    #   _left_gripper_input_open_deg
    #   _left_gripper_input_close_deg
    #   _right_gripper_input_open_deg
    #   _right_gripper_input_close_deg
    #
    # 如果你不传左右独立参数，就自动沿用旧参数。
    # 所以你现在的命令不改，ch16 会保持现在能跑的行为。
    # ========================================================

    left_gripper_channel = int(rospy.get_param("~left_gripper_channel", 8))
    right_gripper_channel = int(rospy.get_param("~right_gripper_channel", 16))

    # 旧的公共输入范围，保留兼容。
    gripper_input_open_deg = float(rospy.get_param("~gripper_input_open_deg", 0.0))
    gripper_input_close_deg = float(rospy.get_param("~gripper_input_close_deg", 60.0))

    # 新的左右独立输入范围。
    # 没传时，默认等于旧的公共输入范围。
    left_gripper_input_open_deg = float(
        rospy.get_param(
            "~left_gripper_input_open_deg",
            gripper_input_open_deg
        )
    )
    left_gripper_input_close_deg = float(
        rospy.get_param(
            "~left_gripper_input_close_deg",
            gripper_input_close_deg
        )
    )

    right_gripper_input_open_deg = float(
        rospy.get_param(
            "~right_gripper_input_open_deg",
            gripper_input_open_deg
        )
    )
    right_gripper_input_close_deg = float(
        rospy.get_param(
            "~right_gripper_input_close_deg",
            gripper_input_close_deg
        )
    )

    left_gripper_output_open = float(rospy.get_param("~left_gripper_output_open", 0.04))
    left_gripper_output_close = float(rospy.get_param("~left_gripper_output_close", 0.0))

    right_gripper_output_open = float(rospy.get_param("~right_gripper_output_open", 0.04))
    right_gripper_output_close = float(rospy.get_param("~right_gripper_output_close", 0.0))

    gripper_effort = float(rospy.get_param("~gripper_effort", 1.0))

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
    rospy.loginfo("dual_piper_tcp_teleop_debug started")
    rospy.loginfo("tcp_ip: %s", tcp_ip)
    rospy.loginfo("tcp_port: %d", tcp_port)
    rospy.loginfo("socket_timeout: %.4f", socket_timeout)
    rospy.loginfo("frame_timeout: %.4f", frame_timeout)
    rospy.loginfo("rate_hz: %.2f", rate_hz)
    rospy.loginfo("piper_margin_rad: %.4f", piper_margin_rad)
    rospy.loginfo("max_step_rad: %.4f", max_step_rad)
    rospy.loginfo("enable_on_start: %s", enable_on_start)
    rospy.loginfo("debug_period: %.2f", debug_period)

    rospy.loginfo("left_j6_channel: ch%d", left_j6_channel)
    rospy.loginfo("right_j6_channel: ch%d", right_j6_channel)

    rospy.loginfo("left_gripper_channel: ch%d", left_gripper_channel)
    rospy.loginfo("right_gripper_channel: ch%d", right_gripper_channel)

    rospy.loginfo("legacy gripper_input_open_deg: %.2f", gripper_input_open_deg)
    rospy.loginfo("legacy gripper_input_close_deg: %.2f", gripper_input_close_deg)

    rospy.loginfo("left_gripper_input_open_deg: %.2f", left_gripper_input_open_deg)
    rospy.loginfo("left_gripper_input_close_deg: %.2f", left_gripper_input_close_deg)
    rospy.loginfo("right_gripper_input_open_deg: %.2f", right_gripper_input_open_deg)
    rospy.loginfo("right_gripper_input_close_deg: %.2f", right_gripper_input_close_deg)

    rospy.loginfo("left_gripper_output_open: %.4f", left_gripper_output_open)
    rospy.loginfo("left_gripper_output_close: %.4f", left_gripper_output_close)
    rospy.loginfo("right_gripper_output_open: %.4f", right_gripper_output_open)
    rospy.loginfo("right_gripper_output_close: %.4f", right_gripper_output_close)
    rospy.loginfo("gripper_effort: %.4f", gripper_effort)

    rospy.loginfo("Command topics:")
    rospy.loginfo("  /left_arm/joint_ctrl_single")
    rospy.loginfo("  /right_arm/joint_ctrl_single")
    rospy.loginfo("Feedback topics:")
    rospy.loginfo("  /left_arm/joint_states_single")
    rospy.loginfo("  /right_arm/joint_states_single")
    rospy.loginfo("======================================================")

    time.sleep(0.5)

    reader = ExoMiniArmTcpReader(
        tcp_ip,
        tcp_port,
        socket_timeout,
        frame_timeout
    )

    if enable_on_start:
        safe_enable_sequence(
            left_pub,
            right_pub,
            left_enable_pub,
            right_enable_pub,
            gripper_effort
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
        # 每个循环只读一次完整 16 路 TCP 数据
        reader.update()

        left_target, left_raw, left_deg = build_arm_target(
            reader,
            LEFT_TABLE,
            piper_margin_rad,
            left_j6_channel,
            left_j6_input_a_deg,
            left_j6_input_b_deg,
            left_j6_output_a_rad,
            left_j6_output_b_rad,
            left_gripper_channel,
            left_gripper_input_open_deg,
            left_gripper_input_close_deg,
            left_gripper_output_open,
            left_gripper_output_close
        )

        right_target, right_raw, right_deg = build_arm_target(
            reader,
            RIGHT_TABLE,
            piper_margin_rad,
            right_j6_channel,
            right_j6_input_a_deg,
            right_j6_input_b_deg,
            right_j6_output_a_rad,
            right_j6_output_b_rad,
            right_gripper_channel,
            right_gripper_input_open_deg,
            right_gripper_input_close_deg,
            right_gripper_output_open,
            right_gripper_output_close
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

        publish_joint(left_pub, left_cmd, gripper_effort)
        publish_joint(right_pub, right_cmd, gripper_effort)

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

            all_tcp_angles = reader.get_all_angles_deg()

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

            rospy.loginfo(
                "TCP all angles deg: ch1-8=%s ch9-16=%s",
                fmt(all_tcp_angles[:8], 2),
                fmt(all_tcp_angles[8:], 2)
            )

            rospy.loginfo("LEFT ch raw_code: %s", fmt_dict(left_raw, 0))
            rospy.loginfo("LEFT ch deg: %s", fmt_dict(left_deg, 2))
            rospy.loginfo("LEFT piper target(rad/grip): %s", fmt(left_target, 3))
            rospy.loginfo("LEFT piper command(rad/grip): %s", fmt(left_cmd, 3))
            rospy.loginfo(
                "LEFT piper feedback(rad/grip): %s age=%s",
                fmt(left_feedback, 3),
                "None" if left_fb_age is None else "{:.2f}s".format(left_fb_age)
            )

            rospy.loginfo("RIGHT ch raw_code: %s", fmt_dict(right_raw, 0))
            rospy.loginfo("RIGHT ch deg: %s", fmt_dict(right_deg, 2))
            rospy.loginfo("RIGHT piper target(rad/grip): %s", fmt(right_target, 3))
            rospy.loginfo("RIGHT piper command(rad/grip): %s", fmt(right_cmd, 3))
            rospy.loginfo(
                "RIGHT piper feedback(rad/grip): %s age=%s",
                fmt(right_feedback, 3),
                "None" if right_fb_age is None else "{:.2f}s".format(right_fb_age)
            )

            rospy.loginfo("--------------------------------------------")

        rate.sleep()

    reader.close()


if __name__ == "__main__":
    main()
