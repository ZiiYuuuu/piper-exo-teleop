#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import JointState

def main():

    rospy.init_node("test_piper_joint")

    pub = rospy.Publisher(
        "/joint_states",
        JointState,
        queue_size=10
    )

    rospy.sleep(2.0)

    rate = rospy.Rate(20)

    direction = 1
    angle = 0.0

    while not rospy.is_shutdown():

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

        msg.position = [
            angle,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0
        ]

        pub.publish(msg)

        angle += direction * 0.02

        if angle > 0.5:
            direction = -1

        if angle < -0.5:
            direction = 1

        rate.sleep()

if __name__ == "__main__":
    main()
