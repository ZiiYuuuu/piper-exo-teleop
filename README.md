# Piper Dual Arm TCP Teleoperation

This repository contains the full ROS catkin workspace for controlling dual Piper robot arms using a TCP-based exoskeleton mini arm.

The main teleoperation script is located at:

```bash
src/readTest/scripts/dual_piper_tcp_teleop_debug.py
```


```bash
| Piper Arm | Piper Joint | TCP Channel |
| --------- | ----------- | ----------- |
| Left      | joint1      | ch1         |
| Left      | joint2      | ch2         |
| Left      | joint3      | ch4         |
| Left      | joint4      | ch5         |
| Left      | joint5      | ch6         |
| Left      | joint6      | ch7         |
| Left      | gripper     | ch8         |
| Right     | joint1      | ch9         |
| Right     | joint2      | ch10        |
| Right     | joint3      | ch12        |
| Right     | joint4      | ch13        |
| Right     | joint5      | ch14        |
| Right     | joint6      | ch15        |
| Right     | gripper     | ch16        |


```bash
rosrun readTest dual_piper_tcp_teleop_debug.py \
  _tcp_ip:=192.168.4.1 \
  _tcp_port:=10000 \
  _enable_on_start:=true \
  _max_step_rad:=0.005 \
  _piper_margin_rad:=0.10 \
  _left_j6_channel:=7 \
  _right_j6_channel:=15 \
  _left_gripper_channel:=8 \
  _right_gripper_channel:=16 \
  _gripper_input_open_deg:=0 \
  _gripper_input_close_deg:=-20 \
  _left_gripper_output_open:=0.0 \
  _left_gripper_output_close:=0.07 \
  _right_gripper_output_open:=0.0 \
  _right_gripper_output_close:=0.07 \
  _gripper_effort:=1.0 \
  _debug_period:=0.5
