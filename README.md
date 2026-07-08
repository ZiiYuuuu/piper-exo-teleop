# Piper Dual Arm TCP Teleoperation

This repository contains the full ROS catkin workspace for controlling dual Piper robot arms using a TCP-based exoskeleton mini arm.
<img width="2185" height="1284" alt="9dcd94558a7bd98ab97d2922c056bce0" src="https://github.com/user-attachments/assets/a9da4242-585d-4c62-a84d-6f02d6776846" />

**Hardware Setup**
<img width="5371" height="4028" alt="1ecd2cfccf44e34cfd5410665d62ac6b" src="https://github.com/user-attachments/assets/b4480c21-dc2c-4b84-8783-e0abd1e8bcfb" />

The main teleoperation script is located at:

```bash
src/readTest/scripts/dual_piper_tcp_teleop_debug.py
```

## Channel Mapping

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
```

## Run Procedures

Turn on the robot, plug the teleoperation arm

**Connect to WIFI VKARM_CONTROLLER**

password： 12345678

### Preparation

Activate can0 and can1, baud 1M

```bash
sudo ip link set can0 up type can bitrate 1000000
sudo ip link set can1 up type can bitrate 1000000
```

Set the Teleoperation Robotic Arm's Servo motor to 0; this needs to be set every time after turning the power back on

- *File path：~/catkin_ws/src/testFile*

```bash
python3 test_tcp_arm.py
```
Hardware Setup
<img width="1702" height="1276" alt="f63f3a0554160e09cf2f53c533bc25ec" src="https://github.com/user-attachments/assets/68a699cd-e9a2-4b97-8b08-dc9c63c1ba62" />

Under the left side, there is a USB-C Port as a Serial Port (didn't use in this method)
<img width="1918" height="1278" alt="155e9b025d7d2cc843932529a1489239" src="https://github.com/user-attachments/assets/1c2b2256-10c0-46c8-a293-4243bb24670a" />

Sample Output
<img width="1277" height="909" alt="d87603d3f8556946dc36a8bf28997bed" src="https://github.com/user-attachments/assets/b932b6a9-1fb5-4b37-897c-91c9a622210a" />

Set IP connection

- *File path：~/catkin_ws/src/testFile*

```bash
python3 test_tcp_arm.py
```

### Main Program Running Procedure

Step 1 - Terminal 1
```bash
roslaunch readTest read_all.launch
```

Step 2 - Terminal 2
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
  _gripper_input_close_deg:=15 \
  _left_gripper_input_open_deg:=0 \
  _left_gripper_input_close_deg:=15 \
  _right_gripper_input_open_deg:=0 \
  _right_gripper_input_close_deg:=-15 \
  _left_gripper_output_open:=0.0 \
  _left_gripper_output_close:=0.09 \
  _right_gripper_output_open:=0.0 \
  _right_gripper_output_close:=0.09 \
  _gripper_effort:=1.0 \
  _debug_period:=0.5
```
**Sample Outputs**

Step 1

<img width="960" height="322" alt="GIF_20260707203156107" src="https://github.com/user-attachments/assets/0dbf756c-4d65-4b9d-b0d7-58df922a0604" />

Step 2 

<img width="960" height="322" alt="GIF_20260707203037472" src="https://github.com/user-attachments/assets/d04e4d55-bb82-4d6a-8f48-3d9a3dabe1d6" />

**Robot Action**

Master & Slave Arm

<img width="960" height="540" alt="GIF_20260707205036550" src="https://github.com/user-attachments/assets/7220fdb7-1685-4bc8-921a-28a88df208c3" />


Different Speed

<img width="960" height="540" alt="GIF_20260707205355382" src="https://github.com/user-attachments/assets/a7ee4ce0-3253-4010-af94-110bb2dfd213" />


Gripper

<img width="540" height="960" alt="GIF_20260707210548628" src="https://github.com/user-attachments/assets/3aaa51b1-a54d-4761-89bb-f6b9eb95303d" />


**ROS Run Command Parameters**

General Runtime Parameters

| Parameter           | Description                                                                                                  |       Default |       Example |
| ------------------- | ------------------------------------------------------------------------------------------------------------ | ------------: | ------------: |
| `_tcp_ip`           | TCP server IP address of the exoskeleton controller.                                                         | `192.168.4.1` | `192.168.4.1` |
| `_tcp_port`         | TCP port used to receive the 16-channel exoskeleton data.                                                    |       `10000` |       `10000` |
| `_socket_timeout`   | TCP socket connection timeout in seconds.                                                                    |         `2.0` |         `2.0` |
| `_frame_timeout`    | Timeout for receiving one complete 37-byte TCP frame.                                                        |        `0.05` |        `0.05` |
| `_rate_hz`          | Main loop frequency in Hz. Controls how often data is read and commands are published.                       |        `25.0` |        `25.0` |
| `_enable_on_start`  | Whether to automatically enable both Piper arms when the node starts.                                        |       `false` |        `true` |
| `_max_step_rad`     | Maximum joint command step per loop in radians. Smaller values make motion slower and smoother.              |        `0.01` |       `0.005` |
| `_piper_margin_rad` | Safety margin near Piper joint limits, in radians. Prevents commands from reaching exact mapping boundaries. |        `0.08` |        `0.10` |
| `_debug_period`     | Debug print interval in seconds.                                                                             |         `0.5` |         `0.5` |


Sixth Joint Parameters

| Parameter                | Description                                                                                         | Default | Example |
| ------------------------ | --------------------------------------------------------------------------------------------------- | ------: | ------: |
| `_left_j6_channel`       | TCP channel used for the left Piper joint6.                                                         |     `7` |     `7` |
| `_right_j6_channel`      | TCP channel used for the right Piper joint6.                                                        |    `15` |    `15` |
| `_left_j6_input_a_deg`   | Input start angle for left joint6 mapping, in degrees.                                              |   `0.0` |     `0` |
| `_left_j6_input_b_deg`   | Input end angle for left joint6 mapping, in degrees. Change the sign if the direction is reversed.  |  `90.0` |    `90` |
| `_left_j6_output_a_rad`  | Piper left joint6 target angle when the input is at point A, in radians.                            |  `2.70` |  `2.70` |
| `_left_j6_output_b_rad`  | Piper left joint6 target angle when the input is at point B, in radians.                            |  `1.20` |  `1.20` |
| `_right_j6_input_a_deg`  | Input start angle for right joint6 mapping, in degrees.                                             |   `0.0` |     `0` |
| `_right_j6_input_b_deg`  | Input end angle for right joint6 mapping, in degrees. Change the sign if the direction is reversed. |  `90.0` |    `90` |
| `_right_j6_output_a_rad` | Piper right joint6 target angle when the input is at point A, in radians.                           | `-2.70` | `-2.70` |
| `_right_j6_output_b_rad` | Piper right joint6 target angle when the input is at point B, in radians.                           | `-1.20` | `-1.20` |


Gripper Parameters

| Parameter                        | Description                                                                                                                           |            Default | Example |
| -------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------- | -----------------: | ------: |
| `_left_gripper_channel`          | TCP channel used for the left Piper gripper.                                                                                          |                `8` |     `8` |
| `_right_gripper_channel`         | TCP channel used for the right Piper gripper.                                                                                         |               `16` |    `16` |
| `_gripper_input_open_deg`        | Global fallback input angle for the open gripper position, in degrees.                                                                |              `0.0` |     `0` |
| `_gripper_input_close_deg`       | Global fallback input angle for the closed gripper position, in degrees.                                                              |             `60.0` |    `15` |
| `_left_gripper_input_open_deg`   | Input angle for the left gripper open position, in degrees. Usually `0` after zero calibration.                                       |  global open value |     `0` |
| `_left_gripper_input_close_deg`  | Input angle for the left gripper closed position, in degrees. Smaller absolute values make the gripper more sensitive.                | global close value |    `15` |
| `_right_gripper_input_open_deg`  | Input angle for the right gripper open position, in degrees. Usually `0` after zero calibration.                                      |  global open value |     `0` |
| `_right_gripper_input_close_deg` | Input angle for the right gripper closed position, in degrees. Use a negative value if the right gripper input direction is reversed. | global close value |   `-15` |
| `_left_gripper_output_open`      | Piper command value for the left gripper open position.                                                                               |             `0.04` |   `0.0` |
| `_left_gripper_output_close`     | Piper command value for the left gripper closed position.                                                                             |              `0.0` |  `0.09` |
| `_right_gripper_output_open`     | Piper command value for the right gripper open position.                                                                              |             `0.04` |   `0.0` |
| `_right_gripper_output_close`    | Piper command value for the right gripper closed position.                                                                            |              `0.0` |  `0.09` |
| `_gripper_effort`                | Gripper effort value. This only affects force if the Piper control node reads `JointState.effort[6]`.                                 |              `1.0` |   `1.0` |


- **How to Tune the Gripper**

- If the gripper direction is reversed
  
  1. Swap open and close output values.
  ```bash
  _left_gripper_output_open:=0.09
  _left_gripper_output_close:=0.0
  ```
  or:
  ```bash
  _right_gripper_output_open:=0.09
  _right_gripper_output_close:=0.0
  ```
  2. Reverse the input direction by changing the sign of input_close_deg
  ```bash
  _right_gripper_input_close_deg:=15
  ```
  or:
  ```bash
  _right_gripper_input_close_deg:=-15
  ```

- If the gripper only moves after bending the exoskeleton gripper very far

  Reduce the absolute value of the closed input angle.
  
  ```bash
  _left_gripper_input_close_deg:=10
  _right_gripper_input_close_deg:=-10
  ```
- If the gripper is too sensitive
  
  Increase the absolute value of the close input angle.
  
  ```bash
  _left_gripper_input_close_deg:=20
  _right_gripper_input_close_deg:=-20
  ```
- If the Piper gripper does not open or close enough

  Increase the output range.
  
  ```bash
  _left_gripper_output_open:=0.0
  _left_gripper_output_close:=0.10
  _right_gripper_output_open:=0.0
  _right_gripper_output_close:=0.10
  ```
