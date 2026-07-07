# 准备CAN
# 激活 can0 和 can1，波特率 1M
sudo ip link set can0 up type can bitrate 1000000
sudo ip link set can1 up type can bitrate 1000000

# 检查状态（确保看到 <NOARP,UP,LOWER_UP,...> 里的 UP）
ip link show can0
ip link show can1



# 运行文件
cd ~/catkin_ws
source devel/setup.bash
roslaunch readTest read_all.launch



# debug
# 查看当前总线上的所有话题
rostopic list

# 实时打印左臂/右臂的关节数据（看看是不是真的在动）
rostopic echo /left_arm/joint_states
rostopic echo /right_arm/joint_states


1.
roslaunch readTest read_all.launch
2.
rosrun readTest dual_piper_servo_teleop_debug.py \
  _enable_on_start:=true \
  _max_step_rad:=0.01 \
  _piper_margin_rad:=0.08 \
  _debug_period:=0.5
  
  3.
  rosrun readTest dual_piper_servo_teleop_debug.py \
  _enable_on_start:=true \
  _max_step_rad:=0.05 \
  _piper_margin_rad:=0.05 \
  _debug_period:=1.0
  
  4.
  rosservice call /left_arm/enable_srv "enable_request: false"
rosservice call /right_arm/enable_srv "enable_request: false"

rosservice call /left_arm/enable_srv "enable_request: true"
rosservice call /right_arm/enable_srv "enable_request: true"
