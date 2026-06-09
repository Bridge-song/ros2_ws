# UAV Flight Controller for ROS2 + PX4

## 功能
通过ROS2控制PX4仿真无人机自动飞行

## 依赖安装
```bash
# 安装px4_msgs
cd ~/ws_uav_control/src
git clone https://github.com/PX4/px4_msgs.git
cd ~/ws_uav_control
colcon build --packages-select px4_msgs