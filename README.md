本仓库为ros2-px4通信验证，实现gazebo里利用ros2控制多旋翼飞行器进行航点飞行

🚀 快速开始  
1. 创建工作空间并克隆代码

```bash
cd ~
git clone https://github.com/Bridge-song/ros2_ws.git
```
用VScode打开
```bash
# 1. 打开一个新终端
# 2. Source ROS2 Jazzy 环境
source /opt/ros/jazzy/setup.bash

# 3. 进入你的工作空间并 source
cd ~/ros2_ws
source install/setup.bash

# 4. 从同一个终端启动 VS Code
code .
```

2. 安装px4_msgs (仓库里自带了，不知道需不需要重新下)

```bash
cd ~/ros2_ws/src
git clone https://github.com/PX4/px4_msgs.git

# 需要提前先编译一遍px4_msgs
cd ~/ros2_ws
colcon build --packages-select px4_msgs
```

3. 编译  
```bash

cd ~/ros2_ws
colcon build --packages-select uav_flight_controller_py --symlink-install
source install/setup.bash
```

4. 运行仿真（4个终端）  

终端1 - PX4 Gazebo仿真  
```bash

cd ~/PX4-Autopilot
make px4_sitl gz_x500
```

终端2 - DDS桥接  
```bash

MicroXRCEAgent udp4 -p 8888
```

终端3 - 控制节点 编译+运行 
```bash

cd ~/ros2_ws
colcon build --packages-select uav_flight_controller_py --symlink-install
source install/setup.bash
ros2 run uav_flight_controller_py flight_control_node
```

终端4 - QGC地面站
```bash
cd ~
chmod +x ./QGroundControl-x86_64.AppImage
./QGroundControl-x86_64.AppImage
```
