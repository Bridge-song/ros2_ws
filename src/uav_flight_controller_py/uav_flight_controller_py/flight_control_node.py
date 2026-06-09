#!/usr/bin/env python3
import math
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleOdometry,
    VehicleStatus,
)


class UavFlightController(Node):
    def __init__(self):
        super().__init__('uav_flight_controller')

        self.get_logger().info('=' * 50)
        self.get_logger().info('UAV Flight Controller Node Starting...')
        self.get_logger().info('=' * 50)

        # 配置QoS（必须与PX4兼容）
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=10,
        )

        self.command_pub = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos)
        self.offboard_control_mode_pub = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos)
        self.trajectory_pub = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos)

        self.odom_sub = self.create_subscription(
            VehicleOdometry, '/fmu/out/vehicle_odometry',
            self.odom_callback, qos)
        self.status_sub = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status',
            self.status_callback, qos)

        self.declare_parameter('waypoint_tolerance', 0.5)
        self.declare_parameter('waypoint_hold_time', 2.0)
        self.declare_parameter('use_relative_waypoints', True)
        self.declare_parameter('control_frequency', 20.0)
        self.declare_parameter('heartbeat_frequency', 10.0)
        self.declare_parameter('waypoints', [
            '0.0, 0.0, -3.0',
            '50.0, 0.0, -5.0',
            '50.0, 50.0, -5.0',
            '0.0, 50.0, -5.0',
            '0.0, 0.0, -3.0',
            '0.0, 0.0, -0.5',
            '0.0, 0.0, 0.0',
        ])

        self.waypoint_tolerance = float(
            self.get_parameter('waypoint_tolerance').value)
        self.waypoint_hold_time = float(
            self.get_parameter('waypoint_hold_time').value)
        self.use_relative_waypoints = bool(
            self.get_parameter('use_relative_waypoints').value)
        control_frequency = float(self.get_parameter('control_frequency').value)
        heartbeat_frequency = float(
            self.get_parameter('heartbeat_frequency').value)

        # 状态变量
        self.current_pose = None
        self.home_position = None
        self.nav_state = None
        self.armed = False
        self.offboard_engaged = False
        self.last_arm_command_time = None
        self.last_offboard_command_time = None
        self.offboard_setpoint_counter = 0
        self.waypoint_reached_since = None
        self.mission_complete = False
        self.landing_phase = False  # 着陆阶段标志
        self.landing_started = False  # 开始着陆标志

        # 飞行任务（NED坐标系：x北 y东 z下；高度为负数表示向上飞）
        self.mission_points = self.load_mission_points()
        self.current_waypoint_index = 0

        self.control_timer = self.create_timer(
            1.0 / control_frequency, self.control_loop)
        # PX4要求进入/维持Offboard时持续发送OffboardControlMode和setpoint。
        self.heartbeat_timer = self.create_timer(
            1.0 / heartbeat_frequency, self.heartbeat)

        self.get_logger().info('Node initialized, waiting for PX4 data...')
        self.get_logger().info(f'Loaded {len(self.mission_points)} waypoints')

    def load_mission_points(self):
        """从参数读取航点，并转换成内部字典格式。"""
        waypoints = self.get_parameter('waypoints').value
        mission_points = []

        for index, waypoint in enumerate(waypoints):
            try:
                if isinstance(waypoint, str):
                    values = [
                        float(value.strip())
                        for value in waypoint.split(',')
                    ]
                else:
                    values = [float(value) for value in waypoint]
            except (TypeError, ValueError):
                self.get_logger().warn(
                    f'Ignoring invalid waypoint {index}: {waypoint}')
                continue

            if len(values) != 3:
                self.get_logger().warn(
                    f'Ignoring invalid waypoint {index}: expected x,y,z')
                continue

            mission_points.append({
                'x': values[0],
                'y': values[1],
                'z': values[2],
                'name': f'Waypoint {index}',
            })

        if not mission_points:
            self.get_logger().warn(
                'No valid waypoints configured, using default takeoff point')
            mission_points.append({
                'x': 0.0,
                'y': 0.0,
                'z': -3.0,
                'name': 'Default takeoff',
            })

        return mission_points

    def odom_callback(self, msg):
        """接收当前位姿"""
        self.current_pose = msg

        if self.home_position is None:
            self.home_position = [
                float(msg.position[0]),
                float(msg.position[1]),
                float(msg.position[2]),
            ]
            self.get_logger().info(
                'Home position set to: '
                f'({self.home_position[0]:.2f}, '
                f'{self.home_position[1]:.2f}, '
                f'{self.home_position[2]:.2f})')

        if hasattr(self, '_odom_count'):
            self._odom_count += 1
        else:
            self._odom_count = 0

        if self._odom_count % 50 == 0:
            self.get_logger().debug(
                f'Position: x={msg.position[0]:.2f}, '
                f'y={msg.position[1]:.2f}, '
                f'z={msg.position[2]:.2f}'
            )

    def status_callback(self, msg):
        """接收飞行状态"""
        self.nav_state = msg.nav_state
        self.armed = (msg.arming_state == VehicleStatus.ARMING_STATE_ARMED)

        if self.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            if not self.offboard_engaged:
                self.offboard_engaged = True
                self.get_logger().info('Offboard mode engaged')
        else:
            self.offboard_engaged = False

    def send_command(self, command, param1=0.0, param2=0.0, param3=0.0):
        """发送PX4车辆命令"""
        msg = VehicleCommand()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.command = command
        msg.param1 = float(param1)
        msg.param2 = float(param2)
        msg.param3 = float(param3)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self.command_pub.publish(msg)

    def arm(self):
        """解锁电机（让螺旋桨转动）"""
        self.send_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
        self.get_logger().info('ARM command sent')

    def disarm(self):
        """上锁电机"""
        self.send_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0)
        self.get_logger().info('DISARM command sent')

    def engage_offboard_mode(self):
        """切换到Offboard模式（由外部控制）"""
        # VEHICLE_CMD_DO_SET_MODE: param1=1(主模式), param2=6(offboard)
        self.send_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
        self.get_logger().info('Offboard mode command sent')

    def publish_offboard_control_mode(self):
        """声明当前Offboard控制类型为位置控制。"""
        msg = OffboardControlMode()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.thrust_and_torque = False
        msg.direct_actuator = False
        self.offboard_control_mode_pub.publish(msg)

    def goto_position(self, x, y, z, yaw=0.0):
        """发送目标位置"""
        msg = TrajectorySetpoint()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.position = [float(x), float(y), float(z)]
        msg.velocity = [math.nan, math.nan, math.nan]
        msg.acceleration = [math.nan, math.nan, math.nan]
        msg.jerk = [math.nan, math.nan, math.nan]
        msg.yaw = float(yaw)
        msg.yawspeed = math.nan
        self.trajectory_pub.publish(msg)

    def get_target_position(self, target):
        """根据参数决定航点使用绝对NED坐标，还是起飞点相对坐标。"""
        if self.use_relative_waypoints and self.home_position is not None:
            return {
                'x': self.home_position[0] + target['x'],
                'y': self.home_position[1] + target['y'],
                'z': self.home_position[2] + target['z'],
                'name': target['name'],
            }

        return target

    def get_distance_to_target(self, target):
        """计算到目标点的欧氏距离"""
        if self.current_pose is None:
            return float('inf')

        target_position = self.get_target_position(target)
        dx = target_position['x'] - self.current_pose.position[0]
        dy = target_position['y'] - self.current_pose.position[1]
        dz = target_position['z'] - self.current_pose.position[2]
        return math.sqrt(dx * dx + dy * dy + dz * dz)

    def heartbeat(self):
        """发送Offboard心跳和当前位置目标。"""
        if self.mission_complete:
            return

        self.publish_offboard_control_mode()

        if self.current_pose is None:
            return

        if self.use_relative_waypoints and self.home_position is None:
            return

        if self.current_waypoint_index < len(self.mission_points):
            wp = self.get_target_position(
                self.mission_points[self.current_waypoint_index])
            self.goto_position(wp['x'], wp['y'], wp['z'])

        if self.offboard_setpoint_counter < 20:
            self.offboard_setpoint_counter += 1

    def control_loop(self):
        """主控制状态机"""
        if self.current_pose is None:
            self.get_logger().warn(
                'Waiting for odometry data...', throttle_duration_sec=2.0)
            return

        if self.use_relative_waypoints and self.home_position is None:
            self.get_logger().warn(
                'Waiting for home position...', throttle_duration_sec=2.0)
            return

        # 在 control_loop 函数中，替换航点完成的判断部分
        if self.current_waypoint_index >= len(self.mission_points):
            if not self.mission_complete:
                self.get_logger().info('Mission completed, initiating soft landing...')
                self.mission_complete = True
                self.landing_phase = True  # 进入着陆阶段
                self.landing_started = False
            return

        # 在 control_loop 函数中添加着陆逻辑（放在解锁和offboard判断之后）
        # 着陆阶段：分步进行
        if self.mission_complete and self.landing_phase:
            if not self.landing_started:
                # 第一步：降低到0.5米高度
                self.get_logger().info('Descending to 0.5m...')
                self.landing_started = True
                self.landing_target_z = -0.5  # 先降到0.5米
                return
    
            current_z = self.current_pose.position[2]
    
            # 第二步：到达0.5米后，缓慢降落
            if current_z > -0.6 and current_z < -0.4:  # 接近0.5米
                self.get_logger().info('Soft landing to ground...')
                # 缓慢下降
                self.goto_position(0.0, 0.0, -0.05)  # 目标非常接近地面
                time.sleep(0.1)
        
                # 检查是否真的落地（高度<0.1米）
                if current_z > -0.1:
                    self.get_logger().info('Landed! Disarming...')
                    self.disarm()
                    time.sleep(1.0)
                    self.control_timer.cancel()
                    return
            else:
                # 正常下降到0.5米
                self.goto_position(0.0, 0.0, self.landing_target_z)
    
            return

        target = self.mission_points[self.current_waypoint_index]
        target_position = self.get_target_position(target)
        distance = self.get_distance_to_target(target)
        now = self.get_clock().now().seconds_nanoseconds()[0]

        if distance < self.waypoint_tolerance:
            if self.waypoint_reached_since is None:
                self.waypoint_reached_since = now
                self.get_logger().info(
                    f'Holding waypoint {self.current_waypoint_index}: '
                    f'{target["name"]} (distance: {distance:.2f}m)')
                return

            if now - self.waypoint_reached_since < self.waypoint_hold_time:
                return

            self.get_logger().info(
                f'Reached waypoint {self.current_waypoint_index}: '
                f'{target["name"]} (distance: {distance:.2f}m)')
            self.current_waypoint_index += 1
            self.waypoint_reached_since = None
            return
        else:
            self.waypoint_reached_since = None

        # PX4进入Offboard前必须先连续收到一段时间的OffboardControlMode/setpoint。
        if self.offboard_setpoint_counter < 10:
            self.get_logger().info(
                'Streaming offboard setpoints before mode switch...',
                throttle_duration_sec=2.0)
            return

        # 状态机：Offboard -> 解锁 -> 自动飞行。
        if not self.offboard_engaged:
            if (self.last_offboard_command_time is None
                    or now - self.last_offboard_command_time >= 2):
                self.engage_offboard_mode()
                self.last_offboard_command_time = now
            return

        if not self.armed:
            if (self.last_arm_command_time is None
                    or now - self.last_arm_command_time >= 2):
                self.arm()
                self.last_arm_command_time = now
            return

        self.goto_position(
            target_position['x'], target_position['y'], target_position['z'])

        if not hasattr(self, '_last_print'):
            self._last_print = 0

        if now - self._last_print > 5:
            self.get_logger().info(
                f'Going to: {target["name"]} | '
                f'Distance: {distance:.2f}m | '
                f'Target: ({target_position["x"]:.1f}, '
                f'{target_position["y"]:.1f}, '
                f'{target_position["z"]:.1f}) | '
                f'Position: ({self.current_pose.position[0]:.1f}, '
                f'{self.current_pose.position[1]:.1f}, '
                f'{self.current_pose.position[2]:.1f})'
            )
            self._last_print = now


def main(args=None):
    rclpy.init(args=args)
    node = UavFlightController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down by user...')
        node.disarm()
    except Exception as e:
        node.get_logger().error(f'Unexpected error: {e}')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
