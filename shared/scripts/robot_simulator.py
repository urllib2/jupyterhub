#!/usr/bin/env python3
"""
RosForge Robot Simulator
Automatically starts robot_state_publisher and publishes joint states.
Students only need to run this one file.
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState, JointState
from std_msgs.msg import String
from std_srvs.srv import Empty
from tf2_ros import TransformBroadcaster
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
import math
import time
import subprocess
import tempfile
import os
import yaml

ROBOT_URDF = """<?xml version="1.0"?>
<robot name="my_robot">

  <link name="base_footprint"/>

  <joint name="base_joint" type="fixed">
    <parent link="base_footprint"/>
    <child link="base_link"/>
    <origin xyz="0 0 0.15" rpy="0 0 0"/>
  </joint>

  <link name="base_link">
    <visual>
      <geometry><box size="0.6 0.4 0.2"/></geometry>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <material name="blue"><color rgba="0.2 0.4 0.8 1.0"/></material>
    </visual>
  </link>

  <joint name="base_right_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="base_right_wheel_link"/>
    <origin xyz="-0.15 -0.225 -0.05" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
  </joint>
  <link name="base_right_wheel_link">
    <visual>
      <geometry><cylinder radius="0.1" length="0.05"/></geometry>
      <origin xyz="0 0 0" rpy="1.5708 0 0"/>
      <material name="dark"><color rgba="0.1 0.1 0.1 1.0"/></material>
    </visual>
  </link>

  <joint name="base_left_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="base_left_wheel_link"/>
    <origin xyz="-0.15 0.225 -0.05" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
  </joint>
  <link name="base_left_wheel_link">
    <visual>
      <geometry><cylinder radius="0.1" length="0.05"/></geometry>
      <origin xyz="0 0 0" rpy="1.5708 0 0"/>
      <material name="dark"><color rgba="0.1 0.1 0.1 1.0"/></material>
    </visual>
  </link>

  <joint name="caster_wheel_joint" type="fixed">
    <parent link="base_link"/>
    <child link="caster_wheel_link"/>
    <origin xyz="0.2 0 -0.10" rpy="0 0 0"/>
  </joint>
  <link name="caster_wheel_link">
    <visual>
      <geometry><sphere radius="0.05"/></geometry>
      <material name="grey"><color rgba="0.6 0.6 0.6 1.0"/></material>
    </visual>
  </link>

</robot>
"""

def start_robot_state_publisher():
    """Launch robot_state_publisher via yaml params file."""
    param_file = tempfile.NamedTemporaryFile(
        mode='w', suffix='.yaml', delete=False)
    yaml.dump({
        'robot_state_publisher': {
            'ros__parameters': {
                'robot_description': ROBOT_URDF
            }
        }
    }, param_file)
    param_file.close()
    env = os.environ.copy()
    proc = subprocess.Popen(
        ['ros2', 'run', 'robot_state_publisher', 'robot_state_publisher',
         '--ros-args', '--params-file', param_file.name],
        env=env,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL
    )
    return proc, param_file.name


class RobotSimulator(Node):
    def __init__(self):
        super().__init__('mobile_robot')

        self.declare_parameter('max_linear_speed', 1.0)
        self.declare_parameter('max_angular_speed', 2.0)
        self.declare_parameter('publish_rate', 10.0)

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.vx = 0.0
        self.vz = 0.0
        self.battery = 100.0
        self.stopped = False
        self.wheel_pos = 0.0

        # Latched QoS for robot_description
        latched_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE
        )

        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.battery_pub = self.create_publisher(BatteryState, 'battery_state', 10)
        self.description_pub = self.create_publisher(String, 'robot_description', latched_qos)
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.cmd_sub = self.create_subscription(
            Twist, 'cmd_vel', self.cmd_callback, 10)

        self.create_service(Empty, 'emergency_stop', self.emergency_stop_cb)
        self.create_service(Empty, 'reset_position', self.reset_cb)

        rate = self.get_parameter('publish_rate').value
        self.timer = self.create_timer(1.0 / rate, self.update)
        self.create_timer(1.0, self.publish_description)
        self.last_time = time.time()

        self.get_logger().info('=' * 50)
        self.get_logger().info('  RosForge Robot Simulator started!')
        self.get_logger().info('  (same robot you will build in Module 5)')
        self.get_logger().info('=' * 50)
        self.get_logger().info('Topics:')
        self.get_logger().info('  /cmd_vel        <- send velocity commands')
        self.get_logger().info('  /odom           -> robot position')
        self.get_logger().info('  /battery_state  -> battery level')
        self.get_logger().info('Services:')
        self.get_logger().info('  /emergency_stop')
        self.get_logger().info('  /reset_position')
        self.get_logger().info('RViz: Fixed Frame=odom, Add RobotModel+TF')
        self.get_logger().info('=' * 50)

    def publish_description(self):
        msg = String()
        msg.data = ROBOT_URDF
        self.description_pub.publish(msg)

    def cmd_callback(self, msg):
        if self.stopped:
            self.get_logger().warn('Emergency stop active!')
            return
        max_v = self.get_parameter('max_linear_speed').value
        max_w = self.get_parameter('max_angular_speed').value
        self.vx = max(-max_v, min(max_v, msg.linear.x))
        self.vz = max(-max_w, min(max_w, msg.angular.z))

    def emergency_stop_cb(self, request, response):
        self.stopped = True
        self.vx = 0.0
        self.vz = 0.0
        self.get_logger().warn('EMERGENCY STOP!')
        return response

    def reset_cb(self, request, response):
        self.x = self.y = self.theta = self.vx = self.vz = 0.0
        self.stopped = False
        self.get_logger().info('Position reset')
        return response

    def update(self):
        now = time.time()
        dt = now - self.last_time
        self.last_time = now

        self.x += self.vx * math.cos(self.theta) * dt
        self.y += self.vx * math.sin(self.theta) * dt
        self.theta += self.vz * dt
        self.wheel_pos += self.vx * dt / 0.1  # wheel radius = 0.1m
        self.battery = max(0.0, self.battery - 0.001)

        stamp = self.get_clock().now().to_msg()

        # TF: odom → base_footprint
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_footprint'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.z = math.sin(self.theta / 2.0)
        t.transform.rotation.w = math.cos(self.theta / 2.0)
        self.tf_broadcaster.sendTransform(t)

        # Odometry
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = math.sin(self.theta / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.theta / 2.0)
        odom.twist.twist.linear.x = self.vx
        odom.twist.twist.angular.z = self.vz
        self.odom_pub.publish(odom)

        # Joint states — wheels rotate with robot speed
        js = JointState()
        js.header.stamp = stamp
        js.name = ['base_left_wheel_joint', 'base_right_wheel_joint']
        js.position = [self.wheel_pos, self.wheel_pos]
        js.velocity = [self.vx / 0.1, self.vx / 0.1]
        js.effort = [0.0, 0.0]
        self.joint_pub.publish(js)

        # Battery
        bat = BatteryState()
        bat.header.stamp = stamp
        bat.percentage = self.battery / 100.0
        bat.voltage = 12.0 * (self.battery / 100.0)
        bat.present = True
        self.battery_pub.publish(bat)


def main(args=None):
    rsp_proc, param_file = start_robot_state_publisher()
    time.sleep(1)

    rclpy.init(args=args)
    node = RobotSimulator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        rsp_proc.terminate()
        try:
            os.unlink(param_file)
        except Exception:
            pass


if __name__ == '__main__':
    main()
