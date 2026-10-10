#!/usr/bin/env python3
"""
Runs alongside the robot. Writes topic data to files for notebook widgets.
  /tmp/battery.txt  — latest battery percentage
  /tmp/odom.txt     — x,y positions, one per line, appended continuously
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from nav_msgs.msg import Odometry

class Monitor(Node):
    def __init__(self):
        super().__init__('rosforge_monitor')
        self.create_subscription(BatteryState, '/battery_state', self.on_battery, 10)
        self.create_subscription(Odometry, '/odom', self.on_odom, 10)
        open('/tmp/odom.txt', 'w').close()
        open('/tmp/battery.txt', 'w').close()
        self.get_logger().info('ROSForge monitor started')

    def on_battery(self, msg):
        pct = msg.percentage
        pct = round(pct * 100, 1) if pct <= 1.0 else round(pct, 1)
        with open('/tmp/battery.txt', 'w') as f:
            f.write(str(pct))

    def on_odom(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        with open('/tmp/odom.txt', 'a') as f:
            f.write(f'{x},{y}\n')

def main():
    rclpy.init()
    node = Monitor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
