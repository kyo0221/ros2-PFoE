#!/usr/bin/env python3
"""
Joy Controller Node
Converts gamepad input to cmd_vel commands for robot control during teaching phase.
Based on the original logicool_training.py from ROS1 implementation.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool
import math


class JoyController(Node):
    def __init__(self):
        super().__init__('joy_controller')

        # Parameters
        self.declare_parameter('linear_scale', 0.2)
        self.declare_parameter('angular_scale', 3.14 / 32.0)
        self.declare_parameter('button_deadman', 0)  # Button to enable control
        self.declare_parameter('button_level_up', 7)
        self.declare_parameter('button_level_down', 6)
        self.declare_parameter('axis_linear', 1)
        self.declare_parameter('axis_angular', 0)

        # Get parameters
        self.linear_scale = self.get_parameter('linear_scale').value
        self.angular_scale = self.get_parameter('angular_scale').value
        self.button_deadman = self.get_parameter('button_deadman').value
        self.button_level_up = self.get_parameter('button_level_up').value
        self.button_level_down = self.get_parameter('button_level_down').value
        self.axis_linear = self.get_parameter('axis_linear').value
        self.axis_angular = self.get_parameter('axis_angular').value

        # State variables
        self.level = 1
        self.teaching_mode = False

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.teaching_mode_pub = self.create_publisher(Bool, 'teaching_mode', 10)

        # Subscribers
        self.joy_sub = self.create_subscription(
            Joy,
            'joy',
            self.joy_callback,
            10
        )
        self.teaching_mode_sub = self.create_subscription(
            Bool,
            'teaching_mode',
            self.teaching_mode_callback,
            10
        )

        self.get_logger().info('Joy Controller initialized')
        self.get_logger().info(f'Linear scale: {self.linear_scale}, Angular scale: {self.angular_scale}')

    def limiter(self, lvl):
        """Limit level value between 1 and 5"""
        if lvl <= 0:
            return 1
        if lvl >= 6:
            return 5
        return lvl

    def teaching_mode_callback(self, msg):
        """Update teaching mode state"""
        self.teaching_mode = msg.data

    def joy_callback(self, joy_msg):
        """Process joystick input"""
        # Only process if in teaching mode
        if not self.teaching_mode:
            return

        # Check button bounds
        if len(joy_msg.buttons) <= max(self.button_deadman, self.button_level_up, self.button_level_down):
            self.get_logger().warn('Not enough buttons in joy message')
            return

        if len(joy_msg.axes) <= max(self.axis_linear, self.axis_angular):
            self.get_logger().warn('Not enough axes in joy message')
            return

        # Adjust level
        if joy_msg.buttons[self.button_level_up] == 1:
            self.level += 1
        if joy_msg.buttons[self.button_level_down] == 1:
            self.level -= 1
        self.level = self.limiter(self.level)

        # Publish velocity when deadman button is pressed
        if joy_msg.buttons[self.button_deadman] == 1:
            twist = Twist()
            twist.linear.x = joy_msg.axes[self.axis_linear] * self.linear_scale * self.level
            twist.angular.z = joy_msg.axes[self.axis_angular] * self.angular_scale * (self.level + 15)
            self.cmd_vel_pub.publish(twist)

        # Auto-decrease level when joystick is centered
        if joy_msg.axes[self.axis_linear] == 0 and joy_msg.axes[self.axis_angular] == 0:
            self.level -= 1
            self.level = self.limiter(self.level)


def main(args=None):
    rclpy.init(args=args)

    try:
        node = JoyController()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
