#!/usr/bin/env python3
"""
Joy Controller Node
Converts gamepad input to cmd_vel commands for robot control during teaching phase.
Simplified version without level-based speed adjustment.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool


class JoyController(Node):
    def __init__(self):
        super().__init__('joy_controller')

        # Parameters
        self.declare_parameter('linear_scale', 0.2)    # Maximum linear velocity [m/s]
        self.declare_parameter('angular_scale', 0.098)  # Maximum angular velocity [rad/s] (≈π/32)
        self.declare_parameter('button_deadman', 0)     # Button to enable control
        self.declare_parameter('axis_linear', 1)        # Axis for linear velocity
        self.declare_parameter('axis_angular', 0)       # Axis for angular velocity

        # Get parameters
        self.linear_scale = self.get_parameter('linear_scale').value
        self.angular_scale = self.get_parameter('angular_scale').value
        self.button_deadman = self.get_parameter('button_deadman').value
        self.axis_linear = self.get_parameter('axis_linear').value
        self.axis_angular = self.get_parameter('axis_angular').value

        # State variables
        self.teaching_mode = False

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

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

        self.get_logger().info('Joy Controller initialized (simplified mode)')
        self.get_logger().info(f'Max linear velocity: {self.linear_scale} m/s')
        self.get_logger().info(f'Max angular velocity: {self.angular_scale} rad/s')

    def teaching_mode_callback(self, msg):
        """Update teaching mode state"""
        self.teaching_mode = msg.data
        if self.teaching_mode:
            self.get_logger().info('Teaching mode: ON')
        else:
            self.get_logger().info('Teaching mode: OFF')

    def joy_callback(self, joy_msg):
        """Process joystick input"""
        # Only process if in teaching mode
        if not self.teaching_mode:
            return

        # Check bounds
        if len(joy_msg.buttons) <= self.button_deadman:
            self.get_logger().warn('Not enough buttons in joy message')
            return

        if len(joy_msg.axes) <= max(self.axis_linear, self.axis_angular):
            self.get_logger().warn('Not enough axes in joy message')
            return

        # Publish velocity when deadman button is pressed
        if joy_msg.buttons[self.button_deadman] == 1:
            twist = Twist()
            # Direct mapping: axis value (-1.0 to 1.0) * max_scale
            twist.linear.x = joy_msg.axes[self.axis_linear] * self.linear_scale
            twist.angular.z = joy_msg.axes[self.axis_angular] * self.angular_scale
            self.cmd_vel_pub.publish(twist)


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
