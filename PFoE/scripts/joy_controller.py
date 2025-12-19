import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool


class JoyController(Node):
    def __init__(self):
        super().__init__('joy_controller')

        self.declare_parameter('linear_vel', 0.8)
        self.declare_parameter('angular_vel', 1.0)
        self.declare_parameter('button_mode_toggle', 6)
        self.declare_parameter('axis_linear', 1)
        self.declare_parameter('axis_angular', 3)

        self.linear_scale = self.get_parameter('linear_vel').value
        self.angular_scale = self.get_parameter('angular_vel').value
        self.button_mode_toggle = self.get_parameter('button_mode_toggle').value
        self.axis_linear = self.get_parameter('axis_linear').value
        self.axis_angular = self.get_parameter('axis_angular').value

        self.teaching_mode = False
        self.last_mode_button_state = 0

        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.teaching_mode_toggle_pub = self.create_publisher(Bool, 'teaching_mode_toggle', 10)

        self.joy_sub = self.create_subscription(Joy, 'joy', self.joy_callback, 10)
        self.teaching_mode_sub = self.create_subscription(Bool, 'teaching_mode', self.teaching_mode_callback, 10)

        self.get_logger().info('Joy Controller initialized')
        self.get_logger().info(f'Max linear velocity: {self.linear_scale} m/s')
        self.get_logger().info(f'Max angular velocity: {self.angular_scale} rad/s')
        self.get_logger().info(f'Button {self.button_mode_toggle}: Toggle teaching/replay mode')

    def teaching_mode_callback(self, msg):
        self.teaching_mode = msg.data
        if self.teaching_mode:
            self.get_logger().info('=== Teaching Mode: ON ===')
        else:
            self.get_logger().info('=== Replay Mode: ON ===')

    def joy_callback(self, joy_msg):
        if joy_msg.buttons[self.button_mode_toggle] == 1 and self.last_mode_button_state == 0:
            toggle_msg = Bool()
            toggle_msg.data = not self.teaching_mode
            self.teaching_mode_toggle_pub.publish(toggle_msg)
            self.get_logger().info(f'{"Teaching" if not self.teaching_mode else "Replay"} mode requested')

        if self.teaching_mode:
            twist = Twist()
            twist.linear.x = joy_msg.axes[self.axis_linear] * self.linear_scale
            twist.angular.z = joy_msg.axes[self.axis_angular] * self.angular_scale
            self.cmd_vel_pub.publish(twist)

        self.last_mode_button_state = joy_msg.buttons[self.button_mode_toggle]


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
