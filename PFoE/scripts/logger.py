#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray, Bool, String
from pfoe_msg.msg import Event
from rclpy.serialization import serialize_message
import rosbag2_py
from datetime import datetime
import os


class Logger(Node):
    def __init__(self):
        super().__init__('logger')

        self.declare_parameter('bag_directory', os.path.expanduser('~/.ros/pfoe_bags'))
        self.bag_directory = self.get_parameter('bag_directory').value

        os.makedirs(self.bag_directory, exist_ok=True)

        self.teaching_mode = False
        self.bag_writer = None
        self.current_feature = []
        self.current_cmd_vel = Twist()
        self.current_bag_path = ""

        self.event_pub = self.create_publisher(Event, 'event', 10)
        self.teaching_mode_pub = self.create_publisher(Bool, 'teaching_mode', 10)
        self.replay_mode_pub = self.create_publisher(Bool, 'replay_mode', 10)
        self.bag_path_pub = self.create_publisher(String, 'bag_path', 10)

        self.feature_sub = self.create_subscription(Float32MultiArray, 'image_feature', self.feature_callback, 10)
        self.cmd_vel_sub = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        self.teaching_mode_sub = self.create_subscription(Bool, 'teaching_mode_toggle', self.teaching_mode_toggle_callback, 10)

        # Timer for periodic recording (10 Hz)
        self.timer = self.create_timer(0.1, self.record_event)

        self.get_logger().info('Logger initialized')
        self.get_logger().info(f'Bag directory: {self.bag_directory}')

    def feature_callback(self, msg):
        self.current_feature = list(msg.data)

    def cmd_vel_callback(self, msg):
        self.current_cmd_vel = msg

    def teaching_mode_toggle_callback(self, msg):
        new_mode = msg.data

        if new_mode and not self.teaching_mode:
            self.start_recording()
            replay_msg = Bool()
            replay_msg.data = False
            self.replay_mode_pub.publish(replay_msg)

        elif not new_mode and self.teaching_mode:
            self.stop_recording()
            replay_msg = Bool()
            replay_msg.data = True
            self.replay_mode_pub.publish(replay_msg)
            # Publish bag file path for replay node
            if self.current_bag_path:
                from std_msgs.msg import String
                bag_path_msg = String()
                bag_path_msg.data = self.current_bag_path
                self.bag_path_pub.publish(bag_path_msg)
                self.get_logger().info(f'Published bag path for replay: {self.current_bag_path}')

        self.teaching_mode = new_mode

        mode_msg = Bool()
        mode_msg.data = self.teaching_mode
        self.teaching_mode_pub.publish(mode_msg)

    def start_recording(self):
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        bag_path = os.path.join(self.bag_directory, f'teaching_{timestamp}')

        storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id='sqlite3')
        converter_options = rosbag2_py.ConverterOptions(
            input_serialization_format='cdr',
            output_serialization_format='cdr'
        )

        self.bag_writer = rosbag2_py.SequentialWriter()
        self.bag_writer.open(storage_options, converter_options)

        # Create topic info for Event message
        topic_info = rosbag2_py.TopicMetadata(
            name='/event',
            type='pfoe_msg/msg/Event',
            serialization_format='cdr'
        )
        self.bag_writer.create_topic(topic_info)
        self.current_bag_path = bag_path
        self.get_logger().info(f'Started recording to: {bag_path}')

    def stop_recording(self):
        if self.bag_writer is not None:
            del self.bag_writer
            self.bag_writer = None
            self.get_logger().info('Stopped recording')

            if self.current_bag_path:
                bag_path_msg = String()
                bag_path_msg.data = self.current_bag_path
                self.bag_path_pub.publish(bag_path_msg)
                self.get_logger().info(f'Published bag path: {self.current_bag_path}')

    def record_event(self):
        if not self.teaching_mode:
            return

        if len(self.current_feature) == 0:
            self.get_logger().warn(f"No image feature （self.current_feature : 0）")
            return

        event = Event()
        event.feature = self.current_feature
        event.linear_x = float(self.current_cmd_vel.linear.x)
        event.angular_z = float(self.current_cmd_vel.angular.z)
        event.timestamp = self.get_clock().now().to_msg()

        self.event_pub.publish(event)

        if self.bag_writer is not None:
            self.bag_writer.write(
                '/event',
                serialize_message(event),
                self.get_clock().now().nanoseconds
            )

    def destroy_node(self):
        self.stop_recording()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    try:
        node = Logger()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
