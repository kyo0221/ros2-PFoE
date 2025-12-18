#!/usr/bin/env python3
"""
Logger Node
Records image features and robot actions during teaching phase.
Saves data to ROS2 bag file and publishes Event messages.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray, Bool
from pfoe_msg.msg import Event
from rclpy.serialization import serialize_message
import rosbag2_py
from datetime import datetime
import os


class Logger(Node):
    def __init__(self):
        super().__init__('logger')

        # Parameters
        self.declare_parameter('bag_directory', os.path.expanduser('~/.ros/pfoe_bags'))

        # Get parameters
        self.bag_directory = self.get_parameter('bag_directory').value

        # Create bag directory if it doesn't exist
        os.makedirs(self.bag_directory, exist_ok=True)

        # State variables
        self.teaching_mode = False
        self.bag_writer = None
        self.current_feature = []
        self.current_cmd_vel = Twist()

        # Publishers
        self.event_pub = self.create_publisher(Event, 'event', 10)
        self.teaching_mode_pub = self.create_publisher(Bool, 'teaching_mode', 10)

        # Subscribers
        self.feature_sub = self.create_subscription(
            Float32MultiArray,
            'image_feature',
            self.feature_callback,
            10
        )
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )
        self.teaching_mode_sub = self.create_subscription(
            Bool,
            'teaching_mode_toggle',
            self.teaching_mode_toggle_callback,
            10
        )

        # Timer for periodic recording (10 Hz)
        self.timer = self.create_timer(0.1, self.record_event)

        self.get_logger().info('Logger initialized')
        self.get_logger().info(f'Bag directory: {self.bag_directory}')

    def feature_callback(self, msg):
        """Store latest feature vector"""
        self.current_feature = list(msg.data)

    def cmd_vel_callback(self, msg):
        """Store latest cmd_vel"""
        self.current_cmd_vel = msg

    def teaching_mode_toggle_callback(self, msg):
        """Toggle teaching mode on/off"""
        new_mode = msg.data

        if new_mode and not self.teaching_mode:
            # Start teaching mode
            self.start_recording()
        elif not new_mode and self.teaching_mode:
            # Stop teaching mode
            self.stop_recording()

        self.teaching_mode = new_mode

        # Publish teaching mode state
        mode_msg = Bool()
        mode_msg.data = self.teaching_mode
        self.teaching_mode_pub.publish(mode_msg)

    def start_recording(self):
        """Start recording to bag file"""
        # Generate bag filename with timestamp
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        bag_path = os.path.join(self.bag_directory, f'teaching_{timestamp}')

        # Create bag writer
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

        # Store bag path as parameter
        self.declare_parameter('current_bag_file', bag_path)
        self.set_parameters([rclpy.parameter.Parameter('current_bag_file',
                                                        rclpy.Parameter.Type.STRING,
                                                        bag_path)])

        self.get_logger().info(f'Started recording to: {bag_path}')

    def stop_recording(self):
        """Stop recording and close bag file"""
        if self.bag_writer is not None:
            del self.bag_writer
            self.bag_writer = None
            self.get_logger().info('Stopped recording')

    def record_event(self):
        """Record current state as Event message"""
        if not self.teaching_mode:
            return

        if len(self.current_feature) == 0:
            # No feature available yet
            return

        # Create Event message
        event = Event()
        event.feature = self.current_feature
        event.linear_x = float(self.current_cmd_vel.linear.x)
        event.angular_z = float(self.current_cmd_vel.angular.z)
        event.timestamp = self.get_clock().now().to_msg()

        # Publish event
        self.event_pub.publish(event)

        # Write to bag file
        if self.bag_writer is not None:
            self.bag_writer.write(
                '/event',
                serialize_message(event),
                self.get_clock().now().nanoseconds
            )

    def destroy_node(self):
        """Cleanup on shutdown"""
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
