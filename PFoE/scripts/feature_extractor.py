#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge
import cv2
import torch
import torch.nn as nn
import numpy as np
from ament_index_python.packages import get_package_share_directory
import os


class FeatureExtractor(Node):
    def __init__(self):
        super().__init__('feature_extractor')

        self.declare_parameter('model_path', '')
        model_path = self.get_parameter('model_path').value

        package_dir = get_package_share_directory('pfoe')
        model_path = os.path.join(package_dir, 'weights', 'placenet.pt')
        
        self.device = torch.device('cuda')
        self.model = torch.jit.load(model_path, map_location=self.device)
        self.model.eval()
        self.bridge = CvBridge()

        self.feature_pub = self.create_publisher(Float32MultiArray, 'image_feature', 10)
        self.image_sub = self.create_subscription(Image, '/zed/zed_node/rgb/image_rect_color', self.image_callback, 10)

        # normalization parameters
        self.mean = np.array([0.485, 0.456, 0.406], dtype=np.float32)
        self.std = np.array([0.229, 0.224, 0.225], dtype=np.float32)

        self.get_logger().info(f'Loading model from: {model_path}')
        self.get_logger().info(f'Using device: {self.device}')
        self.get_logger().info('Model loaded successfully')
        self.get_logger().info('Feature Extractor initialized')

    def center_crop_square(self, image):
        h, w = image.shape[:2]
        min_dim = min(h, w)

        top = (h - min_dim) // 2
        left = (w - min_dim) // 2

        return image[top:top+min_dim, left:left+min_dim]

    def preprocess_image(self, cv_image):
        rgb_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)

        square_image = self.center_crop_square(rgb_image)
        resized = cv2.resize(square_image, (85, 85))
        image_float = resized.astype(np.float32) / 255.0
        normalized = (image_float - self.mean) / self.std

        # Convert to tensor: (H, W, C) -> (C, H, W)
        tensor = torch.from_numpy(normalized).permute(2, 0, 1)

        # Add batch dimension: (C, H, W) -> (1, C, H, W)
        tensor = tensor.unsqueeze(0)

        return tensor.to(self.device)

    def extract_feature(self, image_tensor):
        with torch.no_grad():
            feature = self.model(image_tensor)
            feature_np = feature.cpu().squeeze().numpy()

        return feature_np

    def image_callback(self, msg):

        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        cv_image_bgr = cv_image[:, :, :3]  # the param for bgra8

        image_tensor = self.preprocess_image(cv_image_bgr)
        feature = self.extract_feature(image_tensor)

        feature_msg = Float32MultiArray()
        feature_msg.data = feature.tolist()
        self.feature_pub.publish(feature_msg)


def main(args=None):
    rclpy.init(args=args)

    try:
        node = FeatureExtractor()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        import traceback
        print(f'Error in feature_extractor: {e}')
        print(f'Traceback:\n{traceback.format_exc()}')
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
