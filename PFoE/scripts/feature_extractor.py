#!/usr/bin/env python3
"""
Feature Extractor Node
Subscribes to camera images, extracts features using PlaceNet model,
and publishes the feature vectors.
"""

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

        # Parameters
        self.declare_parameter('model_path', '')
        self.declare_parameter('image_size', 85)
        self.declare_parameter('use_gpu', True)

        # Get parameters
        model_path = self.get_parameter('model_path').value
        self.image_size = self.get_parameter('image_size').value
        use_gpu = self.get_parameter('use_gpu').value

        # If model_path is empty, use default location
        if not model_path:
            package_dir = get_package_share_directory('PFoE')
            model_path = os.path.join(package_dir, 'weights', 'placenet.pt')

        self.get_logger().info(f'Loading model from: {model_path}')

        # Setup device
        self.device = torch.device('cuda' if use_gpu and torch.cuda.is_available() else 'cpu')
        self.get_logger().info(f'Using device: {self.device}')

        # Load model
        try:
            self.model = torch.jit.load(model_path, map_location=self.device)
            self.model.eval()
            self.get_logger().info('Model loaded successfully')
        except Exception as e:
            self.get_logger().error(f'Failed to load model: {e}')
            raise

        # CV Bridge
        self.bridge = CvBridge()

        # Publishers and Subscribers
        self.feature_pub = self.create_publisher(Float32MultiArray, 'image_feature', 10)
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

        # ImageNet normalization parameters
        self.mean = np.array([0.485, 0.456, 0.406], dtype=np.float32)
        self.std = np.array([0.229, 0.224, 0.225], dtype=np.float32)

        self.get_logger().info('Feature Extractor initialized')

    def center_crop_square(self, image):
        """
        Crop image to square from center
        Args:
            image: Input image (H, W, C)
        Returns:
            Cropped square image
        """
        h, w = image.shape[:2]
        min_dim = min(h, w)

        # Calculate crop coordinates
        top = (h - min_dim) // 2
        left = (w - min_dim) // 2

        return image[top:top+min_dim, left:left+min_dim]

    def preprocess_image(self, cv_image):
        """
        Preprocess image for PlaceNet model
        Args:
            cv_image: OpenCV image (BGR)
        Returns:
            Preprocessed tensor ready for model input
        """
        # Convert BGR to RGB
        rgb_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)

        # Center crop to square
        square_image = self.center_crop_square(rgb_image)

        # Resize to target size
        resized = cv2.resize(square_image, (self.image_size, self.image_size))

        # Convert to float and normalize to [0, 1]
        image_float = resized.astype(np.float32) / 255.0

        # Apply ImageNet normalization
        normalized = (image_float - self.mean) / self.std

        # Convert to tensor: (H, W, C) -> (C, H, W)
        tensor = torch.from_numpy(normalized).permute(2, 0, 1)

        # Add batch dimension: (C, H, W) -> (1, C, H, W)
        tensor = tensor.unsqueeze(0)

        return tensor.to(self.device)

    def extract_feature(self, image_tensor):
        """
        Extract feature vector from image tensor
        Args:
            image_tensor: Preprocessed image tensor
        Returns:
            Feature vector as numpy array
        """
        with torch.no_grad():
            feature = self.model(image_tensor)
            # Convert to CPU and numpy
            feature_np = feature.cpu().squeeze().numpy()

        return feature_np

    def image_callback(self, msg):
        """
        Callback for incoming camera images
        """
        try:
            # Convert ROS Image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Preprocess
            image_tensor = self.preprocess_image(cv_image)

            # Extract feature
            feature = self.extract_feature(image_tensor)

            # Publish feature
            feature_msg = Float32MultiArray()
            feature_msg.data = feature.tolist()
            self.feature_pub.publish(feature_msg)

        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')


def main(args=None):
    rclpy.init(args=args)

    try:
        node = FeatureExtractor()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error in feature_extractor: {e}')
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
