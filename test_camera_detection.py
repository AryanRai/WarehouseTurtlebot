#!/usr/bin/env python3
"""
Simple test script to publish test camera images and AprilTag detections
to verify the colour detector is working correctly.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from apriltag_msgs.msg import AprilTagDetectionArray, AprilTagDetection
from geometry_msgs.msg import Point
import cv2
import numpy as np
from cv_bridge import CvBridge
import time

class TestPublisher(Node):
    def __init__(self):
        super().__init__('test_publisher')
        
        # Publishers
        self.image_pub = self.create_publisher(Image, '/camera/unified', 10)
        self.tag_pub = self.create_publisher(AprilTagDetectionArray, '/apriltag/detections', 10)
        
        # Bridge for image conversion
        self.bridge = CvBridge()
        
        # Timer to publish test data
        self.timer = self.create_timer(2.0, self.publish_test_data)
        
        self.get_logger().info("Test publisher started")
        
    def create_test_image(self):
        """Create a test image with colored regions"""
        img = np.zeros((480, 640, 3), dtype=np.uint8)
        
        # Add colored patches
        img[100:200, 100:200] = [0, 255, 0]  # Green patch (mould)
        img[100:200, 300:400] = [255, 0, 0]  # Blue patch (water) - note BGR format
        img[100:200, 500:600] = [0, 0, 255]  # Red patch (blood)
        
        # Add some AprilTag-like square in center
        cv2.rectangle(img, (290, 230), (350, 290), (255, 255, 255), 2)
        cv2.putText(img, "Tag", (300, 270), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        
        return img
        
    def publish_test_data(self):
        """Publish test image and AprilTag detection"""
        
        # Create and publish test image
        test_img = self.create_test_image()
        img_msg = self.bridge.cv2_to_imgmsg(test_img, encoding='bgr8')
        img_msg.header.stamp = self.get_clock().now().to_msg()
        img_msg.header.frame_id = "camera_frame"
        
        self.image_pub.publish(img_msg)
        
        # Create and publish test AprilTag detection
        tag_array = AprilTagDetectionArray()
        tag_array.header.stamp = self.get_clock().now().to_msg()
        tag_array.header.frame_id = "camera_frame"
        
        # Create a test detection
        detection = AprilTagDetection()
        detection.id = 0
        detection.family = "tag36h11"
        
        # Set center point
        detection.centre.x = 320.0  # Center of image
        detection.centre.y = 260.0
        
        # Set corners (approximate square around center)
        corners = [
            Point(x=290.0, y=230.0, z=0.0),  # Top-left
            Point(x=350.0, y=230.0, z=0.0),  # Top-right
            Point(x=350.0, y=290.0, z=0.0),  # Bottom-right
            Point(x=290.0, y=290.0, z=0.0),  # Bottom-left
        ]
        detection.corners = corners
        
        tag_array.detections = [detection]
        self.tag_pub.publish(tag_array)
        
        self.get_logger().info("Published test image and AprilTag detection")

def main():
    rclpy.init()
    node = TestPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
