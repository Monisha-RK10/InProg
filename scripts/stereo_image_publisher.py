import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
import time

class StereoImagePublisher(Node):
    def __init__(self):
        super().__init__('stereo_image_publisher')
        self.left_pub = self.create_publisher(Image, '/camera/left/image_raw', 10)
        self.right_pub = self.create_publisher(Image, '/camera/right/image_raw', 10)

        self.bridge = CvBridge()
        self.left_dir = '/mnt/d/MID-APRIL/SOTA/Projects/Project7/KITTI_3D/left_images/0000'
        self.right_dir = '/mnt/d/MID-APRIL/SOTA/Projects/Project7/KITTI_3D/right_images/0000'
        self.image_filenames = sorted(os.listdir(self.left_dir))
        self.timer = self.create_timer(0.5, self.timer_callback)  # 2 fps
        self.index = 0

    def timer_callback(self):
        if self.index >= len(self.image_filenames):
            self.get_logger().info('All images published.')
            return

        left_img_path = os.path.join(self.left_dir, self.image_filenames[self.index])
        right_img_path = os.path.join(self.right_dir, self.image_filenames[self.index])

        left_img = cv2.imread(left_img_path)
        right_img = cv2.imread(right_img_path)

        if left_img is None or right_img is None:
            self.get_logger().warn(f'Could not read {left_img_path} or {right_img_path}')
            self.index += 1
            return

        left_msg = self.bridge.cv2_to_imgmsg(left_img, encoding='bgr8')
        right_msg = self.bridge.cv2_to_imgmsg(right_img, encoding='bgr8')

        self.left_pub.publish(left_msg)
        self.right_pub.publish(right_msg)

        self.get_logger().info(f'Published frame {self.index}')
        self.index += 1

def main(args=None):
    rclpy.init(args=args)
    node = StereoImagePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
