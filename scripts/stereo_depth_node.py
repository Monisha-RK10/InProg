# Step 2: This code does the following:
# Subscription: Subscribe to KITTI stereo images.
# Stereo setup: Set Stereo parameters such as minDisparity, numDisparities, blockSize, P1, P2, speckleWindowSize, etc.
# fx and baseline:
# Extracts fx from P2 using KITTI calib file. 
# Compute baseline (B) using fx and Tx left & right. Tranform translation matrixes into meters by dividing by focal length in pixels to compute baseline.
# Disparity: Compute disparity using Stereo parameters, left, and right gray images.
# Depth: Compute depth map using fx, baseline, and disparity.
# Q construction: Use images's center, baseline, and fx to construct Q.
# Dense 3D Reconstruction (points_3D): Use disparity and Q to compute dense 3D points.
# PointCloud2 for RViz: Sparse point cloud (points_3D), only where disparity was valid with shape (N, 3).
# Dense 3D image for fusion with 2D detector: Keep the original image shape (H, W, 3) for points_3D for YOLO.
# Publishes point cloud for Rviz.
# Publishes point cloud for YOLO detections.
# Publishes disparity for Rviz.

# Note: 
# Projection matrices P2, P3: Project a 3D world point X = [x, y, z, 1]T into 2D image point using pixel = P.X
# Only the horizontal translation (Tx) matters for depth.
# StereoSGBM assumes epipolar geometry is handled in preprocessing (rectification).
# Rotation matrices are already absorbed into the rectification process.
# Epipolar lines are horizontal. Cameras appear to be aligned (no rotation between them)

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2
from std_msgs.msg import Header
from cv_bridge import CvBridge
import numpy as np
import cv2
import sensor_msgs_py.point_cloud2 as pc2

class StereoDepthNode(Node):
    def __init__(self):
        super().__init__('stereo_depth_node')

        self.bridge = CvBridge()
        self.left_image = None
        self.right_image = None

        # Subscription
        self.left_sub = self.create_subscription(Image, '/camera/left/image_raw', self.left_callback, 10)
        self.right_sub = self.create_subscription(Image, '/camera/right/image_raw', self.right_callback, 10)

        # Publisher
        self.pub_disparity = self.create_publisher(Image, '/stereo/disparity_image', 10)
        self.pub_points3d = self.create_publisher(PointCloud2, '/stereo/points_3d', 10)
        self.pub_points3d_dense = self.create_publisher(Image, '/stereo/points_3d_dense', 10)

        # Parameters
        self.fx, self.baseline = self.parse_calibration('/mnt/d/MID-APRIL/SOTA/Projects/Project7/KITTI_3D/calib/0000.txt')
        self.get_logger().info(f"Parsed fx={self.fx:.2f}, baseline={self.baseline:.4f} m")

        # Stereo matcher
        self.stereo = cv2.StereoSGBM_create(
            minDisparity=0,
            numDisparities=128,
            blockSize=5,
            P1=8 * 3 * 5 ** 2,
            P2=32 * 3 * 5 ** 2,
            disp12MaxDiff=1,
            uniquenessRatio=10,
            speckleWindowSize=100,
            speckleRange=32
        )


        self.get_logger().info("Stereo depth node started.")

    def parse_calibration(self, path):
        with open(path, 'r') as f:
            lines = f.readlines()

        P2 = [float(val) for val in lines[2].split()[1:]]
        P3 = [float(val) for val in lines[3].split()[1:]]
        fx = P2[0]
        Tx_left = P2[3] / fx
        Tx_right = P3[3] / fx
        baseline = abs(Tx_right - Tx_left)
        return fx, baseline

    def left_callback(self, msg):
        self.left_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.process_stereo_pair()

    def right_callback(self, msg):
        self.right_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
        self.process_stereo_pair()

    def process_stereo_pair(self):
        if self.left_image is None or self.right_image is None:
            return

        imgL_gray = cv2.cvtColor(self.left_image, cv2.COLOR_BGR2GRAY)
        imgR_gray = self.right_image

        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "camera"

        # Step 1: Compute disparity
        disparity = self.stereo.compute(imgL_gray, imgR_gray).astype(np.float32) / 16.0   # disparity.shape = (375, 1242)

        # Step 2: Compute depth
        depth_map = (self.fx * self.baseline) / (disparity + 1e-6)

        # Step 3: Reproject to 3D
        h, w = imgL_gray.shape
        cx, cy = w / 2, h / 2
        Q = np.float32([
            [1, 0, 0, -cx],
            [0, 1, 0, -cy],
            [0, 0, 0, self.fx],
            [0, 0, 1 / self.baseline, 0]
        ])

        points_3D = cv2.reprojectImageTo3D(disparity, Q)                                  # points_3D.shape = (375, 1242, 3)

        # PointCloud2 for RViz
        # Sparse point cloud, only where disparity was valid with shape (N, 3)
        mask = disparity > 0
        points = points_3D[mask]
        cloud_msg = pc2.create_cloud_xyz32(header, points)
        self.pub_points3d.publish(cloud_msg)

        # Dense 3D image for fusion with 2D detector
        # Keeps the original image shape (H, W, 3), lookup any pixel coordinate [u, v] directly and get [X, Y, Z]
        points_3D_img = points_3D.astype(np.float32)
        dense_msg = self.bridge.cv2_to_imgmsg(points_3D_img, encoding='32FC3')            # 3 channels float32
        dense_msg.header = header
        self.pub_points3d_dense.publish(dense_msg)

        # Normalize disparity for visualization
        disp_vis = cv2.normalize(disparity, None, 0, 255, cv2.NORM_MINMAX)
        disp_vis = disp_vis.astype(np.uint8)
        disp_msg = self.bridge.cv2_to_imgmsg(disp_vis, encoding='mono8')
        disp_msg.header = header
        self.pub_disparity.publish(disp_msg)

        # Clear images after processing
        self.left_image = None
        self.right_image = None

def main(args=None):
    rclpy.init(args=args)
    node = StereoDepthNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
