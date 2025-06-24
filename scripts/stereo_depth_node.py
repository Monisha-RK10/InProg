# Step 2: This code does the following:
# Subscription: Subscribe to KITTI stereo images.
# Stereo Setup: Set Stereo parameters such as minDisparity, numDisparities, blockSize, P1, P2, speckleWindowSize, etc.
# fx and Baseline:
# Extracts fx from P2 using KITTI calib file. 
# Compute baseline (B) using fx and Tx left & right. 
# Disparity Computation: Compute disparity using Stereo parameters, left, and right gray images.
# Depth Map Computation: Compute depth map using fx, baseline, and disparity.
# Q Matrix Construction: Use images's center, baseline, and fx to construct Q. It reprojects disparity into 3D using known camera parameters.
# 3D Points Reconstruction (points_3D): Use disparity and Q to compute dense 3D points of shape (H, W, 3) with X, Y, Z coordinates.
# Publish PointCloud2 for RViz: Generate a sparse point cloud by filtering points_3D i.e., only where disparity is valid with shape (N, 3).
# Publish Dense 3D Image for YOLO Fusion: Keeps the shape (H, W, 3) to enable 2D detectors (YOLO) to lookup 3D coordinates using [u, v] pixel positions.
# Publish Disparity for RViz

# Note: 
# Projection matrices P2, P3: Project a 3D world point X = [x, y, z, 1]T into 2D image point using pixel = P.X
# Tx in projection matrix is in pixel units. Dividing by fx gives real-world translation (in meters), allowing baseline computation.
# Only the horizontal translation (Tx) matters for depth.
# StereoSGBM assumes epipolar geometry is handled in preprocessing (rectification).
# Rotation matrices are already absorbed into the rectification process.
# Epipolar lines are horizontal. Cameras appear to be aligned (no rotation between them)

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2                                                 # ROS topics
from std_msgs.msg import Header
from cv_bridge import CvBridge
import numpy as np
import cv2
import sensor_msgs_py.point_cloud2 as pc2                                                      # Helper functions to manipulate PointCloud2 data in Python.

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
        self.pub_points3d = self.create_publisher(PointCloud2, '/stereo/points_3d', 10)        # Publishes actual 3D coordinates (X, Y, Z) in ROS standard point cloud format
        self.pub_points3d_dense = self.create_publisher(Image, '/stereo/points_3d_dense', 10)  # Fake 3D image (e.g., each pixel stores a Z-depth or packed XYZ)

        # Parameters
        self.fx, self.baseline = self.parse_calibration('/mnt/d/MID-APRIL/SOTA/Projects/Project7/KITTI_3D/calib/0000.txt')
        self.get_logger().info(f"Parsed fx={self.fx:.2f}, baseline={self.baseline:.4f} m")

        # Stereo matcher
        self.stereo = cv2.StereoSGBM_create(
            minDisparity=0,                                                                    # Start matching from 0-pixel shift (standard for rectified KITTI).
            numDisparities=128,                                                                # Max disparity range to search, must be divisible by 16. Bigger = more depth range, slower.
            blockSize=5,                                                                       # Size of matching window (odd number). Small -> sharp, sensitive.
            P1=8 * 3 * 5 ** 2,                                                                 # Penalty for small disparity changes (smoother surfaces).
            P2=32 * 3 * 5 ** 2,                                                                # Penalty for larger disparity jumps (object boundaries). P2 > P1
            disp12MaxDiff=1,                                                                   # Check consistency between left -> right and right -> left disparity maps.
            uniquenessRatio=10,                                                                # Reject matches too close in score to next-best match (helps accuracy).
            speckleWindowSize=100,                                                             # Remove small isolated blobs (noise) in disparity map.
            speckleRange=32                                                                    # Max disparity variation within that speckle window.
        )
        
        self.get_logger().info("Stereo depth node started.")

    def parse_calibration(self, path):
        with open(path, 'r') as f:
            lines = f.readlines()

        P2 = [float(val) for val in lines[2].split()[1:]]                                      # Skips the first token (which is "P2:") using [1:]
        P3 = [float(val) for val in lines[3].split()[1:]]
        fx = P2[0]
        Tx_left = P2[3] / fx                                                                   # Pixel units to meters
        Tx_right = P3[3] / fx                                                                  # Full translation vector is (Tx, Ty, Tz), but in stereo rectified setup, all the translation is along X-axis only.
        baseline = abs(Tx_right - Tx_left)                                                     # Usually Tx_left ≈ 0 and Tx_right ≈ -0.54 m, baseline = 0.54 m
        return fx, baseline

    def left_callback(self, msg):
        self.left_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')              # Left image: gray for disparity, color for detection 
        self.process_stereo_pair()

    def right_callback(self, msg):
        self.right_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')            # Right image: gray for disparity
        self.process_stereo_pair()

    def process_stereo_pair(self):
        if self.left_image is None or self.right_image is None:
            return

        imgL_gray = cv2.cvtColor(self.left_image, cv2.COLOR_BGR2GRAY)
        imgR_gray = self.right_image

        header = Header()                                                                      # Where the data is located (coordinate frame, like "camera" or "map").
        header.stamp = self.get_clock().now().to_msg()                                         # When the data was valid (used for synchronization).
        header.frame_id = "camera"

        # Compute disparity
        disparity = self.stereo.compute(imgL_gray, imgR_gray).astype(np.float32) / 16.0        # Image plane: each pixel stores disparity (in pixels). Output shape: (H, W) same as input images. 

        # Compute depth
        depth_map = (self.fx * self.baseline) / (disparity + 1e-6)

        # Reproject to 3D
        h, w = imgL_gray.shape
        cx, cy = w / 2, h / 2
        Q = np.float32([
            [1, 0, 0, -cx],
            [0, 1, 0, -cy],
            [0, 0, 0, self.fx],
            [0, 0, 1 / self.baseline, 0]
        ])

        points_3D = cv2.reprojectImageTo3D(disparity, Q)                                       # Camera coordinate system (wrt left camera's origin): 3D coordinates (X, Y, Z) at every pixel (u, v). Output shape: (H, W, 3)

        # PointCloud2 for RViz
        # Sparse point cloud, only where disparity was valid with shape (N, 3)
        mask = disparity > 0
        points = points_3D[mask]
        cloud_msg = pc2.create_cloud_xyz32(header, points)                                     # create_cloud_xyz32(header, points) internally assigns msg.header = header
        self.pub_points3d.publish(cloud_msg)                                                   # RViz expects an array of 3D points, not an image-shaped tensor.

        # Dense 3D image for fusion with 2D detector
        # Keeps the original image shape (H, W, 3), lookup any pixel coordinate [u, v] directly and get [X, Y, Z]
        points_3D_img = points_3D.astype(np.float32)
        dense_msg = self.bridge.cv2_to_imgmsg(points_3D_img, encoding='32FC3')                 # 3 channels float32, converts the numpy array to a ROS Image message.
        dense_msg.header = header                                                              # Manually assign the header. An empty stamp and frame_id can cause issues for a) Synchronization, b) Frame transforms (e.g., TF), c) Accurate fusion with 2D detections (YOLO)
        self.pub_points3d_dense.publish(dense_msg)                                             # Keep a dense version (shape: H × W × 3) in NumPy format, so that downstream detectors (like YOLO) can look up the 3D position of a bounding box center or any pixel directly

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
