import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge
from ultralytics import YOLO
import numpy as np
import cv2


class YoloDepthFusionNode(Node):
    def __init__(self):
        super().__init__('yolo_depth_fusion_node')

        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(Image, '/camera/left/image_raw', self.image_callback, 10)
        self.depth_sub = self.create_subscription(Image, '/stereo/points_3d_dense', self.depth_callback, 10)

        self.detection_pub = self.create_publisher(PointStamped, '/yolo3d_detections', 10)

        self.latest_image = None
        self.latest_depth = None

        self.model = YOLO('/mnt/d/MID-APRIL/SOTA/Projects/Project7/yolo_models/yolov8n.pt')
        self.class_map = {'person': 'Cyclist', 'car': 'Car'}

        self.confidence_threshold = 0.3
        self.use_patch = True                                                 # Toggle: True = use 3x3 patch, False = use center pixel only

    def image_callback(self, msg):
        self.latest_image = msg
        self.try_infer()

    def depth_callback(self, msg):
        self.latest_depth = msg
        self.try_infer()

    def try_infer(self):
        if self.latest_image is None or self.latest_depth is None:
            return

        img = self.bridge.imgmsg_to_cv2(self.latest_image, 'bgr8')
        depth = self.bridge.imgmsg_to_cv2(self.latest_depth, desired_encoding='32FC3')

        if depth.ndim != 3 or depth.shape[2] != 3:
            self.get_logger().warn("Invalid depth shape. Expected (H, W, 3)")
            return

        results = self.model(img)[0]

        coco_to_kitti = {
            'car': 'Car',
            'person': 'Cyclist',
            #'bicycle': 'Cyclist'
        }

        for det in results.boxes:
            if det.conf[0] < self.confidence_threshold:
                continue

            x1, y1, x2, y2 = map(int, det.xyxy[0])
            cls_id = int(det.cls[0])
            cls_name = self.model.names[cls_id]

            # Only consider relevant classes
            if cls_name not in coco_to_kitti:
                continue

            kitti_cls_name = coco_to_kitti[cls_name]


            u = int((x1 + x2) / 2)
            v = int((y1 + y2) / 2)

            if v < 1 or v >= depth.shape[0] - 1 or u < 1 or u >= depth.shape[1] - 1:
                continue

            if self.use_patch:
                patch = depth[v - 1:v + 2, u - 1:u + 2, :]
                patch = patch.reshape(-1, 3)
                patch = patch[np.all(np.isfinite(patch), axis=1)]
                if len(patch) == 0:
                    continue
                X, Y, Z = np.median(patch, axis=0)
            else:
                X, Y, Z = depth[v, u]
                if not np.isfinite(Z) or Z <= 0:
                    continue

            # Filter out too far/invalid points
            if Z > 80:
                continue

            self.get_logger().info(
                f"[YOLO3D] {kitti_cls_name} at X={X:.2f}m, Y={Y:.2f}m, Z={Z:.2f}m"
            )

            pt = PointStamped()
            pt.header = self.latest_image.header
            pt.point.x = float(X)
            pt.point.y = float(Y)
            pt.point.z = float(Z)
            # If needed, attach class info using a custom msg
            self.detection_pub.publish(pt)

        # Reset
        self.latest_image = None
        self.latest_depth = None


def main(args=None):
    rclpy.init(args=args)
    node = YoloDepthFusionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
