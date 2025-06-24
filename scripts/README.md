# Scripts/

## Files

### **stereo_image_publisher.py**

Publishes KITTI stereo images as ROS 2 topics.

- Publishes: 
  - `/camera/left/image_raw`
  - `/camera/right/image_raw`

---

### **stereo_depth_node.py**

- Subscribes to:
  - `/camera/left/image_raw`
  - `/camera/right/image_raw`

- Processes:
  - Computes disparity map using OpenCV’s StereoBM or StereoSGBM
  - Applies camera calibration (fx, baseline, cx, cy) to compute depth
  - Reprojects disparity into 3D using the Q matrix

- Publishes:
  - `/stereo/disparity_image` – Normalized 8-bit disparity image for RViz and debugging
  - `/stereo/points_3d` – 3D point map for RViz (sparse points)
  - `/stereo/points_3d_dense` – Dense 3D image for fusion with YOLO
---

### **object_fusion_warning_node.py**

Handles detection, 3D fusion, and warning generation in one node.

- Subscribes to:
  - `/camera/left/image_raw` – for YOLO object detection
  - `/stereo/points_3d_dense` – for (X, Y, Z) lookup of detected objects

- Processes:
  - YOLOv8 detection on left camera image (car, person -> mapped to KITTI classes)
  - For each 2D bounding box, extracts 3D coordinates using center pixel or 3×3 patch median
  - Filters out invalid or far detections (Z <= 0 or Z > 80)
  - Displays bounding boxes and 3D coordinates on the image
  - **Triggers a warning** (`ACHTUNG!!!`) overlay when object is within < 8 meters
  - Uses color-coded boxes (Red = near, Yellow = mid, Green = far)

- Visualization:
  - Live annotated detection image with 3D info (`cv2.imshow(...)`)

---

### **full_pipeline.launch.py**

`ros2 launch perception_pipeline full_pipeline.launch.py`

This single line will:

- Start stereo_image_publisher
- Start stereo_depth_node
- Start object_fusion_warning_node

and run them all together via ROS 2 launch

---
