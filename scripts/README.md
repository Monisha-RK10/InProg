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

### **yolo_detector_node.py**

YOLO node handles detection + 3D fusion (perception)

- Subscribes to:
  - `/camera/left/image_raw` (for detection)
  - `/stereo/points_3d_dense` (it has X, Y, Z information)

- Processes:
  - Runs YOLOv8 on the left image to detect objects (cars, persons, etc.)
  - Maps COCO classes (like car, person) to KITTI classes (Car, Cyclist), focusing only on relevant classes
  - Computes 2D bounding box center for each detection
  - Looks up corresponding (X, Y, Z) from `point_3d` dense for either center or 3x3 median patch
  - Filters out detections with invalid or too far depth (Z=0 or Z < threshold or z > 80)

- Publishes:
  - `/yolo3d_detections`: list of `[class, X, Y, Z]`

---
