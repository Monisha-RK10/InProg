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
  - `/stereo/disparity_image` – normalized 8-bit disparity image for RViz and debugging
  - `/stereo/points_3d` – 3D point map for RViz (sparse points)
  - `/stereo/points_3d_dense` – Dense 3D image for fusion with YOLO
---
