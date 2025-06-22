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
  - `/stereo/disparity_image` – color-mapped disparity image
  - `/stereo/points_3d` – 3D point map for Rviz (optional for internal use)
  - `/stereo/points_3d_dense` – 3D point map for YOLO dteections
---
