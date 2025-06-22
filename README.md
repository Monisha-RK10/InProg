# Real-Time 3D-Aware Object Perception and Proximity Warning using Stereo Vision and YOLOv8
# This Project is in Progress

This project demonstrates a real-time pipeline for 3D-aware object detection by fusing stereo vision depth estimation with YOLOv8 2D detections, and publishing proximity alerts based on the reconstructed 3D coordinates of dynamic objects such as cars and cyclists.

The entire pipeline is built using ROS 2, runs on KITTI stereo image pairs, and leverages camera calibration files to convert disparity into real-world 3D coordinates.

This project forms a key perception module useful in autonomous driving, robot navigation, and intelligent warning systems.

## Learn the Theory (Optional)

To deeply understand how camera geometry and 3D localization work, refer to my Medium series:

[CV-8 (Camera Calibration, Extracting Intrinsic, Extrinsic, & Distortion Coefficients)](https://medium.com/@monishatemp20/cv-8-camera-calibration-extracting-intrinsic-extrinsic-distortion-coefficients-64c0bd756c7c)

[CV-10 (3D World Coordinate System to 2D Image Coordinate System)](https://medium.com/@monishatemp20/cv-10-3d-world-coordinate-system-to-2d-image-coordinate-system-d6c6faec353d)

[CV-14 (Visual Odometry: Epipolar Geometry — Part 1)](https://medium.com/@monishatemp20/cv-14-visual-odometry-epipolar-geometry-part-1-ffe06a35fa81)

> Note: Person (COCO) is mapped to Cyclist (KITTI) for visual purpose. This is fine for now because this project does not consider evaluation with MOT/IoU.
>
> For better class mapping technique, please refer to my earlier project [Real-Time Multi-Object Tracking on KITTI with ROS 2, YOLOv8 & BYTETrack](https://github.com/Monisha-RK10/Real-Time-Multi-Object-Tracking-on-KITTI-with-ROS-2-YOLOv8-BYTETrack), where I implemented cyclist filter.

## Pipeline Overview

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
  - `/stereo/disparity` – color-mapped disparity image
  - `/stereo/depth_map` – per-pixel depth (in meters)
  - `/stereo/point_3d` – 3D point map (optional for internal use)

---

### **yolo_detector_node.py**

- Subscribes to:
  - `/camera/left/image_raw` (for detection)
  - `/stereo/point_3d` (optional: for real-world depth lookup)

- Processes:
  - Runs YOLOv8 on the left image to detect objects (cars, pedestrians, etc.)
  - Computes 2D bounding box center for each detection
  - Looks up corresponding (X, Y, Z) from point_3d map
  - Filters out detections with invalid depth (Z=0 or Z < threshold)

- Publishes:
  - `/detected_objects_3d`: list of `[class, confidence, X, Y, Z]`

---

### **warning_node.py**

- Subscribes to:
  - `/detected_objects_3d`

- Processes: 
  - Checks 3D distances (Z coordinate) against safety thresholds
  - Triggers proximity alerts based on object type (e.g., person < 3m, car < 5m)

- Publishes:
  - `/proximity_alerts` or `/3d_warning`: visual markers or text alerts (e.g., for RViz or log display)

## Sample Output Visualizations

## Intermediate Output on Rviz (Disparity)

<img src="output/disparity.png" width="500"/>
<img src="output/disparity_image.png" width="500"/>
