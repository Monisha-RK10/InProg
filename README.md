# Real-Time 3D-Aware Object Perception and Proximity Warning using Stereo Vision and YOLOv8

This project demonstrates a real-time pipeline for 3D-aware object detection by fusing stereo vision depth estimation with YOLOv8 2D detections, and publishing proximity alerts based on the reconstructed 3D coordinates of dynamic objects such as cars and cyclists.

The entire pipeline is built using ROS 2, runs on KITTI stereo image pairs, and leverages camera calibration files to convert disparity into real-world 3D coordinates.

This project forms a key perception module useful in autonomous driving, robot navigation, and intelligent warning systems.

## Learn the Theory (Optional)

To deeply understand how camera geometry and 3D localization work, refer to my Medium series:

[CV-8 (Camera Calibration, Extracting Intrinsic, Extrinsic, & Distortion Coefficients)](https://medium.com/@monishatemp20/cv-8-camera-calibration-extracting-intrinsic-extrinsic-distortion-coefficients-64c0bd756c7c)

[CV-10 (3D World Coordinate System to 2D Image Coordinate System)](https://medium.com/@monishatemp20/cv-10-3d-world-coordinate-system-to-2d-image-coordinate-system-d6c6faec353d)

[CV-14 (Visual Odometry: Epipolar Geometry — Part 1)](https://medium.com/@monishatemp20/cv-14-visual-odometry-epipolar-geometry-part-1-ffe06a35fa81)

> Note: Person (coco) is mapped to Cyclist (kitti) for visual purpose. This is fine for now because this project does not consider evaluation with MOT/IoU.
>
> In case,you need better mapping technique, please consider my earlier project [Real-Time Multi-Object Tracking on KITTI with ROS 2, YOLOv8 & BYTETrack] (https://github.com/Monisha-RK10/Real-Time-Multi-Object-Tracking-on-KITTI-with-ROS-2-YOLOv8-BYTETrack), where I implemented cyclist filter.

---
## Pipeline Overview

### **stereo_image_publisher.py**

KITTI stereo pairs are broadcasted as ROS image topics

- Publishes: 

  - `/camera/left/image_raw`
  - `/camera/right/image_raw`

### **stereo_depth_node.py**

- Subscribes to:

  - `/camera/left/image_raw`
  - `/camera/right/image_raw`

- Processes:

  - Computes disparity map using OpenCV’s block matcher
  - Uses camera calibration to compute depth map
  - Constructs 3D point cloud using the Q matrix

- Publishes:

  - `/stereo/disparity`– visual disparity map
  - `/stereo/depth_map` – floating-point depth values

### **yolo_detector_node.py**

- Subscribes to:

  - `/camera/left/image_raw (for object detection)`

- Processes:

  - Runs YOLOv8 detection on the left image
  - For each detection, extracts 2D center point
  - Queries point_3d map to get (X, Y, Z) real-world position
  - Filters invalid depth (Z=0 or below threshold)

- Publishes:

  - `/detected_objects_3d`: list of object classes with 3D coordinates

### **warning_node.py**

- Subscribes to:
  - `/detected_objects_3d` (YOLO detections)
  - `/stereo/depth_map` (3D point map)

- Processes: 
  - Extract 3D positions of detected object centers
  - Apply thresholding to trigger proximity warnings
  
- Publishes:

  - `/3d_warning` or `/proximity_alerts`
---
