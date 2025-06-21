# Real-Time 3D-Aware Object Perception and Proximity Warning System

For understanding 3D geometry, intrinsic/extrinsic properties of camera, epipolar geometry, calibration,, read the below articles on Medium: 
[CV-14 (Visual Odometry: Epipolar Geometry — Part 1)](https://medium.com/@monishatemp20/cv-14-visual-odometry-epipolar-geometry-part-1-ffe06a35fa81)
[CV-8 (Camera Calibration, Extracting Intrinsic, Extrinsic, & Distortion Coefficients)] (https://medium.com/@monishatemp20/cv-8-camera-calibration-extracting-intrinsic-extrinsic-distortion-coefficients-64c0bd756c7c)
[CV-10 (3D World Coordinate System to 2D Image Coordinate System)](https://medium.com/@monishatemp20/cv-10-3d-world-coordinate-system-to-2d-image-coordinate-system-d6c6faec353d)


> Note: Person is mapped to Cyclist for visual purpose. This is fine for now because this project does not consider evaluation with MOT/IoU. In case,you need better mapping technique, please consider my earlier project [Real-Time Multi-Object Tracking on KITTI with ROS 2, YOLOv8 & BYTETrack] (https://github.com/Monisha-RK10/Real-Time-Multi-Object-Tracking-on-KITTI-with-ROS-2-YOLOv8-BYTETrack), where I implemented cyclist filter.

## Pipeline Overview

**stereo_image_publisher.py**

- Publishes:

  - /camera/left/image_raw
  - /camera/right/image_raw

**stereo_depth_node.py**

- Subscribes to:

  - /camera/left/image_raw
  - /camera/right/image_raw

- And publishes:

  - /stereo/disparity
  - /stereo/depth_map
  - /stereo/3d_warning (optional)
