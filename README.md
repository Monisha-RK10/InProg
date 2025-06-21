# Real-Time 3D-Aware Object Perception and Proximity Warning System

> Note: Person is mapped to Cyclist for visual purpose. This is fine for now because this project does not consider evaluation with MOT/IoU. In case,you need better mapping technique, please consider my earlier project [Real-Time Multi-Object Tracking on KITTI with ROS 2, YOLOv8 & BYTETrack] (https://github.com/Monisha-RK10/Real-Time-Multi-Object-Tracking-on-KITTI-with-ROS-2-YOLOv8-BYTETrack), where I implemented cyclist filter.

## Pipeline Overview

**stereo_image_publisher.py**
Publishes:

- /camera/left/image_raw
- /camera/right/image_raw

**stereo_depth_node.py**
Subscribes to:

- /camera/left/image_raw
- /camera/right/image_raw

And publishes:

- /stereo/disparity
- /stereo/depth_map
- /stereo/3d_warning (optional)
