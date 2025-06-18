# ROS 2 Workspace: perception_pipeline

### Workspace Structure:
```bash

 ros2_ws/
 ├── src/
 │   └── perception_pipeline/
 │       ├── detection_node/
 │       │   └── detection_node.py
 │       ├── stereo_depth_node/
 │       │   └── stereo_depth_node.py
 │       ├── pointcloud_node/
 │       │   └── pointcloud_node.py
 │       ├── tracking_node/
 │       │   └── tracking_node.py
 │       └── launch/
 │           └── perception_launch.py

Pedestrian at (X=1.56, Y=0.59, Z=5.42) → Close, slightly right of center
Pedestrian at (X=6.34, Y=0.65, Z=8.61) → Further right and farther away
Car at (X=-4.86, Y=0.76, Z=14.27) → Left and far
Car at (X=2.98, Y=0.72, Z=20.30) → Right and very far

Z-values from ~5m to ~20m for pedestrians and cars are consistent with typical street scenes.

Y-values ~0.5–0.8m make sense for the bottom center of bounding boxes (feet on ground).

X-values show objects are across the field of view.

Timing & Performance
Speed: 4.9ms preprocess, 116.4ms inference, 10.5ms postprocess
Speed: 1.2ms preprocess, 43.4ms inference, 0.8ms postprocess

The first frame had more objects (4 cars, 4 bikes) —> hence higher inference time.
The second had fewer objects —> so inference dropped to ~43 ms.



Since the camera is mounted at ~1.2–1.6m height (typical for KITTI), this means the ground point will project to Y ≈ 0.5–0.8 meters in the 3D frame (a bit under the camera).

In KITTI and most robotics vision setups (like ROS and OpenCV camera frames):

X-axis: Left/Right (positive = right, negative = left)

Y-axis: Vertical, up/down (positive = up)

Z-axis: Forward/Depth (distance from the camera, always positive)

So the coordinate (X=1.56, Y=0.59, Z=5.42) means:

The object is 1.56 m to the right of the camera center (X)

0.59 m above the camera’s optical center (Y)

5.42 m in front of the camera (Z)

color for detection, grayscale for stereo.
