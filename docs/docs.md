## Useful Concepts to Understand the Project Flow

- **Focal Length (f) & Baseline (b)** - These are useful for computing depth, 3D reconstruction.
- **Stereo SGBM & its Parameters to compute Disparity** - This is used for computing disparity (d).
- **Computing Depth & full 3D Manually** - Using f, b, and d.
- **Computing 3D** - Using cv2.reprojectImageTo3D(disparity, Q). Skips the calculation of depth & full 3D manually.
- **Disparity vs Depth vs 3D (Sparse + Dense)** - To understand if we are in image plane, camera plane, or world frame.
- **Coordinate Frame in the Pipeline** - To understand at what stage, we use intrinsic & extrinsic properties of camera.
- **Center Pixel vs 3x3 Median Patch** - Two different ways to consider depth for the detected object.

---

### Focal Length (f) & Baseline (b)

| Term                          | Meaning                                    | Unit                        |
| ----------------------------- | ------------------------------------------ | --------------------------- |
| **Focal length** `f`          | Distance from camera center to image plane | pixels (in computer vision) |
| **Image center** `(c_x, c_y)` | Where optical axis hits image plane        | pixels                      |
| **Baseline** `b`              | Distance between left/right camera centers | meters or mm                |

---

### Stereo SGBM & its Parameters to compute Disparity

StereoSGBM works by comparing patches (blocks of pixels) between the left and right image. It tries to find the best horizontal shift (disparity) where the two patches match best.

- If a block around pixel (x, y) in the left image matches best with a block around (x–d, y) in the right image, then disparity = d for that pixel.
- It does this for every pixel or small region.

### Stereo SGBM Parameters

| Parameter           | Role                              | Value      | Typical Range    |
| ------------------- | --------------------------------- | ---------- | ---------------- |
| `minDisparity`      | Start of disparity search         | `0`        | Usually `0`      |
| `numDisparities`    | Search range (multiple of 16)     | `128`      | `64–256`         |
| `blockSize`         | Matching block size               | `5`        | `3–11`           |
| `P1`                | Small disparity penalty           | `600`      | Based on formula |
| `P2`                | Large disparity penalty           | `2400`     | Based on formula |
| `disp12MaxDiff`     | Left-right match tolerance        | `1`        | `1–2`            |
| `uniquenessRatio`   | Second-best match rejection       | `10`       | `10–20`          |
| `speckleWindowSize` | Minimum speckle region size       | `100`      | `50–200`         |
| `speckleRange`      | Allowed disparity jump in speckle | `32`       | `1–64`           |

### Values Set in this Project

 `minDisparity = 0` 
 
- Start matching from 0-pixel shift (standard for rectified KITTI)
- Disparity is small for far objects
 
 `numDisparities=128`
 
 - Max disparity range to search, must be divisible by 16.
 - Bigger = more depth range, slower.
 - 128 pixels means we search from 0 to 127 pixel shifts.
 - Depth = fx.B/Disparity = 700 * 0.54/128 = 2.95 (approx.) where fx=721.54, B=0.5327 m for KITTI. Closest depth it can measure is ~3 meters.
 - Covers the useful range of 3 to 50 meters, which is ideal for KITTI objects like cars and pedestrians.
   
> Note: In theory, we can get 378 m (with disparity = 1), however, in practice, anything beyond 50–70 m is unreliable with standard 1242×375 KITTI images. Image resolution matters because more pixels = more detail = better matching at a distance.
>
> At large depths, disparity becomes very small.
> E.g., at 50 m → disparity ~7.5 px, at 100 m → disparity ~3.78 px, Stereo matchers like SGBM have trouble detecting disparities that small accurately (sub-pixel errors dominate, noise increases).

 `blockSize=5`             
 
 - Size of matching window
 - Always odd number
 - Small -> sharp, sensitive
 
  `P1=8 * 3 * 5**2 = 600`
  
  - Penalty for small disparity changes (smoother surfaces).
  
  `P2=32 * 3 * 5**2 = 2400`
  
  - Penalty for larger disparity jumps (object boundaries).
  - P2 > P1
            
  `disp12MaxDiff=1`
  
  - Check consistency between left -> right and right -> left disparity maps.
  - Helps reject occlusions and mismatches.
  
  `uniquenessRatio=10`
  
  - Reject matches too close in score to next-best match (helps accuracy).
  - Ensures the best match is significantly better than the second-best.   
  
  `speckleWindowSize=100`
  
  - Remove small isolated blobs (noise) in disparity map.
            
  `speckleRange=32`
  
  - Max disparity variation within that speckle window.

    Imagine a region where most pixels have disparity 50, and suddenly a small 4x4 patch (16 pixels) has disparity 90.
    With:

     - speckleWindowSize=100 -> the 16-pixel patch is too small
     - speckleRange=32 -> the difference from 50 to 90 is too large
    
    This patch is removed (set to -1 in disparity map).
---

### Computing Depth & full 3D Manually 
> Note: We can skip this step (calculating depth & full 3D manually), as OpenCV supports `cv2.reprojectImageTo3D(disparity, Q)`.

Depth Calculation: `Z = (f.b)/d`, where f is focal length, b is baseline, and d is disparity

So depth is accurate when you know:

- How zoomed-in the cameras are (`focal length`)
- How far apart the cameras are (`baseline`)
- How much the object shifted between left/right (`disparity`)

Once depth (Z) is calculated, the full 3D point in left camera frame is

 `X = (u-cx).Z/f`, 
 
 `Y = (u-cy).Z/f`

where `(u, v)`: pixel, `(cx, cy)`: principle point x, y, `Z`: depth, and `f`: focal length

---

### Disparity vs Depth vs 3D (Sparse + Dense)

| Stage            | Output                   | Shape     | Masked? | Plane         | ROS Format                |
| ---------------- | ------------------------ | --------- | ------- | ------------- | ------------------------- |
| Disparity        | Disparity map            | (H, W)    | –       | Image plane   | `mono16` or `float32`     |
| Depth            | Depth map                | (H, W)    | –       | Camera plane  | Optional                  |
| 3D Sparse (RViz) | `points_3D[mask]`        | (N, 3)    |  Yes    | Camera coords | `sensor_msgs/PointCloud2` |
| 3D Dense         | `cv2.reprojectImageTo3D` | (H, W, 3) |  No     | Camera coords | `32FC3` image             |

---

### Coordinate Frame in the Pipeline

| Stage                                | Coordinate Frame     | Uses Intrinsics? | Uses Extrinsics?  |
| ------------------------------------ | -------------------- | ---------------- | ----------------- |
| Input stereo image                   | Image frame          | –                | –                 |
| Disparity map                        | Image plane (pixels) | –                | –                 |
| RViz point cloud                     | Camera frame         | –                | –                 |
| `cv2.reprojectImageTo3D` → 3D points | **Camera frame**     |   via Q          |   not world frame |
| Warning system                       | Camera-relative      | –                | –                 |

**Coordinate Frame**
- All 3D points are computed in the left camera frame.
- Disparity is computed with respect to the left image (as reference), and the reprojected 3D points (X, Y, Z) represent real-world positions relative to the left camera's optical center.
- Depth Z increases forward, X is right-left, Y is up-down.

---

### Center Pixel vs 3x3 Median Patch (Depth Calculation)

| Aspect                       | **Center Pixel** (`depth[v, u]`)     | **3x3 Median Patch**                              |
| ---------------------------- | ------------------------------------ | ------------------------------------------------- |
| **Speed**                    | Very fast                            | Slightly slower (reads 9 values, filters, median) |
| **Simplicity**               | Easiest to implement                 | Slightly more logic                               |
| **Robustness to Noise**      | Bad: 1 noisy pixel -> bad depth      | Good: median ignores outliers                     |
| **Handling Missing/Invalid** | Fails if center is NaN               | Patch gives more chance to find valid data        |
| **Edge/Corners**             | Can access invalid or missing values | Can also be tricky, especially at borders         |

**When to Use**

**Center Pixel**
- When depth image is very clean (e.g., synthetic or high-quality sensor)
- If real-time speed and simplicity are priority

**Median Patch**
- For Real-world stereo depth, where noise, outliers, or NaNs are common
- When Object borders may have broken depth -> median helps
- For robust 3D position for downstream logic (e.g., warnings, planning)

---
