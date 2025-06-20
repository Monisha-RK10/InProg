### Understanding How StereoSGBM Work

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

     - speckleWindowSize=100 → the 16-pixel patch is too small
     - speckleRange=32 → the difference from 50 to 90 is too large
    
    → This patch is removed (set to -1 in disparity map).
