### Understanding How StereoSGBM Work

StereoSGBM works by comparing patches (blocks of pixels) between the left and right image. It tries to find the best horizontal shift (disparity) where the two patches match best.

- If a block around pixel (x, y) in the left image matches best with a block around (x–d, y) in the right image, then disparity = d for that pixel.
- It does this for every pixel or small region.

**blockSize**

> For example, consider the size of the window i.e., blockSize=5, then a 5×5 pixel patch centered at (x, y) in the left image is compared with a 5×5 patch at (x–d, y) in the right image for all d ∈ [minDisparity, minDisparity + numDisparities).
- Smaller blockSize → can detect fine details (like poles, wires), but more sensitive to noise.
- Larger blockSize → smoother results, but fine structures may be lost.
- It is not the range of search, but the size of what we are comparing at each pixel.

**numDisparities**

Let’s say:
- minDisparity = 0,
- numDisparities = 128
- Then, for every pixel in the left image, the matcher looks in the right image at x - d, where d ∈ [0, 128).

> numDisparities is like "How far are we searching?" This is where the search range comes in. It defines how many shifts (in pixels) we try to find a good match for each block. In a way, we are saying "Let’s try matching this 5×5 patch in the left image against patches in the right image up to 128 pixels leftward."

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
 - Depth = f.B/Disparity = 700 * 0.54/128 = 2.95 (approx.) Closest depth it can measure is ~3 meters.
 - Covers the useful range of 3 to 50 meters, which is ideal for KITTI objects like cars and pedestrians.
> Note: In theory, we can get 378 m (with disparity = 1), however, in practice, anything beyond 50–70 m is unreliable with standard 1242×375 KITTI images.
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
