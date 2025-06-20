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
 
 - Max disparity range to search, must be divisible by 16
 - Bigger = more depth range, slower
 - 128 pixels means we search from 0 to 127 pixel shifts     
 
 `blockSize=5`             
 
 - Size of matching window
 - Always odd number
 - Small -> sharp, sensitive
 
  `P1=8 * 3 * 5**2 = 600`
  
  - Penalty for small disparity changes (smoother surfaces).
  
  `P2=32 * 3 * 5**2`
  
  - Penalty for larger disparity jumps (object boundaries).
  - P2 > P1
            
  `disp12MaxDiff=1`
  
  - Check consistency between left -> right and right -> left disparity maps.
  - Helps reject occlusions and mismatches
  
  `uniquenessRatio=10`
  
  - Reject matches too close in score to next-best match (helps accuracy). Ensures the best match is significantly better than the second-best   
  
  `speckleWindowSize=100`
  
  - Remove small isolated blobs (noise) in disparity map.
            
  `speckleRange=32`
  
  - Max disparity variation within that speckle window.
