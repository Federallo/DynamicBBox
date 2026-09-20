# DynamicBBox
## Overview
This project implements a sequential clustering algorithm for moving point clouds to generate adaptive 3D bounding boxes for vehicle tracking in autonomous driving scenarios. The system processes LiDAR point cloud streams at 10Hz, maintaining object identity across frames through two novel clustering strategies built on top of DBSCAN.\
The main goal was to extend DBSCAN with a seeded, temporal approach for real-time bounding box tracking in a simulated Pittsburgh intersection.
## Problem statement
Autonomous vehicles rely on LiDAR sensors to perceive their 3D surroundings. Point clouds from moving sensors present challenges for object detection and tracking because they require
- Real-time processing (10Hz = 100 ms budget per frame)
- Adaptive bounding boxes that grow/shrink with vehicle motion
- Temporal consistency - maintaining object identity across frames
- Noise robustness - handling sensor artifacts and sparse points
<!---->
These topics are addressed by extending DBSCAN clustering with temporal priors from previous frames.
## Technologies used
| Category    | Technologies |
| -------- | ------- |
| Core Library | [Open3D](https://www.open3d.org/) (3D data processing, clustering, visualization) |
| Simulation | [BlenSor](https://www.blensor.org/) (LiDAR simulation in Blender) |
| Dataset | [Argoverese](http://argoverse.org/) (Pittsburg and Miami driving scenarios) |
| Language | Python 3.x |
| Dependencies | `numpy`, `open3d`, `matplotlib` |
| Hardware (dev) | Intel i5-10600K, 16GB RAM |
| IDE | Neovim |
## Pipeline
### 1. First bounding boxes initialization (Frame 0)
CSV files &#8594; dataset_loader &#8594; PointClud (voxel downsample) &#8594; DBSCAN clustering &#8594; OrientedBoundingBox (OBB) per cluster &#8594; Open3D rendering (green boxes = initial)
### 2. Sequential update (Frame t &#8594; t+1)
For each existing bounding box:
1. **`OBB.get_point_indices_within_bounding_box()`** is used for core points extraction
2. The region is expanded using two alternative strategies:
   - Expand BBox: enlarge OBB by fixed margin, re-query internal points
   - Sphere Bound: create sphere at centroid with radius = max(extent) x expansion_factor
3. A custom DBSCAN algorithm is applied which seeds with core points (label=1) and expands to neighbors (label=-1) via KD-Tree radius search
4. New OBB are created from resulting cluster (red = tracked)
5. A pairwise OBB intersection test is applied to remove smaller volume box, in order to resolve potential overlaps
<!---->
To detect new objects, the application runs Open3D DBSCAN on unassigned points and, for each newly found cluster, a green box is applied to it.\
Finally, all the boxes that are below a minimum point threshold are removed.
### 3. Visualization
From real-time Open3D window, it is shown
- Current frame point cloud
- Previous bboxes (green)
- Expanded regions / spheres
- Updated bboxes (red)
- New detections (green)
## Main clustering strategies
| Strategy | Description | Pros | Cons |
|----------|-------------|------|------|
| **`dbscan_expand_bbox`** | Enlarge OBB by fixed delta, re-query points | Simple, fast, works for smooth motion | May miss points during sharp turns |
| **`dbscan_sphere_bound`** | Sphere at centroid, radius = max(extent) × factor | Better coverage for erratic motion | Slightly more compute, includes more noise |
> Note: both use the same custom DBSCAN core seeded from previous frame's core points.
## Results
### Execution time (avg over 4 frames, 10 Hz)
| Sensors | Expand BBox (s) | Sphere Bound (s) |
|---------|-----------------|------------------|
| 1       | 0.032           | 0.034            |
| 2       | 0.074           | 0.082            |
| 3       | 0.111           | 0.118            |
| 4       | 0.130           | 0.137            |
| 5       | 0.139           | 0.148            |
> Time scales with sensor count (more points = more KD-Tree queries).
### Tracking Precision
- **Ideal conditions** (spaced vehicles, smooth trajectories): both methods produce tight, stable OBBs
- **Challenging scenarios** (abrupt maneuvers, dense traffic): sphere method more robust
### Limitations
1. **CSV I/O bottleneck** - 11 MB files per frame; physical LiDAR would eliminate this
2. **No motion model** - model don't predict velocity / direction; purely reactive
3. **Dense traffic risk** - pre-labeling all internal points can merge adjacent vehicles
## Future work
- **NumPy vectorization** — replace Python loops with vectorized NumPy operations and batch indexing for memory/speed
- **Motion prediction** - Kalman filter / constant-velocity for bbox propagation
- **GPU acceleration** - PyTorch / CuPy / Open3D tensor ops for KD-Tree and DBSCAN
- **Multi-object data association** - Hungarian algorithm for ID consistency
- **Ground removal optimization** - pre-filter terrain points before clustering
- **Real sensor integration** — ROS 2 node subscribing to `sensor_msgs/PointCloud2`
- **Benchmark on real datasets** — nuScenes / Waymo evaluation
## References
- Ester et al., *A Density-Based Algorithm for Discovering Clusters in Large Spatial Databases with Noise* (DBSCAN, 1996)
- [Open3D: A Modern Library for 3D Data Processing](http://www.open3d.org/)
- [Argoverse Dataset](https://www.argoverse.org/)
- [BlenSor: Blender Sensor Simulation](https://www.blensor.org/)
