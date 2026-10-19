# Global corrected RGB map, SimpleLoopClosure — 19 Oct 2026

The FAST-LIVO2 /cloud_registered contains actual PointXYZRGB in camera_init,
stamped with ROS time at image processing/publication. This stamp is NOT the
exact timestamp of /cloud_registered_metric or /aft_mapped_to_init.

For each RGB frame at time t_rgb, select a keyframe k with closest
metric-LiDAR stamp only if |t_rgb - t_k| <= 0.75 seconds. Transform each
already world-frame RGB point by
T_GTSAM_world_body[k] * inverse(T_FAST_world_body[k]).
Never apply the raw FAST pose a second time. The corrected point clouds
are concatenated, voxel filtered, and published at /pgo_rgb_map_cloud
as colored PointCloud2 with RGB fields and frame camera_init.

The correction is a timestamp-gated *keyframe approximation*, not
sensor-accurate per-point deskewing. The original FAST RGB cloud can combine
old LiDAR points with a newer camera image; no per-point acquisition times are
carried by the colored cloud. Invalid frames, mismatched frames, absent RGB,
or frames whose timestamps differ too much from all optimized keyframes are
rejected, not falsely claimed loop-corrected.

Memory and throughput bounds: camera admission <=5Hz, raw pending queue<=8,
1200 cached frames, 2500 points/frame, 0.15m RGB frame downsampling,
0.20m map voxel and a 350000 point hard output cap. Conversion occurs
in a separate worker, reconstruction in a separate visualization thread
every 4s. Log SLC_PGO_RGB_MAP reports matched/unmatched frame counts and
map point counts. The original XYZI /pgo_map_cloud, exact-time C1 stream,
GTSAM/ICP geometry, and FAST-LIVO2 are NOT changed.

The canonical RGB RViz layout enables /pgo_rgb_map_cloud (RGB8) and GTSAM
optimized magenta marker, hides LOCAL FAST cyan original trajectory and raw
odom arrows by default; users can toggle these for comparison.
LIO without RGB produces no corrected colored cloud, as intended.

To run after validated promotion, launch FAST-LIVO2 with mapping_ugv.launch.py,
SimpleLoopClosure with config/offline_lio_livo_c1_candidate.yaml, and UGV10
with LiDAR topic remaps, ROS_DOMAIN_ID=84, ROS_LOCALHOST_ONLY=1,
RMW_IMPLEMENTATION=rmw_cyclonedds_cpp and --clock.

No independent ground truth or full F3 calibration yet. The native graph
may accept false-positive loop factors, which this visualization does not
validate.

## Validation evidence 19 Oct 2026

- Unit and package contracts: 125 tests PASS, zero errors/failures/skips.
- UGV10 50-sensor-second pilot, same ROS_DOMAIN_ID on isolated domain:
  /pgo_rgb_map_cloud PointCloud2 RGB fields nonempty, 47,476 points,
  109 optimized keyframes, 123 RGB frames matched and 0 pending overflows.
- UGV10 full 190-second playback, same FAST-LIVO2 integration/ROS2 Humble:
  405 optimized GTSAM keyframes, 85 accepted legacy loop factors,
  RGB map final 145,728 points, 448/507 RGB frames matched,
  59 temporally unmatched, 0 pending drops. A direct payload probe found
  491 unique RGB colors in a 491-point subsample, channel ranges
  R 3..254, G 6..255, B 0..254.
- Separate 40-s latest-binary test with independent marker and map loops:
  optimized_keyframes and loop_edges published every 1.00s median;
  /pgo_map_cloud at 3.03s median; /pgo_rgb_map_cloud at 4.01s median.
- All tests used immutable UGV10 bag and published isolated domains.
  Source evidence local only, under:
  ~/Documents/robotics_research_archive/slc_rgb_validation_20261019/
- Scientific caution: 85 is a *legacy backend accepted-factor count*, not
  85 ground-truth correct closures. Additional geometry/F3 heldout gates remain.
