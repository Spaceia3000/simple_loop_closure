# UGV10 RViz visibility fix (2026-10-18)

This change is visualization-only. It does not change FAST-LIVO2, the SLC loop
closure algorithm or gates, sensor publication rates, message stamps, DDS/RMW
configuration, or trajectory geometry. It does not interpolate or fabricate a
global `Path`.

## What made the view misleading

- The optimized sequential edges, original keyframe chain, and accepted loop
  edges all used the same 0.10 m marker width. The magenta optimized marker
  could cover the cyan FAST trajectory, while accepted loop edges appeared much
  heavier than their meaning warranted.
- RViz retained 1000 large magenta `/pgo_odom` arrows, visually creating a
  second thick global trajectory. The display now retains one tiny arrow: it is
  the current global PGO pose, not a path.
- The cyan native original-keyframe marker and current metric LiDAR scan were
  disabled.
- The offline candidate rendered only one of every four stored map frames
  (`vis_map_cloud_frame_interval: 3`) and voxelized the result at 0.8 m.

The offline LIO/LIVO candidate therefore uses independent marker widths:
optimized magenta 0.03 m and original cyan 0.03 m, matching the
previous RTAB-Map Path billboard width (visual reference only); accepted loops
red 0.008 m is a separate *new visualization preference*, as RTAB-Map MapGraph
does not expose an equivalent Line Width property in its RViz config.
FAST local /path width is 0.08 m, matching the historical UGV online RViz. A visible thin red edge means only that the existing backend accepted
that loop factor; its count or appearance is not evidence that the closure is
geometrically correct. Code defaults and `config/simple_loop_closure.yaml`
preserve the historical 0.10 m marker widths.

## LOCAL and GLOBAL data shown

| Scope | Display / topic | Meaning |
|---|---|---|
| LOCAL | `/path` | Vivid-cyan FAST `nav_msgs/Path`, uncorrected FAST trajectory. |
| LOCAL | `/simple_loop_closure/original_keyframes` | Cyan native `LINE_STRIP` through the actual stored FAST keyframe poses; useful even where `/path` overlaps the optimized result. |
| LOCAL | `/cloud_registered` | FAST RGB XYZ cloud, accumulated only by RViz using `RGB8` and `Decay Time: 10000`; it is not loop-corrected. This long retention can consume substantial GPU memory. |
| LOCAL | `/cloud_registered_metric` | Current metric XYZI LiDAR scan in the FAST world/fixed frame (`camera_init`), shown with `Decay Time: 0`; it is not an accumulated local LiDAR map. |
| LOCAL | `/rgb_img` | Current FAST camera image. |
| GLOBAL | `/simple_loop_closure/optimized_keyframes` | Magenta native `LINE_LIST` joining GTSAM-optimized keyframe poses. This is the global optimized trajectory display. |
| GLOBAL | `/simple_loop_closure/loop_edges` | Thin red native `LINE_LIST` containing backend-accepted loop factors only. |
| GLOBAL | `/pgo_odom` | One small arrow at the current corrected PGO pose (`Keep: 1`), not trajectory history. |
| GLOBAL | `/pgo_map_cloud` | Gray XYZI map rebuilt from stored metric keyframe clouds at optimized poses. It is not RGB. |

FAST contains a `/Laser_map` publisher but has no actual publish call on the
examined frontend path. Consequently there is no real local accumulated LiDAR
map topic to show here; `/cloud_registered_metric` is deliberately labeled as
the current metric scan. No `/rtabmap/*` topic or
`/simple_loop_closure/global_path` is used.

For visibility, the offline candidate changes only presentation sampling:
`vis_map_cloud_frame_interval` from 3 to 1 and
`vis_map_voxel_leaf_size` from 0.8 m to 0.4 m, matching the existing source and
target voxel leaf size. This retains more keyframes and points, so map rendering
and reconstruction are expected to use more CPU and RAM. It is a display-quality
tradeoff, not geometric calibration and not a SLAM algorithm change. The
default `config/simple_loop_closure.yaml` retains interval 3 and leaf size
0.8 m.

## Three terminals: original UGV10 bag at 1.0x

Start terminals 1 and 2 before terminal 3. All three use domain 84 and the same
CycloneDDS/localhost settings. The bag QoS override is explicit; `--rate 1.0`
preserves original sensor timing, and `--clock 100` controls clock publication
frequency rather than LiDAR, IMU, or camera rates.

**Terminal 1 — FAST-LIVO2 LIVO plus this RViz view**

```bash
source /opt/ros/humble/setup.bash
source ~/Documents/sensors_ws/install/setup.bash
source ~/Documents/fast_livo2_baseline_ws/install/setup.bash
export ROS_DOMAIN_ID=84 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ROS_LOCALHOST_ONLY=1
ros2 launch fast_livo mapping_ugv_livo.launch.py \
  use_sim_time:=true use_rviz:=true \
  rviz_config:=$HOME/Documents/simple_loop_closure_ws/worktrees/slc_rviz_visibility_fix_20261018/rviz/simple_loop_closure_ab.rviz
```

**Terminal 2 — SimpleLoopClosure candidate**

```bash
source /opt/ros/humble/setup.bash
source ~/Documents/simple_loop_closure_ws/worktrees/slc_rviz_visibility_fix_20261018/.codex_native/viz/install/setup.bash
export ROS_DOMAIN_ID=84 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ROS_LOCALHOST_ONLY=1
ros2 launch simple_loop_closure simple_loop_closure.launch.py \
  loop_closure_params_file:=$HOME/Documents/simple_loop_closure_ws/worktrees/slc_rviz_visibility_fix_20261018/config/offline_lio_livo_c1_candidate.yaml \
  cloud_topic:=/cloud_registered_metric odometry_topic:=/aft_mapped_to_init
```

**Terminal 3 — original UGV10 raw bag at 1.0x**

```bash
source /opt/ros/humble/setup.bash
source ~/Documents/sensors_ws/install/setup.bash
export ROS_DOMAIN_ID=84 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ROS_LOCALHOST_ONLY=1
ros2 bag play ~/Documents/datasets/ugv_fast_livo2/ugv_custom_fastlivo2_10 \
  --rate 1.0 --clock 100 \
  --qos-profile-overrides-path ~/Documents/fast_livo2_rtabmap_lc_ws/src/fast_livo2_rtabmap_lc/fast_livo2_rtabmap_bringup/config/ugv10_replay_qos.yaml \
  --topics /livox/lidar /livox/imu /camera/camera/color/image_raw
```
