# Loop Closure V1 operator guide (validated 2026-10-18)

This is the reproducible UC3M UGV operator path for ROS 2 Humble with
CycloneDDS. It covers FAST-LIVO2 **LIO and LIVO only** and the experimental
SimpleLoopClosure backend. It does not certify LO, loop accuracy, or a stable
release.

## Frozen source and runtime preflight

- FAST-LIVO2 source:
  `/home/breidy/Documents/fast_livo2_baseline_ws/gtime_yaml_first_worktree`,
  commit `52db4d422e00a0e1588bed86d91ac4361b21aca9`.
- Matching FAST runtime overlay:
  `/home/breidy/Documents/fast_livo2_jetson_frontend/install/setup.bash`.
  Its installed S1 launch and LIO/LIVO profiles have the same SHA-256 values as
  the frozen source.
- SimpleLoopClosure source: this checkout, commit
  `3f35f20b3dbc9426c11f0f2b690b1b3d6b11d68c`.
- SimpleLoopClosure is a candidate, not a stable release. Source an isolated
  install built from that exact commit as `SLC_INSTALL`. The shared
  `/home/breidy/Documents/simple_loop_closure_ws/install` is **not** a valid
  substitute: its installed parameter file is stale and it has no installed
  `simple_loop_closure_ab.rviz`.

Before a run, prove the source and inspect arguments without starting nodes:

```bash
git -C /home/breidy/Documents/fast_livo2_baseline_ws/gtime_yaml_first_worktree rev-parse HEAD
git -C /home/breidy/Documents/simple_loop_closure_ws/worktrees/slc_v1_operator_docs_20261018 rev-parse HEAD
export ROS_LOG_DIR=/tmp/slc_v1_ros_logs; mkdir -p "$ROS_LOG_DIR"
source /opt/ros/humble/setup.bash
source /home/breidy/Documents/fast_livo2_jetson_frontend/install/setup.bash
ros2 launch /home/breidy/Documents/fast_livo2_baseline_ws/gtime_yaml_first_worktree/launch/s1_fast_golden.launch.py --show-args
: "${SLC_INSTALL:?set SLC_INSTALL to an isolated install built from 3f35f20}"
source "$SLC_INSTALL/setup.bash"
ros2 launch /home/breidy/Documents/simple_loop_closure_ws/worktrees/slc_v1_operator_docs_20261018/launch/simple_loop_closure.launch.py --show-args
```

The verified FAST arguments are `mode`, `params_file`, `camera_params_file`,
`input_mode`, `image_topic`, `use_sim_time`, `use_rviz`, and `rviz_config`.
The verified backend arguments are `loop_closure_params_file`,
`odometry_topic` (default `/aft_mapped_to_init`), and `cloud_topic` (default
`/cloud_registered_metric`). Do not invent a `use_sim_time` backend launch
argument; none is declared.

Use this environment in every runtime terminal on one computer:

```bash
source /opt/ros/humble/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=84
export ROS_LOCALHOST_ONLY=1
export ROS_LOG_DIR=/tmp/slc_v1_ros_logs
mkdir -p "$ROS_LOG_DIR"
```

`ROS_LOCALHOST_ONLY=1` deliberately confines discovery and traffic to the same
host. For processes split across physical computers, set
`ROS_LOCALHOST_ONLY=0` (or unset it) on every host, retain the same domain ID
and RMW, and verify that the selected interfaces, multicast/unicast discovery,
and UDP firewall rules permit CycloneDDS traffic. Any `CYCLONEDDS_URI` must be
reviewed consistently across the hosts. Do not mix localhost-only and network
participants.

## A. Original UGV10 sensor bag through FAST-LIVO2 only (two terminals)

Start Terminal 1 first. Choose exactly one mode; the example is LIO. For LIVO,
replace both `lio` occurrences with `livo`. The explicit profile path prevents
an older installed profile from being selected. FAST RViz is disabled because
the backend overlay is covered separately.

**Terminal 1 — pinned FAST frontend**

```bash
source /opt/ros/humble/setup.bash
source /home/breidy/Documents/fast_livo2_jetson_frontend/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ROS_DOMAIN_ID=84 ROS_LOCALHOST_ONLY=1 ROS_LOG_DIR=/tmp/slc_v1_ros_logs
ros2 launch /home/breidy/Documents/fast_livo2_baseline_ws/gtime_yaml_first_worktree/launch/s1_fast_golden.launch.py mode:=lio params_file:=/home/breidy/Documents/fast_livo2_baseline_ws/gtime_yaml_first_worktree/config/ugv_v1_lio.yaml input_mode:=raw use_sim_time:=true use_rviz:=false
```

**Terminal 2 — original sensor bag**

LIO needs LiDAR and IMU:

```bash
source /opt/ros/humble/setup.bash
source /home/breidy/Documents/fast_livo2_jetson_frontend/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ROS_DOMAIN_ID=84 ROS_LOCALHOST_ONLY=1 ROS_LOG_DIR=/tmp/slc_v1_ros_logs
ros2 bag play /home/breidy/Documents/datasets/ugv_fast_livo2/ugv_custom_fastlivo2_10 --clock 100 --topics /livox/lidar /livox/imu --remap /livox/lidar:=/mid360/livox/lidar /livox/imu:=/mid360/livox/imu
```

LIVO additionally needs the raw color image:

```bash
source /opt/ros/humble/setup.bash
source /home/breidy/Documents/fast_livo2_jetson_frontend/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ROS_DOMAIN_ID=84 ROS_LOCALHOST_ONLY=1 ROS_LOG_DIR=/tmp/slc_v1_ros_logs
ros2 bag play /home/breidy/Documents/datasets/ugv_fast_livo2/ugv_custom_fastlivo2_10 --clock 100 --topics /livox/lidar /livox/imu /camera/camera/color/image_raw --remap /livox/lidar:=/mid360/livox/lidar /livox/imu:=/mid360/livox/imu
```

The remaps are required: the bag records `/livox/{lidar,imu}`, whereas both
reviewed profiles subscribe to `/mid360/livox/{lidar,imu}`. Do not use the
camera IMU as the FAST IMU. LO is implemented by the frontend launch but is
outside the V1 3-dataset x 2-mode evidence and is unsupported by this guide.

## B. Recorded FAST stream through SimpleLoopClosure (three terminals)

This workflow does **not** rerun FAST and must use a normalized frontend
artifact, not the raw sensor bag. Start Terminals 1 and 2 before playback.
Set `SLC_INSTALL` to an isolated install built from candidate `3f35f20`.

**Terminal 1 — candidate backend**

```bash
source /opt/ros/humble/setup.bash
: "${SLC_INSTALL:?set SLC_INSTALL to an isolated install built from 3f35f20}"
source "$SLC_INSTALL/setup.bash"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ROS_DOMAIN_ID=84 ROS_LOCALHOST_ONLY=1 ROS_LOG_DIR=/tmp/slc_v1_ros_logs
ros2 launch /home/breidy/Documents/simple_loop_closure_ws/worktrees/slc_v1_operator_docs_20261018/launch/simple_loop_closure.launch.py loop_closure_params_file:=/home/breidy/Documents/simple_loop_closure_ws/worktrees/slc_v1_operator_docs_20261018/config/simple_loop_closure.yaml odometry_topic:=/aft_mapped_to_init cloud_topic:=/cloud_registered_metric
```

**Terminal 2 — native A/B RViz overlay**

```bash
source /opt/ros/humble/setup.bash
: "${SLC_INSTALL:?set SLC_INSTALL to an isolated install built from 3f35f20}"
source "$SLC_INSTALL/setup.bash"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ROS_DOMAIN_ID=84 ROS_LOCALHOST_ONLY=1 ROS_LOG_DIR=/tmp/slc_v1_ros_logs
rviz2 -d /home/breidy/Documents/simple_loop_closure_ws/worktrees/slc_v1_operator_docs_20261018/rviz/simple_loop_closure_ab.rviz --ros-args -p use_sim_time:=true
```

**Terminal 3 — choose one recorded FAST artifact**

LIO:

```bash
source /opt/ros/humble/setup.bash
source /home/breidy/Documents/fast_livo2_jetson_frontend/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ROS_DOMAIN_ID=84 ROS_LOCALHOST_ONLY=1 ROS_LOG_DIR=/tmp/slc_v1_ros_logs
ros2 bag play /home/breidy/Documents/fast_livo2_jetson_frontend/normalized_bags/ugv10_lio_jetson_full_normalized --clock 100 --topics /aft_mapped_to_init /cloud_registered_metric /tf
```

LIVO:

```bash
source /opt/ros/humble/setup.bash
source /home/breidy/Documents/fast_livo2_jetson_frontend/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ROS_DOMAIN_ID=84 ROS_LOCALHOST_ONLY=1 ROS_LOG_DIR=/tmp/slc_v1_ros_logs
ros2 bag play /home/breidy/Documents/fast_livo2_rtabmap_lc_ws/mvp1_runs/ugv10_livo_jetson_full_normalized --clock 100 --topics /aft_mapped_to_init /cloud_registered_metric /tf
```

The RViz fixed frame is `camera_init`. It shows cyan FAST odometry/original
keyframes, magenta `/pgo_odom` and optimized sequential graph edges, red native
accepted loop edges, and `/pgo_map_cloud`. The replayed `/tf` is the FAST input
transform. SimpleLoopClosure does not broadcast a corrected TF.

To inspect an active run from a fourth terminal, use source-level expectations
only as a starting point and verify the negotiated graph:

```bash
source /opt/ros/humble/setup.bash
: "${SLC_INSTALL:?set SLC_INSTALL to an isolated install built from 3f35f20}"
source "$SLC_INSTALL/setup.bash"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ROS_DOMAIN_ID=84 ROS_LOCALHOST_ONLY=1
for topic in /aft_mapped_to_init /cloud_registered_metric /pgo_odom /pgo_map_cloud /simple_loop_closure/original_keyframes /simple_loop_closure/optimized_keyframes /simple_loop_closure/loop_edges; do ros2 topic info -v "$topic"; done
```

The recorded UGV10 artifacts offer reliable volatile `KEEP_LAST` data: depth
10 for odometry and depth 100 for metric cloud and TF. Candidate source sets
the cloud subscriber to reliable `KEEP_LAST(10)`; the odometry subscriber uses
the ROS default reliable profile. `/pgo_odom`, `/pgo_map_cloud`,
`/vis_pose_graph`, and the three explicit marker publishers use reliable
volatile `KEEP_LAST(1)` defaults. `ros2 topic info -v` is authoritative for an
active run; source and bag metadata alone do not prove live DDS matching or
delivery.

## Interface meaning and stop conditions

The admitted pair is exact-stamp metric data in the FAST global frame:

```text
/aft_mapped_to_init       nav_msgs/Odometry: camera_init -> aft_mapped
/cloud_registered_metric sensor_msgs/PointCloud2: points in camera_init
```

The backend uses ExactTime and rejects wrong frames, unequal timestamps,
invalid poses, empty clouds, and non-PointXYZI schemas. `/cloud_registered` is
not interchangeable because FAST stamps it from the node clock.

SimpleLoopClosure stores each accepted global cloud in the local body frame,
optimizes keyframe poses, and publishes a corrected `/pgo_odom`, corrected
`/pgo_map_cloud`, and native graph markers. It does **not** modify FAST's
internal odometry estimate, overwrite `/aft_mapped_to_init`, or publish a
corrected TF. Never interpret the visible FAST TF as the PGO result.

Stop the replay first, allow callbacks to drain, then stop backend and RViz.
No `/save_req` is needed for normal viewing; publishing it writes map/pose/PCD
files to the requested directory and is therefore a separate deliberate
operation.

## Evidence and present certification limit

Read-only `ros2 bag info` reported:

| Artifact | Duration | Relevant counts |
|---|---:|---|
| Raw UGV10 sensor bag | 190.042500638 s | 1,899 LiDAR; 37,984 Livox IMU; 4,502 color images; 94,322 total |
| UGV10 LIO normalized FAST | 189.400548254 s | 1,895 each odometry, metric cloud, TF |
| UGV10 LIVO normalized FAST | 149.606492528 s | 1,376 each odometry, metric cloud, TF |

The raw metadata SHA-256 is
`d3fa41166508d2f2ae83e09f9eedd72d026167639464d015cab70f5fe1a29a91`;
LIO normalized metadata is
`1b72377808ae381cf041e17799c9f046a26073fe985205858fef1f8a260bd3b7`;
LIVO normalized metadata is
`3342cbe7bfbdfb53976412e1f8049dc237579f37f0a82b75ef5bf91d8c8059aa`.
Each normalized artifact also has a `fast_frontend_provenance.json` naming the
clean producer commit `52db4d422e00a0e1588bed86d91ac4361b21aca9`; its topic
counts agree with `ros2 bag info`.

Historical read-only reports cover UGV10, Retail Street, and CBD Building 03
in LIO/LIVO. Five cases have complete exact-stamp FAST-to-PGO coverage. CBD03
LIVO attempt 10 is incomplete: 2,369 matches and eight missing backend
odometries. Those reports were computed from existing artifacts; they are not
fresh six-case reproductions of candidate `3f35f20`.

F3 / six-case certification therefore remains **open**. At minimum it needs
six fresh, provenance-locked candidate runs with complete delivery (including
a clean CBD03 LIVO run), retained stream/QoS evidence, and independent
geometric truth or adjudication. Current overlap/bootstrap summaries are
descriptive and correlated; they do not label true/false loops or prove
precision, recall, APE/RPE improvement, GTSAM factor residuals, covariance
quality, or global accuracy. Small PGO corrections are not proof of benefit,
and large corrections are not proof of a false loop.
