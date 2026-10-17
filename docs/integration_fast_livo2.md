# FAST-LIVO2 input integration audit

Date: 2026-10-17

This document is the G0/G1/G2 evidence record for the SimpleLoopClosure
integration.  It does not claim a rosbag replay, a detected loop, or runtime
accuracy.

## Stage status

| Stage / gate | Status | Evidence |
|---|---|---|
| G0 source and interface audit | PASS | Both source revisions, the real FAST YAML profiles, source functions, ROS interfaces, frames, timestamps, cloud ownership, GTSAM use, build dependencies, and four bag metadata records were inspected. |
| G0 UGV10 raw-bag availability | PASS | `ros2 bag info` reads the 17.5 GiB bag and reports 94,322 messages. No replay was performed. |
| G0 UGV10 calibrated RGB-D modality | PASS (availability only) | Image, CameraInfo, aligned depth, depth CameraInfo, depth cloud, `/tf`, and `/tf_static` are present. Calibration correctness and synchronization were not re-proved here. |
| G0 Retail camera modality | FAIL (multimodal admission) | Raw `sensor_msgs/Image` exists, but CameraInfo, depth, `/tf`, and `/tf_static` do not. It is image-bearing, not yet a proved calibrated visual input. |
| G0 CBD03 camera modality | FAIL (multimodal admission) | Raw `sensor_msgs/Image` and exposure exist, but CameraInfo, depth, `/tf`, and `/tf_static` do not. It is image-bearing, not yet a proved calibrated visual input. |
| G0 frozen FAST output | PASS | The normalized bag has 2,952 odometry, 2,952 metric-cloud, and 2,952 FAST TF messages. Its manifest proves exact sequence/stamp pairing and the frames below. |
| G1 unchanged SimpleLoopClosure build | FAIL | Correct isolated build reached CMake and failed because `tf2_eigen` was passed to `ament_target_dependencies()` without `find_package(tf2_eigen REQUIRED)`. |
| G2 wrapper/config build | BLOCKED | The required build base `/home/breidy/Documents/simple_loop_closure_ws/build` is read-only in this worker sandbox. Colcon stopped with `OSError: [Errno 30]` before CMake ran. No alternate build location was used. |
| G2 contract C++ tests | BLOCKED | Nine focused GTests are present, but the current source could not be compiled at the mandated build location. No current-binary GTest result is claimed. |
| G2 launch/static tests | PASS | Direct source-level pytest: 5 passed, 0 failed, 0 skipped. Colcon execution remains blocked with the build. |
| G2 installed launch contract | BLOCKED | No current install was produced, so installed `--show-args` was not claimed. Source-level launch assertions passed. |
| G2 live ROS / rosbag replay | NOT EVALUATED | Intentionally not run. No active ROS process was found during preflight, but this assignment does not authorize replay at G2. |

## Frozen revisions

- Immutable FAST frontend:
  `/home/breidy/Documents/fast_livo2_jetson_frontend/src/fast_livo2_frontend`
  at `52db4d422e00a0e1588bed86d91ac4361b21aca9` on
  `refactor/fast-livo-yaml-first`. Its worktree was clean and was not changed.
- SimpleLoopClosure starting point:
  `9f76a996f632d98934984719622195a1d40eb463` on
  `feat/fast-livo2-input`.

## FAST inputs selected by the real UGV YAML

All three canonical UGV profiles select these input names:

| Input | YAML evidence | Type / callback |
|---|---|---|
| LiDAR | `config/ugv_v1_{lo,lio,livo}.yaml:21`: `/mid360/livox/lidar` | `livox_ros_driver2/msg/CustomMsg`; `LIVMapper::livox_pcl_cbk`, `src/LIVMapper.cpp:1139` |
| IMU | `config/ugv_v1_{lo,lio,livo}.yaml:20`: `/mid360/livox/imu` | `sensor_msgs/msg/Imu`; `LIVMapper::imu_cbk`, `src/LIVMapper.cpp:1183` |
| Image | `config/ugv_v1_{lo,lio,livo}.yaml:19`: `/camera/camera/color/image_raw` | `sensor_msgs/msg/Image`; `LIVMapper::img_cbk` (the LIVO profile uses raw input) |

`LIVMapper::initializeSubscribersAndPublishers()` creates those subscriptions at
`src/LIVMapper.cpp:373-388`. Mode is derived rather than separately configured:
`img_en && lidar_en` gives LIVO, otherwise `imu_en` gives LIO, otherwise LO
(`src/LIVMapper.cpp:309`). The profile hashes are:

- LO: `1e6c23997c7a864b32a022632d494ae000d7ec628f9cbe02fb5d94d536c3a607`
- LIO: `2a904a82b06baa43d6f5b95f57ede43d0fb863fcd42e5ce43f1a82ad9068af46`
- LIVO: `b468764878c5c0baefea5869ab2fa49a1dd807387cce09ffc116d71da9a0038a`

The raw UGV10 bag uses `/livox/lidar` and `/livox/imu`, not the YAML's
`/mid360/livox/*` names. FAST therefore needs explicit replay remaps (or a
separately reviewed profile); this SimpleLoopClosure package must not silently
rename raw sensor inputs.

## FAST outputs relevant to loop closure

`LIVMapper::initializeSubscribersAndPublishers()` creates the relevant output
publishers at `src/LIVMapper.cpp:389-435`.

| Topic | Type | Frame and stamp | Admission |
|---|---|---|---|
| `/aft_mapped_to_init` | `nav_msgs/msg/Odometry` | `header.frame_id=camera_init`, `child_frame_id=aft_mapped`, stamp=`sec2Stamp(LidarMeasures.last_lio_update_time)` in `publish_odometry()`, `src/LIVMapper.cpp:1848-1868` | Required |
| `/cloud_registered_metric` | `sensor_msgs/msg/PointCloud2` | Global points in `camera_init`; stamp is the same `last_lio_update_time` in `processLIO()`, `src/LIVMapper.cpp:719-735` | Required |
| `/cloud_registered` | `sensor_msgs/msg/PointCloud2` | Global points in `camera_init`, but stamped with node-clock `now()` in `publish_frame_world()`, `src/LIVMapper.cpp:1568-1624` | Rejected for this synchronized adapter |
| `/tf` | `tf2_msgs/msg/TFMessage` | `camera_init -> aft_mapped`, copied from the odometry header in `publish_odometry()`, `src/LIVMapper.cpp:1860-1867` | Independent frame evidence |
| `/path` | `nav_msgs/msg/Path` | `camera_init`; appended from current FAST body pose in `publish_path()`, `src/LIVMapper.cpp:1879-1885` | Display only; not an optimizer input |

The metric output is enabled as `/cloud_registered_metric` by each UGV profile
(`publish.metric_cloud_en/topic`: LIO lines 108-109, LIVO lines 111-112, LO
lines 108-109). The UGV launch defaults to the LIVO YAML and `use_sim_time=true`
at `launch/mapping_ugv_livo.launch.py:18-20,42-45,52-60`.

## Timestamp, frame, and cloud-storage contract

The only admitted pair is:

```text
odom:  /aft_mapped_to_init     T_camera_init_body(t)
cloud: /cloud_registered_metric points in camera_init at exactly the same t
```

SimpleLoopClosure synchronizes `/cloud` and `/odometry` with an ApproximateTime
queue of 50 and a 0.01 s upper interval (`src/simple_loop_closure_node.cpp:85-91`),
but this FAST adapter additionally requires nanosecond equality. ApproximateTime
is only the delivery mechanism; it is not permission to accept an approximate
FAST pair.

For every accepted global point, storage is local body-frame data:

```text
p_body = inverse(T_camera_init_body(t)) * p_camera_init
```

The implementation is at `src/simple_loop_closure_node.cpp:364-370`. It also
changes the stored PCL header to `aft_mapped`. Map construction later applies
the optimized pose exactly once at `src/simple_loop_closure_node.cpp:403-415`.
FAST has already deskewed/aggregated the published cloud; this package performs
no IMU processing and no second deskew.

`mapped_cloud=true`, `expected_fixed_frame=camera_init`,
`expected_body_frame=aft_mapped`, and `require_exact_timestamps=true` are
fail-closed defaults in
`config/simple_loop_closure.yaml`. A mismatched cloud frame, odometry parent,
odometry child, stamp, non-finite/non-unit pose, empty cloud, or non-PointXYZI
schema is rejected before publishing or storing anything by
`inputContractValid()`. Startup also rejects `mapped_cloud=false`, empty/equal
frame names, and a negative synchronization tolerance.
Map and graph output also stop if the stored output header is not the expected
fixed frame (`src/simple_loop_closure_node.cpp:423-443,583-612`). Output stamps
come from the latest accepted keyframe header, not wall time.

## SimpleLoopClosure inputs, storage, GTSAM, and outputs

The launch remaps its private adapter names as follows:

- `/odometry -> /aft_mapped_to_init`
- `/cloud -> /cloud_registered_metric`

The save request remains `/save_req` (`std_msgs/msg/String`). Accepted keyframe
clouds are local `pcl::PointCloud<pcl::PointXYZI>` and accepted poses are
`Eigen::Affine3d` `T_camera_init_body`, held in parallel deques declared at
`include/simple_loop_closure/simple_loop_closure_node.hpp:94-99`.

GTSAM behavior is native SimpleLoopClosure behavior:

- `constructOdometryGraph()` adds a tight Pose3 prior and consecutive Pose3
  BetweenFactors from original FAST odometry deltas
  (`src/simple_loop_closure_node.cpp:650-678`).
- `constructLoopEdge()` searches optimized positions, builds target/source
  local-cloud assemblies, accepts ICP only on convergence and fitness threshold,
  then adds a robust Cauchy Pose3 BetweenFactor and records the actual accepted
  edge (`src/simple_loop_closure_node.cpp:815-875`).
- `updateISAM2()` updates iSAM2 and stores `calculateEstimate()`
  (`src/simple_loop_closure_node.cpp:878-894`).

Published backend-native outputs are:

| Topic | Type | Meaning / color |
|---|---|---|
| `/pgo_odom` | `nav_msgs/msg/Odometry` | Current input pose corrected from the last native optimized keyframe; original sensor header is retained. |
| `/pgo_map_cloud` | `sensor_msgs/msg/PointCloud2` | Corrected registered map in `camera_init`. |
| `/vis_pose_graph` | `visualization_msgs/msg/MarkerArray` | Backward-compatible aggregate of the four graph markers. |
| `/simple_loop_closure/original_keyframes` | `visualization_msgs/msg/Marker` | Cyan original FAST keyframe chain, directly from stored input odometry. |
| `/simple_loop_closure/optimized_keyframes` | `visualization_msgs/msg/Marker` | Magenta consecutive edges between native GTSAM optimized vertices. |
| `/simple_loop_closure/loop_edges` | `visualization_msgs/msg/Marker` | Red edges only for ICP-accepted loop factors recorded by the backend. |

The three explicit marker publishers are created at
`src/simple_loop_closure_node.cpp:26-31` and populated/published at lines
`479-612`. No interpolated trajectory or synthetic loop edge is
presented as native optimization output.

A `/save_req` message names an output directory. `saveThread()` writes
`map.pcd`, `poses.csv`, and one local-frame PCD per optimized keyframe
(`src/simple_loop_closure_node.cpp:198-279`). This is an explicit external
write requested through the existing API; the integration does not invoke it
automatically.

Pre-existing algorithm risks retained for a later algorithm stage: loop noise
uses ICP fitness as all six diagonal variances, and `search_radius_` is squared
before it is also used to derive ICP maximum correspondence distance. These are
not input-adapter changes and need independent G3 evaluation.

## Build dependency audit

FAST declares/finds ROS 2 ament/rclcpp/rclpy messages, PCL/PCL conversions,
tf2_ros, livox_ros_driver2, vikit_common/vikit_ros, cv_bridge, image_transport,
Eigen3, OpenCV, Sophus, Boost, fmt, optional OpenMP, and optional mimalloc
(`fast_livo2_frontend/CMakeLists.txt:4-5,64-103,133-202`). Its CMake directly
links vikit libraries from the workspace install tree at lines 172-176, so an
isolated FAST build still depends on those sibling packages being built there.

SimpleLoopClosure requires ament/rclcpp, ROS message packages,
message_filters, PCL/PCL conversions, tf2/tf2_eigen, Eigen3, Boost filesystem,
GTSAM, and nanoflann (`CMakeLists.txt:11-59`). GTSAM is found in the Humble
prefix on this host (`ros-humble-gtsam` 4.2.0). The APIs used here are
`Pose3`, `PriorFactor`, `BetweenFactor`, `Values`, and `ISAM2`, all supplied by
that package. The nanoflann gitlink is uninitialized; system
`libnanoflann-dev` is 1.4.2 (`NANOFLANN_VERSION=0x142`). CMake therefore uses a
vendored-header hint with a required system-search fallback. The source has
separate pre-1.5 and 1.5+ adaptor APIs; only the installed 1.4.2 branch was
previously compiled in the inherited artifact, not by this blocked run.
`package.xml` declares
the rosdep keys for Boost, Eigen, GTSAM, and the nanoflann fallback;
reproducible deployment must provide a discoverable `GTSAMConfig.cmake`.

The original G1 build exposed a real CMake dependency bug: `tf2_eigen` was in
the target dependency list but had no `find_package`. The correction adds
`find_package(tf2_eigen REQUIRED)`, adds the directly used `message_filters`
target dependency, and removes unused direct `rcl_interfaces`/`rcutils` target
dependencies. `rosdep check --rosdistro humble` reports all dependencies
satisfied.

## Dataset evidence (metadata only; no replay)

### UGV10 raw bag

Path:
`/home/breidy/Documents/datasets/ugv_fast_livo2/ugv_custom_fastlivo2_10`

- Metadata SHA-256:
  `d3fa41166508d2f2ae83e09f9eedd72d026167639464d015cab70f5fe1a29a91`
- DB SHA-256:
  `821c7b6d26544069b458a76a123f09e4df176210180d4a237b142a6bd3f41f99`
- 190.042500638 s, 94,322 messages.
- LiDAR 1,899; Livox IMU 37,984; color Image 4,502; color CameraInfo
  4,502; aligned depth Image 4,501; aligned-depth CameraInfo 4,502; depth
  cloud 4,502; TF 1,890; TF static 4.

### Frozen FAST UGV10 output

Path:
`/home/breidy/Documents/fast_livo2_rtabmap_lc_ws/mvp1_runs/metric_parity_20261013/canonical_fast_normalized`

- Metadata SHA-256:
  `a3449ac92b3bb234101aa31d54361a73b2df671e8e2a3ca7850ec5ef701814ab`
- DB SHA-256:
  `4837423ed871d25b605dd5a4fc6e9b5d3a5c0b01698e74673159493f43947732`
- 149.639847703 s; 2,952 messages on each required topic.
- `normalization_manifest.json:28-120` reports all odom/cloud stamps unique,
  monotonic, identical as a sequence, and all frames `camera_init` with child
  `aft_mapped`. Its payload digest matches the source (`:122-140`).
- This bag came from an earlier FAST output listed in the manifest, not from the
  pinned `52db4d4` checkout. It proves the transport/frame contract, not bitwise
  frontend-version equivalence.

### Retail and CBD03 image availability

Retail sensor-time path:
`/home/breidy/Documents/datasets/fast_livo2/converted/sensor_time/Retail_Street`
(metadata SHA-256
`c1220861b9865c8839e875b134276bb2ab65d019546f1edbe3b83434098586ec`).
It has 1,355 raw images, 1,355 LiDAR messages, and 27,447 IMU messages.

CBD03 sensor-time path:
`/home/breidy/Documents/datasets/fast_livo2/converted/sensor_time/CBD_Building_03`
(metadata SHA-256
`7394f8117ce3959348962b6162e94996315aa07e7d612dd27e492f47dd753272`).
It has 2,663 raw images, 2,663 LiDAR messages, 54,182 IMU messages, and
2,663 exposure messages. `ros2 bag info` successfully resolves its hidden
`.CBD_Building_03.tmp-36197467398_0.db3` payload.

This independently establishes image presence for both datasets, but neither
metadata record contains CameraInfo or TF. FAST-LIVO2 consumes calibration from
YAML rather than requiring CameraInfo, so this is not by itself a runtime
blocker; it means calibration provenance and the converted timestamp contract
must be validated externally before either run can count as closure evidence.

## Requirements for dataset execution

Common prerequisites for all three datasets:

1. Make `/home/breidy/Documents/simple_loop_closure_ws/build` writable, run the
   Humble build and both colcon test commands below, and source
   `build/install/setup.bash` only after they pass.
2. Build and source the immutable frontend revision `52db4d4` with its
   `livox_ros_driver2`, vikit, PCL/OpenCV, and other declared dependencies. Do
   not modify the frozen checkout to prepare a run.
3. Use `use_sim_time=true` for both nodes and replay with `--clock`; use an
   isolated ROS domain and confirm no unrelated publishers exist on either
   admitted backend input.
4. Confirm at runtime that `/aft_mapped_to_init` and
   `/cloud_registered_metric` have equal counts/stamp sequences, parent frame
   `camera_init`, child `aft_mapped`, and PointXYZI metric-cloud fields before
   assessing backend output.
5. Record enough CPU/RAM/disk headroom for the frontend map and backend stored
   keyframes, then preserve logs and output-topic evidence. A completed replay,
   optimizer result, and (where expected) detected loop are required before
   claiming dataset closure.

UGV10 additionally requires the reviewed `ugv_v1_livo.yaml` calibration and
mode, plus replay remaps `/livox/lidar -> /mid360/livox/lidar` and
`/livox/imu -> /mid360/livox/imu`. Its image topic already matches
`/camera/camera/color/image_raw`. The profile must retain metric output enabled
on `/cloud_registered_metric`. The normalized FAST-output bag may exercise the
backend contract without rerunning FAST, but it is not proof that pinned
`52db4d4` reproduced that output.

Retail and CBD03 already use `/livox/lidar`, `/livox/imu`, and
`/left_camera/image`, matching the Avia/MARS-style profiles. Before running,
select and provenance-check the dataset's LiDAR-IMU-camera extrinsics,
intrinsics/distortion, image scale, and IMU/image offsets; do not infer them
from the absence of CameraInfo. Ensure the selected profile explicitly sets
`publish.metric_cloud_en=true` and topic `/cloud_registered_metric` (the
checked `avia.yaml` does; `MARS_LVIG.yaml` does not). CBD03's
`/exposure_time` topic is not subscribed by this frontend, which estimates
exposure internally; any intention to consume that topic requires a separately
reviewed adapter. Both converted bags still require a full stamp/frame audit
and replay.

## Reproducible commands and results

The requested command was issued with a clean Humble setup and the exact
concurrency limits:

```bash
source /opt/ros/humble/setup.bash
export CMAKE_BUILD_PARALLEL_LEVEL=2
export PYTEST_DISABLE_PLUGIN_AUTOLOAD=1
colcon --log-base /home/breidy/Documents/simple_loop_closure_ws/build/colcon-log build \
  --base-paths /home/breidy/Documents/simple_loop_closure_ws/src/simple_loop_closure \
  --packages-select simple_loop_closure \
  --build-base /home/breidy/Documents/simple_loop_closure_ws/build \
  --install-base /home/breidy/Documents/simple_loop_closure_ws/build/install \
  --parallel-workers 1 \
  --event-handlers console_direct+
```

Result: **BLOCKED before CMake**. Colcon could not create the build/log base:
`OSError: [Errno 30] Read-only file system:
'/home/breidy/Documents/simple_loop_closure_ws/build'`. No alternate build,
install, or test base was used.

Once that path is writable, the required test commands are:

```bash
source /opt/ros/humble/setup.bash
export CMAKE_BUILD_PARALLEL_LEVEL=2
export PYTEST_DISABLE_PLUGIN_AUTOLOAD=1
colcon --log-base /home/breidy/Documents/simple_loop_closure_ws/build/colcon-log test \
  --base-paths /home/breidy/Documents/simple_loop_closure_ws/src/simple_loop_closure \
  --packages-select simple_loop_closure \
  --build-base /home/breidy/Documents/simple_loop_closure_ws/build \
  --install-base /home/breidy/Documents/simple_loop_closure_ws/build/install \
  --parallel-workers 1 \
  --return-code-on-test-failure \
  --event-handlers console_direct+
colcon test-result \
  --test-result-base /home/breidy/Documents/simple_loop_closure_ws/build/simple_loop_closure \
  --verbose
```

Current exact counts are: C++ GTest **0 executed** (9 defined; build blocked),
colcon launch/static **0 executed** (5 defined; build blocked), and direct
source-level launch/static pytest **5 passed, 0 failed, 0 skipped**. The direct
test used the Humble environment, disabled third-party pytest plugin loading,
and wrote ROS logs under `/tmp`; it is supporting evidence, not a substitute
for the required colcon result.

Additional source checks passed: package discovery with explicit
`--base-paths`, XML parsing, `rosdep check --rosdistro humble`, `git diff
--check`, generated-artifact ignore checks, and immutable frontend revision
verification. Runtime/replay remains **NOT EVALUATED**.

## Commit handoff

Atomic commit status: **BLOCKED by the read-only `.git` sandbox policy**.
Nothing was staged or committed; the supervisor must review and commit the
working tree. No merge or push was attempted.
