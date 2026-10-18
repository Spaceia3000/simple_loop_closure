# Global graph correction: traceable A/B

This package exposes a read-only A/B view of native SimpleLoopClosure output.
It neither changes FAST-LIVO2 nor the loop detector/optimizer.

## Native topics and frame contract

The fixed frame is `camera_init` and the body child frame is `aft_mapped`.
There is no extra map/odom transform in this integration.

| Evidence | Topic | Contract |
|---|---|---|
| A full pose | `/aft_mapped_to_init` | FAST `T_camera_init_aft_mapped`; measurement timestamp |
| B full pose | `/pgo_odom` | Corrected pose with the original FAST header and child frame |
| A keyframe positions | `/simple_loop_closure/original_keyframes` | Native cyan `LINE_STRIP`, `camera_init` |
| B sequential positions | `/simple_loop_closure/optimized_keyframes` | Native magenta `LINE_LIST`, `camera_init` |
| Accepted loop edges | `/simple_loop_closure/loop_edges` | Native red `LINE_LIST`; accepted factors only |
| Aggregate graph | `/vis_pose_graph` | The same native markers plus optimized nodes |
| Corrected map | `/pgo_map_cloud` | Native corrected cloud, `camera_init` |

`/vis_pose_graph` serializes positions, not optimized orientations, loop-factor
measurements, covariances, or GTSAM residuals. The evaluator therefore uses
exact-stamp `/aft_mapped_to_init` versus `/pgo_odom` for full SE(3), and uses
the last aggregate marker snapshot only for positional graph metrics.

The backend does not broadcast a corrected TF. Its `/pgo_odom` retains
`camera_init -> aft_mapped` identifiers and the input measurement stamp, while
`/pgo_map_cloud` and graph markers use `camera_init` and the latest accepted
input header. Any `camera_init -> aft_mapped` TF visible in RViz remains the
frontend's input TF and must not be interpreted as a PGO transform.

## Offline evaluator

The evaluator reads bags through `rosbag2_py`; it does not play them, publish,
optimize, or modify them:

```bash
source /opt/ros/humble/setup.bash
source INSTALL_PREFIX/setup.bash
ros2 run simple_loop_closure evaluate_graph_correction.py \
  --fast-bag FAST_OUTPUT_BAG \
  --backend-bag BACKEND_OUTPUT_BAG \
  --output /tmp/graph_correction_ab.json
```

The JSON contains:

- exact-stamp full-SE(3) position/rotation correction distributions;
- relative sequential-edge SE(3) consistency distributions;
- final original-to-optimized node displacement distributions;
- sequential graph edge-vector and edge-length-change distributions;
- accepted-loop endpoint chord changes, explicitly not factor residuals;
- counts below `0.10 m` and at/above `5 m`.

Those scale bands make centimetre corrections distinguishable from a
five-metre-scale event. They are descriptive inspection bands, not detector
thresholds or truth labels. A `>=5 m` correction deserves inspection against
the red native loop edges and sensor evidence, but this tool does not call it
an erroneous loop. Likewise, a small correction is not evidence of improved
accuracy.

## ROS 2 Humble RViz overlay

The installed configuration consumes existing native topics only:

```bash
rviz2 -d "$(ros2 pkg prefix simple_loop_closure)/share/simple_loop_closure/rviz/simple_loop_closure_ab.rviz"
```

It overlays cyan original keyframes, magenta optimized edges, red accepted
loop edges, cyan/magenta odometry arrows, `/pgo_map_cloud`, and the input TF
with `camera_init` fixed. RViz colors for graph edges come from the backend
marker messages; the config does not synthesize corrected paths, corrected TF,
or loop edges.

## Reproducible two-terminal playback

These examples consume a previously recorded FAST output artifact. They do
not rerun FAST and must not be pointed at a raw sensor bag. The LIO/LIVO label
is provenance: use the corresponding artifact produced with the reviewed
`ugv_v1_lio.yaml` or `ugv_v1_livo.yaml`, both of which publish the admitted
`/aft_mapped_to_init` and `/cloud_registered_metric` contract. No equivalence
between LIO and LIVO results is implied.

Use an isolated domain in both terminals. For a LIO artifact:

Terminal 1:

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=84
ros2 bag play FAST_LIO_OUTPUT_BAG --clock \
  --topics /aft_mapped_to_init /cloud_registered_metric
```

Terminal 2:

```bash
source /opt/ros/humble/setup.bash
source INSTALL_PREFIX/setup.bash
export ROS_DOMAIN_ID=84
ros2 launch simple_loop_closure simple_loop_closure.launch.py
```

For a LIVO artifact, Terminal 2 is unchanged and Terminal 1 is:

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=84
ros2 bag play FAST_LIVO_OUTPUT_BAG --clock \
  --topics /aft_mapped_to_init /cloud_registered_metric
```

The backend launch is grounded in its existing defaults:
`odometry_topic:=/aft_mapped_to_init` and
`cloud_topic:=/cloud_registered_metric`. Run the RViz command separately when
visual inspection is needed. If reproducing FAST from a raw UGV10 bag instead,
use the pinned frontend's existing `s1_fast_golden.launch.py mode:=lio` or
`mode:=livo` and the documented sensor-topic remaps; that is a three-process
workflow and is intentionally not disguised as the two-terminal artifact
playback above.

## Interpretation limits

The sequential residual is
`inverse(relative_FAST_i_j) * relative_PGO_i_j`; it measures how much a
successive motion changed. The graph edge-vector residual is an analogous
position-only quantity. Neither is a serialized GTSAM factor residual.

Without ground truth or independent adjudication, none of these metrics
establishes accuracy improvement, TP/FP, precision, recall, or that a specific
accepted edge is erroneous.
