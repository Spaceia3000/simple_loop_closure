# Comparación RViz RTAB-Map → SimpleLoopClosure (UGV10 LIVO)

Esta vista conserva el layout operativo RGB de
`fast_livo2_rtabmap_bringup/rviz/mvp1.rviz`, pero usa exclusivamente salidas
nativas de SimpleLoopClosure para el resultado global. No modifica FAST-LIVO2,
sus perfiles, modos, timestamps, el sistema ROS ni la geometría del cierre.

## Correspondencia exacta de displays

| Vista | Topic | Tipo / significado |
|---|---|---|
| GLOBAL odometría corregida magenta | `/pgo_odom` | `nav_msgs/msg/Odometry`; conserva la salida PGO original |
| GLOBAL mapa gris | `/pgo_map_cloud` | `sensor_msgs/msg/PointCloud2` XYZI; intensidad, **no RGB** |
| GLOBAL trayectoria de keyframes GTSAM optimizados | `/simple_loop_closure/optimized_keyframes` | `visualization_msgs/msg/Marker` |
| GLOBAL cierres aceptados rojos | `/simple_loop_closure/loop_edges` | `visualization_msgs/msg/Marker` |
| LOCAL trayectoria FAST cian | `/path` | `nav_msgs/msg/Path`, geometría local sin corrección de lazo |
| LOCAL preview RGB acumulado | `/cloud_registered` | RGB8, `Decay Time: 10000`; geometría FAST sin corrección global |
| LOCAL último scan RGB | `/cloud_registered` | RGB8, `Decay Time: 0`; desactivado para evitar duplicar carga |
| LOCAL imagen cámara | `/rgb_img` | `sensor_msgs/msg/Image`; activada por defecto |
| Diagnóstico métrico | `/cloud_registered_metric` | nube de entrada del backend; desactivada en RViz |
| Keyframes originales opcionales | `/simple_loop_closure/original_keyframes` | marker cian; desactivado por defecto |

No se crea ni publica un `nav_msgs/Path` global adicional. La trayectoria global
que visualizamos es el marcador nativo de las poses optimizadas de GTSAM:
`/simple_loop_closure/optimized_keyframes`. `/pgo_odom` es odometría
propagada a partir de la última corrección optimizada y no constituye una
reoptimización independiente de cada timestamp.

No se usa ningún topic `/rtabmap/*`. La nube RGB de FAST sigue siendo local y
no se presenta como mapa RGB corregido. SLC tampoco publica un TF corregido.
Si el preview de `Decay Time: 10000` consume demasiada memoria/GPU, desactivar
solo `LOCAL FAST RGB cumulative preview`; la imagen puede mantenerse activa.

## Tres terminales, raw bag original a 1.0×

Usar el mismo `ROS_DOMAIN_ID` en las tres terminales. Esta secuencia mantiene el
launch nativo UGV LIVO y solo selecciona este RViz mediante el argumento ya
existente `rviz_config`. No ejecutar a la vez el backend RTAB-Map.

**Terminal 1 — FAST-LIVO2 UGV online + RViz**

```bash
source /opt/ros/humble/setup.bash
source ~/Documents/sensors_ws/install/setup.bash
source ~/Documents/fast_livo2_baseline_ws/install/setup.bash
export ROS_DOMAIN_ID=84 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ROS_LOCALHOST_ONLY=1
ros2 launch fast_livo mapping_ugv_livo.launch.py \
  use_sim_time:=true use_rviz:=true \
  rviz_config:=$HOME/Documents/simple_loop_closure_ws/worktrees/slc_v1_drain_dds_candidate_20261018/rviz/simple_loop_closure_ab.rviz
```

**Terminal 2 — SimpleLoopClosure**

```bash
source /opt/ros/humble/setup.bash
source ~/Documents/simple_loop_closure_ws/worktrees/slc_v1_drain_dds_candidate_20261018/.codex_native/v1merged/install/setup.bash
export ROS_DOMAIN_ID=84 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ROS_LOCALHOST_ONLY=1
ros2 launch simple_loop_closure simple_loop_closure.launch.py \
  loop_closure_params_file:=$HOME/Documents/simple_loop_closure_ws/worktrees/slc_v1_drain_dds_candidate_20261018/config/offline_lio_livo_c1_candidate.yaml \
  cloud_topic:=/cloud_registered_metric odometry_topic:=/aft_mapped_to_init
```

**Terminal 3 — UGV10 raw ROS bag original a 1.0×**

```bash
source /opt/ros/humble/setup.bash
source ~/Documents/sensors_ws/install/setup.bash
export ROS_DOMAIN_ID=84 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ROS_LOCALHOST_ONLY=1
ros2 bag play ~/Documents/datasets/ugv_fast_livo2/ugv_custom_fastlivo2_10 \
  --rate 1.0 --clock 100 \
  --qos-profile-overrides-path ~/Documents/fast_livo2_rtabmap_lc_ws/src/fast_livo2_rtabmap_lc/fast_livo2_rtabmap_bringup/config/ugv10_replay_qos.yaml \
  --topics /livox/lidar /livox/imu /camera/camera/color/image_raw
```

Arrancar primero FAST/RViz y SLC; después iniciar el reproductor. El valor `--clock 100` establece la frecuencia de publicación del reloj ROS, no la frecuencia de LiDAR/IMU/cámara. `--rate 1.0` mantiene el ritmo temporal original. No reejecutar RTAB-Map simultáneamente.

El `Path` global se actualiza en el thread de
visualización existente (1 Hz), usa el último stamp de entrada sincronizada
aceptada y expresa cada pose optimizada directamente en `camera_init`.
