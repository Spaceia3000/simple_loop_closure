# Perfil operativo LIO/LIVO — candidato funcional V1 (2026-10-18)

Este perfil utiliza exactamente la misma ruta C++ que la rama integrada `75a1def`. Mantiene el YAML predeterminado intacto.

## Identidad

- Candidato integrado: `integration/slc-v1-drain-dds-candidate-20261018 @75a1def`.
- Instalación aislada comprobada: `~/Documents/simple_loop_closure_ws/worktrees/slc_v1_drain_dds_candidate_20261018/.codex_native/v1merged/install`.
- Perfil explícito: `config/offline_lio_livo_c1_candidate.yaml` de este worktree; incluye `sync_queue_size: 128`, `cloud_input_depth: 100`, `pgo_output_depth: 10`, `simple_stream_audit: true` y timestamps exactos.
- FAST-LIVO2 congelado: `refactor/fast-livo-yaml-first @52db4d4`; los artefactos normalizados contienen nube métrica y odometría con timestamps iguales.
- Solo reproducción offline LIO/LIVO en **la misma computadora**. No configura redes de varios PCs, no valida LO ni es una calibración ICP/GTSAM.

## Ejecución simple con datos FAST ya normalizados

**Terminal 1 — SimpleLoopClosure**

```bash
source /opt/ros/humble/setup.bash
source ~/Documents/simple_loop_closure_ws/worktrees/slc_v1_drain_dds_candidate_20261018/.codex_native/v1merged/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ROS_DOMAIN_ID=84 ROS_LOCALHOST_ONLY=1
ros2 launch ~/Documents/simple_loop_closure_ws/worktrees/slc_v1_drain_dds_candidate_20261018/launch/simple_loop_closure.launch.py loop_closure_params_file:=~/Documents/simple_loop_closure_ws/worktrees/slc_v1_drain_dds_candidate_20261018/config/offline_lio_livo_c1_candidate.yaml cloud_topic:=/cloud_registered_metric odometry_topic:=/aft_mapped_to_init
```

**Terminal 2 — RViz (trayectorias FAST y PGO, grafo/edges/mapa)**

```bash
source /opt/ros/humble/setup.bash
source ~/Documents/simple_loop_closure_ws/worktrees/slc_v1_drain_dds_candidate_20261018/.codex_native/v1merged/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ROS_DOMAIN_ID=84 ROS_LOCALHOST_ONLY=1
rviz2 -d ~/Documents/simple_loop_closure_ws/worktrees/slc_v1_drain_dds_candidate_20261018/rviz/simple_loop_closure_ab.rviz --ros-args -p use_sim_time:=true
```

**Terminal 3 — reproducir exactamente una de las opciones**

```bash
source /opt/ros/humble/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ROS_DOMAIN_ID=84 ROS_LOCALHOST_ONLY=1
ros2 bag play ~/Documents/fast_livo2_rtabmap_lc_ws/mvp1_runs/cbd03_lio_jetson_full_normalized --clock 100 --topics /aft_mapped_to_init /cloud_registered_metric /tf
```

Cambiar solo el path anterior a `cbd03_livo_jetson_full_normalized` para probar LIVO. Los tres procesos deben usar el mismo dominio; reproducir después de arrancar SLC y RViz y confirmar que suscriptores están activos.

**NOTA:** En reproducciones de 1× bajo carga, el CLI predeterminado de rosbag2 puede ofrecer depth10 para LiDAR cuando FAST original ofrecía depth100. Para campañas estrictas, usar los overrides QoS verificados en `scripts/run_slc_transport_diagnostic.py` y el gate `--certify-all`; la guía anterior solo demuestra el camino operativo y no certifica la entrega DDS.

## Visualización y límites

- Azul/cian FAST: odometría original; magenta PGO: trayectoria optimizada; marcadores rojos: cierres aceptados; `/pgo_map_cloud`: mapa reconstruido a partir de poses optimizadas.
- `/tf` proviene de FAST. SimpleLoopClosure **no** actualiza el TF o la odometría interna de FAST-LIVO2.
- La rama funcional usa parámetros geométricos heredados; sus 26 cierres en CBD03 LIO no equivalen a 26 cierres verdaderos científicamente validados.
- F3 matemático se mantiene en `feat/slc-f3-rank-gate-20261018 @2c13f0d`: por defecto inhabilita los factores de cierre sin ICP correspondencia calibrada, covarianza SPD en coordenadas GTSAM y condición de observabilidad validada. No fusionar este experimento mientras la calibración/heldout esté pendiente.
- Referencias: `docs/slc_v1_operator_validated_20261018.md`, `docs/livo_dds_telemetry_integration_20261018.md` y `docs/f3_covariance_frame_contract_20261018.md` en sus respectivas ramas.
