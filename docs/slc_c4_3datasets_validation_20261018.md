# SLC-C4 — Validación real 3 datasets × LIO/LIVO (18-10-2026)

Trabajo aislado `feat/slc-c4-20261018`, base SimpleLoopClosure `96d358e`, productor FAST-LIVO2 `52db4d4`. Evaluador read-only: `scripts/evaluate_graph_correction.py`; los rosbags de FAST y los grafos existentes no se modificaron. No se usó LO ni RTAB-Map.

| Dataset | Modo | FAST–PGO por timestamp exacto | FAST sin PGO | Factores finales | Cambio p95 de keyframes (m) |
|---|---|---:|---:|---:|---:|
| UGV10 | LIO | 1895 | 0 | 91 | 0.02120 |
| UGV10 | LIVO | 1376 | 0 | 86 | 0.03373 |
| Retail Street | LIO | 1352 | 0 | 36 | 0.00832 |
| Retail Street | LIVO | 1351 | 0 | 36 | 0.01305 |
| CBD Building 03 | LIO | 2660 | 0 | 29 | 0.05051 |
| CBD Building 03 | LIVO (intento 10) | 2369 | **8** | 28 | 0.07607 |

Evidencias JSON en `~/Documents/simple_loop_closure_ws/reports/{ugv10_lio,ugv10_livo,retail_lio,retail_livo,cbd03_lio}_graph_ab_20261018.json` y `cbd03_livo_attempt10_graph_ab_20261018.json`. El caso CBD03 LIVO no pasa cobertura por ocho mensajes perdidos; no inferir su geometría comparando índices contra FAST completo.

Todas las combinaciones devolvieron JSON legible; la función computó diferencias temporales exactas sin fabricar etiquetas de lazo verdadero/falso. Correcciones pequeñas no prueban mejora de APE/RPE; faltan ground truth independiente y medida nativa de factor GTSAM, orientación de keyframes y covarianzas en los marcadores.

Validación Codex: build ROS 2 Humble PASS, CTest 34/34, pytest dirigido 11/11. Las campañas aquí son auditoría **offline** sobre resultados históricos, no nuevas reproducciones ni un PASS científico global. No modificar la rama estable hasta validar el conjunto C1/C2 con datasets.
