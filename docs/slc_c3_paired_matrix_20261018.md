# SLC-C3 — Auditoría geométrica emparejada UGV10, Retail y CBD03

**Fecha:** 2026-10-18. **Alcance:** LIO y LIVO únicamente. La entrada FAST-LIVO2 permanece congelada en `52db4d4`. Este informe se deriva de geometría ya extraída de rosbags finalizados; no se han ejecutado nuevos ROS replays ni se han ajustado umbrales de loop closure.

| Dataset | Modo | Cierres aceptados analizados | Delta medio solapamiento holdout a 0,5 m (cierre menos control) | Ganancias en pares | Bootstrap pareado 95%, descriptivo |
|---|---|---:|---:|---:|---|
| UGV10 | LIO | 91 | +0,564 puntos porcentuales | 61/91 | [+0,28; +0,81] pp |
| UGV10 | LIVO | 86 | +1,302 pp | 69/86 | [+1,05; +1,62] pp |
| Retail Street | LIO | 36 | +2,158 pp | 22/36 | [−1,26; +5,57] pp |
| Retail Street | LIVO | 36 | +3,453 pp | 26/36 | [+0,17; +6,38] pp |
| CBD Building 03 | LIO | 29 | −0,168 pp | 14/29 | [−2,62; +1,99] pp |
| CBD Building 03 | LIVO | 28 | +0,427 pp | 11/28 | [−2,75; +2,77] pp |

Fuente de cálculo y hashes de las seis entradas: `~/Documents/simple_loop_closure_ws/reports/slc_c3_paired_3x2_20261018.json`; evaluaciones por modalidad `.../reports/{ugv10_lio,ugv10_livo,retail_lio,retail_livo,cbd03_lio}_geometry_v3_20261018.json` y `cbd03_livo_attempt10_geometry_v3_20261018.json`.

**Advertencias:** las parejas comparables comparten nubes, targets y una misma trayectoria; el bootstrap no representa independencia espacio-temporal entre cierres. Sus intervalos son diagnósticos descriptivos, NO intervalos de precisión de detección. Los controles `matched_admissible_nonedge` no están etiquetados como falsos positivos. El solapamiento simétrico usa la media de dos solapamientos unidireccionales sobre registros ajustados separadamente y no un único transform congelado. El informe adicional distingue pares con registros convergidos en ambas direcciones y pares que agotan iteraciones. Las métricas no autorizan clasificar TP/FP, recall, APE/RPE ni aplicar un umbral binario diseñado post hoc.

**Gate:** UGV10 muestra separación descriptiva favorable, Retail es mixto, CBD03 es inconcluso. CBD03 LIVO analizó únicamente las 2.369 odometrías realmente recibidas por el backend (de 2.377 esperadas). No aprobar recepción completa ni verdad geométrica para esa campaña.
