# Checkpoint Loop Closure V1 — 18 de octubre de 2026

## Identidad del candidato
- Repositorio FAST-LIVO2: rama refactor/fast-livo-yaml-first, commit 52db4d4, limpio y sin modificaciones.
- SimpleLoopClosure estable: integration/slc-v1-20261018, commit 502225d, limpio e intacto.
- Candidato funcional UNIFICADO: integration/slc-v1-c1-s123-candidate-20261018, base 3f35f20, merge C1 66dc91b, guía 00f819e. Compilación Humble PASS, 103 pruebas de paquete sin fallos; suite Python unificada 102 PASS.
- Candidato SLC usa ExactTime(128), cloud_input_depth configurable (default 10, ensayos a 100), pgo_output_depth configurable (default 1, ensayos a 10), CycloneDDS en mismo host. No declara lazo calibrado científico.

## Certificación estricta C1
- C1 en rama fix/slc-c1-external-completeness-20261018: commits 97a54e6, 73de838, 93 tests Python PASS.
- El gate --certify-all exige auditorías nativas y secuencias de header.stamp iguales en fuente cloud/odom, testigo, recorder y ambas copias de PGO. Conserva multiplicidades y orden; fuente por SQLite y metadatos; sin contar 2660 de forma fija (admite también LIVO 2377).
- Auditoría offline de artefactos reales: CBD03 LIO íntegro PASS; tercera LIO FAIL (testigo cloud 2637/2660, grabador cloud 2612/2660); CBD03 LIVO FAIL (fuente 2377, backend 2341, testigo cloud 2309, grabador cloud 2305, PGO 2341).

## CBD03 LIO — evidencias anteriores
- Tres recorridos completos sobre SLC 3f35f20: backend recibe 2660 nubes, ExactTime sincroniza 2660, PGO publica/graba 2660, 351 keyframes, 176 búsquedas, 26 factores de cierre y 377 factores en todos; digests de búsquedas a2dc0d694e034765 y cierres c540fa4a97daa892; velocidades efectivas aprox. 0.9969x, 0.9983x, 0.9984x.
- Solo los dos primeros tuvieron integridad C1 externa completa. El tercero perdió nubes en testigo/recorder aunque no en backend ni PGO; por ello NO existe 3/3 global C1.

## Validación ROS del candidato unificado (30 segundos, 301 pares)
- Prueba inicial en directorio runs/slc_v1_unified_c1_certifyall_30s_20261018: con --drain-sec 5, FAIL correctamente. Fuente, testigo y grabador conservaban 301 pares; backend solo procesó 280 clouds/sync/PGO, faltando índices finales 280..300, sin errores UDP.
- Nueva prueba con mismo binario y demás ajustes, solo --drain-sec 25, directorio runs/slc_v1_unified_c1_certifyall_30s_drain25_20261018: PASS. Fuente/testigo/recorder registraron 301 nubes y 301 odometrías, testigo/recorder PGO 301, ambas auditorías internas PASS, 4 procesos retornaron 0. Un proceso externo de calibración LiDAR–cámara coexistía; no atribuir causalidad exclusiva. El valor 25 segundos no es universal; mejorar a drenaje basado en condición con watchdog.
- Primer intento de esta prueba con ROS_DOMAIN_ID 196 fue rechazado por validador seguro 0..190, antes de iniciar. Repetido con dominio 189.

## Geometría F3 — gate científico todavía ABIERTO
- Rama aislada feat/slc-f3-safe-geometry-20261018, commit 9dacb10. Build PASS, 115 tests (0 errores/fallos, 1 omitido por gate de observabilidad no implementado).
- Corrige unidades entre radio nanoflann (umbral cuadrático m2) e ICP (distancia m), excluye el scan fuente y vecinos demasiado cercanos por tiempo del submapa, separa covarianzas de rotación (rad2) y traslación (m2), prueba derivadas SE(3) y conserva la dirección del factor.
- NO hay ICP máximo calibrado, ni varianzas empíricas ni Hessiano/rank/condition robustos en producción: por seguridad esta rama deshabilita LOOP FACTORS por defecto. NO fusionarla con candidato funcional sin calibración; no declarar cerrado F3 ni validación TP/FP, APE/RPE, ground truth.
- Definir parámetros en fit/heldout con submapas reales y correspondencias ICP, validación de incertidumbre y observabilidad antes de integrar.

## Otras tareas y protección
- Guía reproducible LIO/LIVO y RViz: docs/slc_v1_operator_validated_20261018.md; original commit cbe6bcc e integrado en candidato. El backend no publica TF corregido. LO NO certificado.
- Telemetría DDS opcional: rama feat/slc-s2-ingress-dds-telemetry-20261018 @ee4477e, 111 tests PASS, pendiente de prueba causal en CBD03 LIVO.
- Forense LIVO: test/slc-s2-livo-loss-diagnostics-20261018 @19bd8fd, 6 tests PASS; nubes ausentes cerca 177.8–184.5 s, testigo/recorder/backend perdieron conjuntos distintos, DDS root cause desconocida.
- Restan 6 combinaciones UGV10/Retail/CBD03 por LIO/LIVO x 3 repeticiones, medición de rendimiento y calidad geométrica, visualización y UGV físico. LO aparte.
- SSD estaba al 99% y con unos 24 GB libres; no programar 18 bags de varios GB sin almacenamiento/retención segura; no borrar datos sin autorización.
- Todas las ramas y el frontend y SLC estable quedaron limpios; no se hicieron push ni merge a la rama estable.
