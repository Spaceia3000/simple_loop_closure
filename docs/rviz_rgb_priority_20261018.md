# Comparación RViz UGV10: RGB FAST-LIVO2, RTAB-Map y SimpleLoopClosure
Fecha: 2026-10-18

## Hallazgo verificado

- `fast_livo2_baseline_ws/gtime_yaml_first_worktree/rviz_cfg/fast_livo2_ugv_livo.rviz` publica visualmente `/cloud_registered` en dos vistas RGB8: current 4 píxeles, Decay 0 s, y acumulada de 1 píxel, Decay 10000 s.
- `fast_livo2_rtabmap_lc_ws/src/fast_livo2_rtabmap_lc/fast_livo2_rtabmap_bringup/rviz/mvp1.rviz` muestra la nube local RGB de FAST `/cloud_registered` (RGB8, 2 píxeles, alpha 0.8, decay 10000), el mapa RTAB `MapCloud` está **desactivado**, el `/path` FAST es cian y el camino optimizado RTAB es magenta, ambos con `Line Width: 0.03`.
- En SimpleLoopClosure `/cloud_registered_metric` es XYZI (LiDAR métrico), y `/pgo_map_cloud` se construye en C++ como `pcl::PointXYZI`, con las nubes de keyframes transformadas mediante poses GTSAM optimizadas. La nube PGO **NO contiene RGB**. No se obtiene un mapa RGB global corregido cambiando un `Color Transformer`.
- El display previo de `/pgo_map_cloud` tenía cuadrados grises de 3 píxeles con opacidad 0.9: tapaba visualmente los puntos RGB de FAST y producía el mapa predominantemente gris.
- El FAST `/path` (cian) y el marcador cian de keyframes originales eran dos visualizaciones del mismo recorrido, produciendo superposición gruesa, no un error métrico certificado.

## Dos perfiles de un mismo ejecutable

**Vista inicial `rviz/simple_loop_closure_ab.rviz` (RGB prioritaria)**

- RGB acumulado: `/cloud_registered`, RGB8, 2 píxeles, alpha 1, Decay 10000. No está corregido por PGO.
- Scan RGB reciente: mismo tópico, RGB8, 4 píxeles, Decay 0.
- LiDAR geométrico temporal: `/cloud_registered_metric`, ventana de 2 s, amarillo ámbar atenuado, no es un mapa voxel local persistente.
- Mapa global geométrico de SLC: `/pgo_map_cloud` con intensidad XYZ, gris tenue alpha 0.18, 1 píxel, sin RGB.
- Trayectoria local FAST: `/path`, cian, ancho 0.03 m (referencia RTAB-Map). Keyframe original cian SLC desactivado para no superponer el mismo recorrido dos veces.
- Trayectoria global: marcador **nativo** GTSAM `/simple_loop_closure/optimized_keyframes` magenta, ancho publicado 0.03 m. Cierres aceptados: marcador rojo del backend ancho 0.008 m. Una sola flecha actual de `/pgo_odom`.
- Cámara: `/rgb_img` activo.

**Vista de geometría `rviz/simple_loop_closure_global_geometry.rviz` (PGO prioritaria)**

- Mismo algoritmo, mismas poses, ROS_DOMAIN_ID y 3 terminales.
- Resalta `/pgo_map_cloud` gris alpha 0.85 / 2 píxeles y deja RGB FAST alpha 0.45. Permite comparar el mapa geométrico global con el RGB original, sin confundir sus marcos.
- Puedes alternar entre las dos con `File > Open Config` en RViz; no necesitan más terminales.

### Limitación científica que no se debe ocultar

Ninguno de estos perfiles crea `/pgo_rgb_map_cloud` ni un **mapa global RGB corregido**. Para ello se requiere conservar los colores de `/cloud_registered`, asociar cada nube VIO a una pose temporal válida y aplicar la corrección optimizada a esos puntos. La nube VIO usa un timestamp distinto al par métrico LiDAR–odometría; no se puede asociar sin especificar y verificar el contrato temporal y los frames. Implementar ese pipeline requiere código y pruebas geométricas nuevas; no se debe fingir con un display de RViz.

### Reproducción

Se conservan EXACTAMENTE los tres comandos históricos y perfil UGV online del documento `docs/rviz_visibility_fix_20261018.md`, con `ROS_DOMAIN_ID=84`, `--rate 1.0`, `--clock 100`. No se modifica FAST-LIVO2, sensores, cámara, sincronización, DDS, factores, mapa PGO ni frecuencia de mensajes. Tras cargar la configuración, reproduce el bag original desde el principio para que el acumulador RGB construya su mapa visible. Si la nube RGB aparece vacía, comprobar si `/cloud_registered` publica campos `rgb` y si los mensajes VIO son no vacíos; no confundirlo con el tópico XYZI métrico.

**Sobre colores de líneas:** cian significa FAST sin corrección, magenta significa GTSAM optimizado, rojo significa arista de cierre aceptada. Cada categoría conserva UN color uniforme. Poner las tres clases exactamente del mismo color impediría reconocer la corrección; ese modo requeriría autorización explícita para alterar las diferencias visuales.
