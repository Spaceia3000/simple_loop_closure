# Cohemo UGV — Simple Loop Closure (ROS 2 Humble)

Rama Cohemos, derivada de integration/v1-humble. Contiene solo el paquete
ROS 2 de lazo cerrado para UGV. Drivers de sensores y FAST-LIVO2 se despliegan
en sus propios paquetes; este repositorio no incluye dichos componentes.

## Dependencias

Ubuntu 22.04, ROS 2 Humble, GTSAM 4.x, PCL, Eigen, Boost, nanoflann
(libnanoflann-dev) y las dependencias de package.xml. Instalar los drivers
de Livox MID-360, Sony e IMU y FAST-LIVO2 por separado.

## Archivos para el despliegue

- src/ + include/: fuente y cabeceras del backend.
- config/cohemo_livo.yaml: LiDAR/IMU/Sony, mapa RGB global activado.
- config/cohemo_lio.yaml: LiDAR/IMU sin reconstrucción RGB.
- launch/simple_loop_closure.launch.py: único launch.
- rviz/simple_loop_closure_ab.rviz: vista del operador, ejes fijo/móvil.
- CMakeLists.txt, package.xml, LICENSE: construcción y licencia.

No hay datasets, scripts de investigación, informes ni tests en esta rama.

## Compilar en workspace ROS 2 del robot

~~~bash
source /opt/ros/humble/setup.bash
cd ~/Documents/simple_loop_closure_ws
colcon build --packages-select simple_loop_closure --symlink-install
source install/setup.bash
~~~

## Ejecutar después de iniciar sensores y FAST-LIVO2

FAST-LIVO2 debe ejecutarse con perfil Cohemo y use_sim_time:=false
para datos reales, publicando /aft_mapped_to_init y
/cloud_registered_metric con timestamps idénticos.

~~~bash
source /opt/ros/humble/setup.bash
source ~/Documents/simple_loop_closure_ws/install/setup.bash
ros2 launch simple_loop_closure simple_loop_closure.launch.py   loop_closure_params_file:=$HOME/Documents/simple_loop_closure_ws/src/simple_loop_closure/config/cohemo_livo.yaml   cloud_topic:=/cloud_registered_metric   odometry_topic:=/aft_mapped_to_init
~~~

Para LIO sin Sony, cambia a config/cohemo_lio.yaml. El mapa global de
RGB aparece en /pgo_rgb_map_cloud cuando la cámara aporta puntos reales,
con frame camera_init. La corrección RGB usa un keyframe temporalmente
compatible y no implica corrección precisa por punto.

## RViz

Cargar rviz/simple_loop_closure_ab.rviz. Contiene los ejes de origen y
del robot y la visualización GTSAM. Conserva los últimos ajustes que
guardó el operador. Si el display GLOBAL corrected 3D RGB map quedó
desactivado, marcar Enabled y seleccionar Color Transformer RGB8 para
visualizar colores reales en vez de colores de altura.

## Limitaciones de seguridad y validación

La certificación F3 de geometría de cierres y consistencia contra una
referencia independiente sigue pendiente. No tratar cierres ICP
aceptados como evidencia de localización segura en producción. Los
experimentos y reportes completos siguen preservados en integration/v1-humble.
