# Cohemos — Simple Loop Closure para UGV (ROS 2 Humble)

Repositorio: https://github.com/Spaceia3000/simple_loop_closure — rama **Cohemos**.

Este repositorio contiene solo el backend GTSAM para UGV. Para producir nubes de puntos, odometría y RGB debe instalarse por separado FAST-LIVO2 https://github.com/Spaceia3000/FAST-LIVO2-ROS2-MID360-Fisheye, rama **Cohemos**, y los drivers de LiDAR/cámara.

## Dependencias y versiones de referencia

Comprobadas el 19/10/2026 en Ubuntu 22.04.5 x86_64 / ROS 2 Humble / Python 3.10.12 / GCC 11.4.0, C++17. Estas versiones no han sido certificadas para Jetson ARM64.

| Componente | Versión observada | Repositorio / APT |
|---|---|---|
| ROS 2 Humble | ros-humble-ros-base **0.10.0** | https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html |
| GTSAM usado al compilar el backend | **4.2.0** (CMake GTSAM_DIR=/opt/ros/humble/lib/cmake/GTSAM) | https://github.com/borglab/gtsam, paquete ros-humble-gtsam |
| GTSAM checkout adicional (NO es el usado por el build SLC) | **4.2a9**, Git c57988fe554e7213c77fe379c1d7c483de26ad33 | ~/Documents/third_party/gtsam |
| nanoflann | **1.4.2**, encabezado /usr/include/nanoflann.hpp | https://github.com/jlblancoc/nanoflann, paquete libnanoflann-dev |
| Eigen / Boost | **3.4.0 / 1.74** | libeigen3-dev, libboost-dev |
| PCL / pcl_ros | **1.12.1 / 2.4.5** | libpcl-dev, ros-humble-pcl-ros |
| ROS rclcpp | **16.0.21** | ros-humble-rclcpp |
| ROS message_filters | **4.3.20** | ros-humble-message-filters |
| ROS RViz2 | **11.2.29**, opcional en la Jetson sin GUI | ros-humble-rviz2 |
| FAST-LIVO2 | Rama Cohemos desde integración 8faf006 | https://github.com/Spaceia3000/FAST-LIVO2-ROS2-MID360-Fisheye |
| Driver Livox MID-360 | 1.2.6, Git 13eb05e4e6dd7a765b934d0c5fd6236676a57b49 | https://github.com/Spaceia3000/livox_ros_driver2 |

Importante: el paquete GTSAM **realmente detectado por CMake** es el de ROS Humble 4.2.0. No instalar sin querer otro GTSAM incompatible en /usr/local y dar por supuesta la equivalencia ABI. El antiguo checkout del sistema en versión 4.2a9 no es la dependencia detectada para SLC.

## Instalación Ubuntu 22.04 / ROS 2 Humble

Instalar ROS 2 Humble desde la guía oficial. Después:

~~~bash
sudo apt update
sudo apt install -y \
  python3-colcon-common-extensions \
  libpcl-dev libeigen3-dev libboost-all-dev libnanoflann-dev \
  ros-humble-ament-cmake ros-humble-rclcpp ros-humble-gtsam \
  ros-humble-pcl-ros ros-humble-pcl-conversions \
  ros-humble-message-filters ros-humble-tf2 ros-humble-tf2-eigen \
  ros-humble-nav-msgs ros-humble-sensor-msgs \
  ros-humble-geometry-msgs ros-humble-std-msgs ros-humble-std-srvs \
  ros-humble-visualization-msgs
~~~

RViz opcional en el ordenador del operador: sudo apt install -y ros-humble-rviz2.

**No se necesita clonar el submódulo nanoflann.** La rama Cohemos utiliza el paquete de sistema libnanoflann-dev.

Clonar el backend dentro del workspace del UGV:

~~~bash
mkdir -p ~/Documents/simple_loop_closure_ws/src
cd ~/Documents/simple_loop_closure_ws/src
git clone --single-branch --branch Cohemos \
  https://github.com/Spaceia3000/simple_loop_closure.git

source /opt/ros/humble/setup.bash
cd ~/Documents/simple_loop_closure_ws
colcon build --packages-select simple_loop_closure --symlink-install
source install/setup.bash
~~~

## Configuraciones y runtime

- src/, include/: backend C++, GTSAM, filtros y mapa RGB global.
- config/cohemo_lio.yaml: LiDAR+IMU, sin RGB.
- config/cohemo_livo.yaml: LiDAR+IMU+Sony, mapa RGB global por keyframes.
- launch/simple_loop_closure.launch.py: único launch.
- rviz/simple_loop_closure_ab.rviz: pantalla guardada del operador, ejes de origen y robot.
- CMakeLists.txt, package.xml, LICENSE: construcción y licencia.

Iniciar primero los drivers MID-360/Sony e IMU y FAST-LIVO2. En robot real los relojes deben ser coherentes y no se requiere /clock simulado.

~~~bash
source /opt/ros/humble/setup.bash
source ~/Documents/simple_loop_closure_ws/install/setup.bash

ros2 launch simple_loop_closure simple_loop_closure.launch.py \
  loop_closure_params_file:=$HOME/Documents/simple_loop_closure_ws/src/simple_loop_closure/config/cohemo_livo.yaml \
  cloud_topic:=/cloud_registered_metric \
  odometry_topic:=/aft_mapped_to_init
~~~

Para LIO sin imágenes cambiar el archivo a config/cohemo_lio.yaml.

**Contrato exacto:** /cloud_registered_metric y /aft_mapped_to_init tienen que compartir el mismo timestamp para message_filters ExactTime, frame fijo camera_init y frame de cuerpo aft_mapped. El RGB original llega de /cloud_registered, y el mapa RGB reconstruido por correcciones de poses GTSAM sale por /pgo_rgb_map_cloud. La asociación por timestamp a keyframes es una **aproximación temporal**, no sincronización RGB físicamente certificada por punto.

En RViz activar el display GLOBAL corrected 3D RGB map y seleccionar Color Transformer = RGB8 para ver colores reales Sony. La visualización guardada puede tener ese display apagado. La trayectoria optimizada está en /simple_loop_closure/optimized_keyframes y las aristas de cierre aceptadas en /simple_loop_closure/loop_edges.

Configurar correctamente ROS_DOMAIN_ID y DDS entre ordenador, Jetson y drivers. ROS_LOCALHOST_ONLY=1 bloquea descubrimiento entre máquinas y debe evitarse si los sensores están en distintos dispositivos.

## Jetson AGX Orin, estado de verificación

Compilar todo para ARM64 con versiones apropiadas a su JetPack/L4T y Ubuntu; no copiar binarios x86_64. Registrar SHAs de drivers, GTSAM, Vikit y FAST-LIVO2 en la Jetson. La rama reducida Cohemos no se ha recompilado ni ejecutado aún tras retirar material de desarrollo.

F3 (geometría/observabilidad del lazo), precisión APE/RPE y seguridad de navegación siguen pendientes; los ensayos históricos están en integration/v1-humble.
