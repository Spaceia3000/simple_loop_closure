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

Clonar el backend en cualquier workspace ROS 2 (la rama de despliegue es `Cohemos`; los cambios de modo permanecen en una rama experimental hasta autorización de merge):

~~~bash
mkdir -p "$HOME/robot_ws/src"
cd "$HOME/robot_ws/src"
git clone --single-branch --branch Cohemos \
  https://github.com/Spaceia3000/simple_loop_closure.git

source /opt/ros/humble/setup.bash
cd "$HOME/robot_ws"
rosdep install --from-paths src --ignore-src --rosdistro humble -y
colcon build --packages-select simple_loop_closure --symlink-install
source install/setup.bash
~~~

Para usar `mode:=lio/livo` antes de su integración en `Cohemos`, compilar desde la rama experimental que contiene esos commits. En ARM64 se debe compilar localmente, no copiar `build/` ni `install/` desde x86_64.

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
source "$HOME/robot_ws/install/setup.bash"

ros2 launch simple_loop_closure simple_loop_closure.launch.py mode:=livo
~~~

Para LIO (LiDAR + IMU, sin reconstrucción RGB):

~~~bash
ros2 launch simple_loop_closure simple_loop_closure.launch.py mode:=lio
~~~

El launch resuelve `config/cohemo_lio.yaml` o `config/cohemo_livo.yaml` mediante el índice de paquetes ROS 2 (`ament_index`), relativo al prefijo instalado. No depende de un usuario, una carpeta concreta ni de la arquitectura de CPU. Asegúrese de ejecutar `source <workspace>/install/setup.bash` tras compilar el paquete en el equipo destino. Para pruebas especiales existe la opción `loop_closure_params_file:=/ruta/a/otro.yaml`, que prevalece sobre el perfil de modo.

Los tópicos predeterminados son `/cloud_registered_metric` y `/aft_mapped_to_init`; pueden cambiarse mediante `cloud_topic:=...` y `odometry_topic:=...`.

**Contrato exacto:** /cloud_registered_metric y /aft_mapped_to_init tienen que compartir el mismo timestamp para message_filters ExactTime, frame fijo camera_init y frame de cuerpo aft_mapped. El RGB original llega de /cloud_registered, y el mapa RGB reconstruido por correcciones de poses GTSAM sale por /pgo_rgb_map_cloud. La asociación por timestamp a keyframes es una **aproximación temporal**, no sincronización RGB físicamente certificada por punto.

En RViz activar el display GLOBAL corrected 3D RGB map y seleccionar Color Transformer = RGB8 para ver colores reales Sony. La visualización guardada puede tener ese display apagado. La trayectoria optimizada está en /simple_loop_closure/optimized_keyframes y las aristas de cierre aceptadas en /simple_loop_closure/loop_edges.

Configurar correctamente ROS_DOMAIN_ID y DDS entre ordenador, Jetson y drivers. ROS_LOCALHOST_ONLY=1 bloquea descubrimiento entre máquinas y debe evitarse si los sensores están en distintos dispositivos.

## Jetson AGX Orin, estado de verificación

ROS 2 Humble soporta oficialmente Ubuntu 22.04 arm64 como plataforma Tier 1 (REP-2000: https://reps.openrobotics.org/rep-2000/). En el host x86_64 se ha verificado la resolución de claves `rosdep` para `ubuntu:jammy` y `humble`, pero esa operación **no consulta el catálogo binario ARM64** ni demuestra que cada dependencia esté disponible en los repositorios APT de la Jetson.

En Jetson (Ubuntu 22.04 arm64, con repositorios Ubuntu/ROS configurados), validar primero:

~~~bash
uname -m
source /opt/ros/humble/setup.bash
rosdep check --from-paths "$HOME/robot_ws/src" --ignore-src --rosdistro humble --os ubuntu:jammy
apt-cache policy ros-humble-gtsam ros-humble-pcl-ros libnanoflann-dev
~~~

Tras compilar, ejecutar en la propia Jetson `ros2 run simple_loop_closure check_runtime.sh` antes de lanzar `mode:=lio` o `mode:=livo`. El preflight comprueba ejecutable, perfiles instalados y dependencias ELF usando `ldd`; aborta con error si encuentra bibliotecas `not found`. No omitir la comprobación tras actualizar GTSAM, ROS o PCL. La biblioteca `liblaser_mapping.so` de la fotografía pertenece a FAST-LIVO2 y debe comprobarse por separado en el frontend.

Confirmar candidatos ARM64 y compatibilidad ABI de GTSAM/PCL antes de instalar y compilar. Después usar `rosdep install` y `colcon build` en la propia Jetson. No copiar binarios ni `install/` desde x86_64. Registrar SHAs de drivers, GTSAM y FAST-LIVO2. La compilación y la ejecución sobre Jetson ARM64 siguen sin certificar; esta nota no sustituye esa prueba.

F3 (geometría/observabilidad del lazo), precisión APE/RPE y seguridad de navegación siguen pendientes; los ensayos históricos están en integration/v1-humble.
