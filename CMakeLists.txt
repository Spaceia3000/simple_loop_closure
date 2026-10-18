cmake_minimum_required(VERSION 3.5)
project(simple_loop_closure)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(GTSAM REQUIRED)
find_package(Boost COMPONENTS filesystem REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(message_filters REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_path(NANOFLANN_INCLUDE_DIR NAMES nanoflann.hpp
  HINTS ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/nanoflann/include
  REQUIRED
)

include_directories(
  include
  ${NANOFLANN_INCLUDE_DIR}
  ${GTSAM_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${Boost_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(${PROJECT_NAME}_node src/simple_loop_closure_node.cpp)
ament_target_dependencies(${PROJECT_NAME}_node
  "pcl_conversions"
  "rclcpp"
  "sensor_msgs"
  "nav_msgs"
  "std_msgs"
  "geometry_msgs"
  "message_filters"
  "pcl_ros"
  "visualization_msgs"
  "tf2"
  "tf2_eigen"
)
target_link_libraries(${PROJECT_NAME}_node
  ${Boost_LIBRARIES}
  ${PCL_LIBRARIES}
  gtsam
)
install(TARGETS
  ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

# add_executable(test_lc_node src/test.cpp)
# ament_target_dependencies(test_lc_node
#   "pcl_conversions"
#   "rcl_interfaces"
#   "rclcpp"
#   "rcutils"
#   "sensor_msgs"
#   "nav_msgs"
#   "std_msgs"
#   "geometry_msgs"
#   "pcl_ros"
#   "visualization_msgs"
#   "tf2"
#   "tf2_eigen"
# )
# target_link_libraries(test_lc_node
#   ${Boost_LIBRARIES}
#   ${PCL_LIBRARIES}
#   gtsam
# )
# install(TARGETS
#   test_lc_node
#   DESTINATION lib/${PROJECT_NAME}
# )

install(DIRECTORY config launch rviz
  DESTINATION share/${PROJECT_NAME} 
)

install(PROGRAMS
  scripts/analyze_loop_geometry.py
  scripts/evaluate_loop_closures.py
  scripts/evaluate_graph_correction.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_gtest(test_fast_livo2_contract test/test_fast_livo2_contract.cpp)
  target_include_directories(test_fast_livo2_contract PRIVATE include)
  target_link_libraries(test_fast_livo2_contract Eigen3::Eigen)
  ament_add_gtest(test_deterministic_loop_schedule test/test_deterministic_loop_schedule.cpp)
  target_include_directories(test_deterministic_loop_schedule PRIVATE include)
  ament_add_gtest(test_deterministic_gtsam_schedule test/test_deterministic_gtsam_schedule.cpp)
  target_include_directories(test_deterministic_gtsam_schedule PRIVATE include ${GTSAM_INCLUDE_DIRS})
  target_link_libraries(test_deterministic_gtsam_schedule gtsam)
  # This host has an incompatible globally installed third-party pytest plugin.
  # Disable plugin auto-loading so the package's self-contained tests remain
  # deterministic in both CMake's discovery probe and the test invocation.
  set(ENV{PYTEST_DISABLE_PLUGIN_AUTOLOAD} 1)
  ament_add_pytest_test(test_launch_contract test/test_launch_contract.py
    ENV
      "PYTEST_DISABLE_PLUGIN_AUTOLOAD=1"
      "ROS_LOG_DIR=${CMAKE_CURRENT_BINARY_DIR}/ros_test_logs"
  )
  ament_add_pytest_test(test_evaluate_loop_closures test/test_evaluate_loop_closures.py
    ENV "PYTEST_DISABLE_PLUGIN_AUTOLOAD=1"
  )
  ament_add_pytest_test(test_evaluate_graph_correction test/test_evaluate_graph_correction.py
    ENV "PYTEST_DISABLE_PLUGIN_AUTOLOAD=1"
  )
  ament_add_pytest_test(test_analyze_loop_geometry test/test_analyze_loop_geometry.py
    ENV "PYTEST_DISABLE_PLUGIN_AUTOLOAD=1"
  )
endif()

ament_package()
