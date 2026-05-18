cmake_minimum_required(VERSION 3.8)
project(semantic_octomap_ros2)

set(CMAKE_CXX_STANDARD 17)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O3)
endif()

# NUM_SEMANTICS configurable at build time (default=6 for PTv3 maize model)
set(NUM_SEMANTICS "6" CACHE STRING "Number of semantic classes tracked per voxel")
add_definitions(-DNUM_SEMANTICS=${NUM_SEMANTICS})
message(STATUS "Building semantic_octomap_ros2 with NUM_SEMANTICS=${NUM_SEMANTICS}")

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(octomap REQUIRED)
find_package(octomap_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL 1.10 REQUIRED COMPONENTS common filters)
find_package(semantic_octomap_interfaces REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
  include
  ${OCTOMAP_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

add_executable(semantic_octomap_node
  src/Semantics.cpp
  src/octomap_generator.cpp
  src/octomap_generator_ros2.cpp
  src/main.cpp
)

ament_target_dependencies(semantic_octomap_node
  rclcpp
  sensor_msgs
  nav_msgs
  std_msgs
  std_srvs
  geometry_msgs
  octomap_msgs
  tf2
  tf2_ros
  tf2_eigen
  pcl_conversions
  semantic_octomap_interfaces
  visualization_msgs
)


target_link_libraries(semantic_octomap_node
  ${OCTOMAP_LIBRARIES}
  ${PCL_LIBRARIES}
)

add_executable(test_subscriber
  src/test_subscriber.cpp
  src/Semantics.cpp
  src/octomap_generator.cpp
)
ament_target_dependencies(test_subscriber
  rclcpp sensor_msgs octomap_msgs nav_msgs visualization_msgs
  std_srvs std_msgs geometry_msgs tf2 tf2_ros tf2_eigen
  pcl_conversions semantic_octomap_interfaces
)
target_link_libraries(test_subscriber ${OCTOMAP_LIBRARIES} ${PCL_LIBRARIES})

install(TARGETS semantic_octomap_node test_subscriber
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch)

install(DIRECTORY rviz/
  DESTINATION share/${PROJECT_NAME}/rviz)

ament_package()
