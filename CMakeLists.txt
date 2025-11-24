cmake_minimum_required(VERSION 3.10)
project(galaxy_camera_ros2)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(camera_info_manager REQUIRED)

include_directories(
  include
  include/libgxiapi
)

set(GALAXY_CAMERA_SOURCES
  src/galaxy_camera.cpp
)

add_library(galaxy_camera_component SHARED ${GALAXY_CAMERA_SOURCES})
ament_target_dependencies(galaxy_camera_component
  rclcpp
  rclcpp_components
  sensor_msgs
  image_transport
  camera_info_manager
)
target_link_libraries(galaxy_camera_component gxiapi)

rclcpp_components_register_nodes(galaxy_camera_component "galaxy_camera::GalaxyCameraNode")

add_executable(galaxy_camera_node src/galaxy_camera_node.cpp)
target_link_libraries(galaxy_camera_node galaxy_camera_component)
ament_target_dependencies(galaxy_camera_node
  rclcpp
)

install(TARGETS
  galaxy_camera_component
  galaxy_camera_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
