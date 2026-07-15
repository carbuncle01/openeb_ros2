cmake_minimum_required(VERSION 3.16)
project(openeb_ros2)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(event_camera_codecs REQUIRED)
find_package(event_camera_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)

# JetPilot uses the CenturyArks SilkyEvCam-enabled OpenEB/Metavision SDK
# installed under /usr/local.
find_package(MetavisionSDK COMPONENTS base core stream REQUIRED)

add_library(openeb_driver_component SHARED
  src/driver_component.cpp
)
target_include_directories(openeb_driver_component
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(openeb_driver_component
  diagnostic_msgs
  event_camera_msgs
  rclcpp
  rclcpp_components
)
target_link_libraries(openeb_driver_component
  MetavisionSDK::base
  MetavisionSDK::core
  MetavisionSDK::stream
)
rclcpp_components_register_nodes(openeb_driver_component
  "openeb_ros2::DriverComponent"
)

add_library(openeb_preprocessor_component SHARED
  src/preprocessor_component.cpp
)
target_include_directories(openeb_preprocessor_component
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(openeb_preprocessor_component
  diagnostic_msgs
  event_camera_codecs
  event_camera_msgs
  rclcpp
  rclcpp_components
  sensor_msgs
)
rclcpp_components_register_nodes(openeb_preprocessor_component
  "openeb_ros2::PreprocessorComponent"
)

add_executable(openeb_driver_node src/driver_node.cpp)
target_link_libraries(openeb_driver_node openeb_driver_component)
ament_target_dependencies(openeb_driver_node rclcpp)

add_executable(openeb_preprocessor_node src/preprocessor_node.cpp)
target_link_libraries(openeb_preprocessor_node openeb_preprocessor_component)
ament_target_dependencies(openeb_preprocessor_node rclcpp)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS
    openeb_driver_component
    openeb_preprocessor_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  TARGETS
    openeb_driver_node
    openeb_preprocessor_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

install(
  FILES README.md LICENSE
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_dependencies(
  diagnostic_msgs
  event_camera_codecs
  event_camera_msgs
  rclcpp
  rclcpp_components
  sensor_msgs
)
ament_package()
