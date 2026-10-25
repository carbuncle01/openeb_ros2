cmake_minimum_required(VERSION 3.16)
project(openeb_ros2)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options($<$<COMPILE_LANGUAGE:CXX>:-Wall>
    $<$<COMPILE_LANGUAGE:CXX>:-Wextra> $<$<COMPILE_LANGUAGE:CXX>:-Wpedantic>)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()
find_package(diagnostic_msgs REQUIRED)
find_package(event_camera_codecs REQUIRED)
find_package(event_camera_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)

# JetPilot uses the CenturyArks SilkyEvCam-enabled OpenEB/Metavision SDK
# installed under /usr/local.
find_package(MetavisionSDK COMPONENTS base core stream REQUIRED)
find_package(MetavisionHAL REQUIRED)
find_package(Threads REQUIRED)

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
  std_srvs
)
target_link_libraries(openeb_driver_component
  Metavision::HAL
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
  rcl_interfaces
  rclcpp
  rclcpp_components
  sensor_msgs
)
target_link_libraries(openeb_preprocessor_component
  Threads::Threads
  MetavisionSDK::base
  MetavisionSDK::core
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
  DIRECTORY launch config docs
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
  rcl_interfaces
  rclcpp
  rclcpp_components
  sensor_msgs
  std_srvs
)
option(OPENEB_BUILD_TENSOR_PIPELINE "Build Isaac ROS 5 / Lyrical GPU tensor pipeline" ON)
if(OPENEB_BUILD_TENSOR_PIPELINE)
  enable_language(CUDA)
  find_package(CUDAToolkit REQUIRED)
  find_package(Threads REQUIRED)
  ament_auto_add_library(openeb_tensor_components SHARED
    src/event_tensor_encoder_node.cpp
    src/async_event_tensor_preprocessor_node.cpp
    src/event_tensor_cuda_backend.cu)
  target_link_libraries(openeb_tensor_components CUDA::cudart MetavisionSDK::base Threads::Threads)
  set_target_properties(openeb_tensor_components PROPERTIES
    CUDA_STANDARD 17 CUDA_STANDARD_REQUIRED ON CUDA_SEPARABLE_COMPILATION ON)
  rclcpp_components_register_node(openeb_tensor_components
    PLUGIN "openeb_ros2::EventTensorEncoderNode" EXECUTABLE event_tensor_encoder_node)
  rclcpp_components_register_node(openeb_tensor_components
    PLUGIN "openeb_ros2::AsyncEventTensorPreprocessorNode" EXECUTABLE async_event_tensor_preprocessor_node)
  ament_auto_add_executable(openeb_tensor_pipeline src/tensor_pipeline_node.cpp)
  target_link_libraries(openeb_tensor_pipeline openeb_driver_component openeb_preprocessor_component openeb_tensor_components)
  if(BUILD_TESTING)
    add_executable(event_tensor_cuda_load_benchmark test/event_tensor_cuda_load_benchmark.cpp)
    target_include_directories(event_tensor_cuda_load_benchmark PRIVATE include)
    target_link_libraries(event_tensor_cuda_load_benchmark openeb_tensor_components CUDA::cudart)
    add_executable(histogram_bin_cuda_benchmark test/histogram_bin_cuda_benchmark.cpp)
    target_include_directories(histogram_bin_cuda_benchmark PRIVATE include)
    target_link_libraries(histogram_bin_cuda_benchmark openeb_tensor_components CUDA::cudart)
    add_executable(histogram_bin_cuda_test test/histogram_bin_cuda_test.cpp)
    target_include_directories(histogram_bin_cuda_test PRIVATE include)
    target_link_libraries(histogram_bin_cuda_test openeb_tensor_components CUDA::cudart)
    add_test(NAME histogram_bin_cuda_test COMMAND histogram_bin_cuda_test)
    add_executable(event_tensor_cuda_backend_test test/event_tensor_cuda_backend_test.cpp)
    target_include_directories(event_tensor_cuda_backend_test PRIVATE include)
    target_link_libraries(event_tensor_cuda_backend_test openeb_tensor_components CUDA::cudart)
    add_test(NAME event_tensor_cuda_backend_test COMMAND event_tensor_cuda_backend_test)
  endif()
endif()
if(BUILD_TESTING)
  add_executable(histogram_bin_updates_test test/histogram_bin_updates_test.cpp)
  target_include_directories(histogram_bin_updates_test PRIVATE include)
  add_test(NAME histogram_bin_updates_test COMMAND histogram_bin_updates_test)
  add_executable(evt3_runs_histogram_test test/evt3_runs_histogram_test.cpp)
  target_include_directories(evt3_runs_histogram_test PRIVATE include)
  add_test(NAME evt3_runs_histogram_test COMMAND evt3_runs_histogram_test)
  add_executable(evt3_runs_codec_equivalence_test test/evt3_runs_codec_equivalence_test.cpp)
  target_include_directories(evt3_runs_codec_equivalence_test PRIVATE include)
  ament_target_dependencies(evt3_runs_codec_equivalence_test event_camera_codecs)
  add_test(NAME evt3_runs_codec_equivalence_test COMMAND evt3_runs_codec_equivalence_test)
  add_executable(event_palette_test test/event_palette_test.cpp)
  target_include_directories(event_palette_test PRIVATE include)
  add_test(NAME event_palette_test COMMAND event_palette_test)
  add_executable(packet_router_test test/packet_router_test.cpp)
  target_include_directories(packet_router_test PRIVATE include)
  add_test(NAME packet_router_test COMMAND packet_router_test)
endif()
ament_auto_package()
