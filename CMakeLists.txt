cmake_minimum_required(VERSION 3.8)
project(v4l2_publisher)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(stereo_image_publisher src/stereo_compressed_publisher.cpp)
target_compile_features(stereo_image_publisher PUBLIC cxx_std_17)
target_include_directories(stereo_image_publisher PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(stereo_image_publisher ${OpenCV_LIBS})
ament_target_dependencies(stereo_image_publisher rclcpp sensor_msgs)

add_executable(camera_backend_probe src/camera_backend_probe.cpp)
target_compile_features(camera_backend_probe PUBLIC cxx_std_17)
target_include_directories(camera_backend_probe PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(camera_backend_probe ${OpenCV_LIBS})

add_library(rectification STATIC src/rectification.cpp)
target_compile_features(rectification PUBLIC cxx_std_17)
target_compile_options(rectification PRIVATE -fPIC)
target_include_directories(rectification PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(rectification ${OpenCV_LIBS})

# IMU device library (C++ wrapper over C HAL)
add_library(imu_device STATIC
  src/imu_device.cpp
  src/imu_icm_45686.cpp
  src/i2c_drive_linux.c
)
target_compile_features(imu_device PUBLIC cxx_std_17)
set_source_files_properties(src/i2c_drive_linux.c PROPERTIES LANGUAGE C)
target_include_directories(imu_device PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_include_directories(imu_device PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/src
)

# Shared V4L2 backend library (capture + clock offset)
add_library(v4l2_publisher_backend STATIC
  src/v4l2_capture.cpp
  src/clock_offset.cpp
)
target_compile_features(v4l2_publisher_backend PUBLIC cxx_std_17)
target_compile_options(v4l2_publisher_backend PRIVATE -fPIC)
target_include_directories(v4l2_publisher_backend PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(v4l2_publisher_backend ${OpenCV_LIBS})

# Composable node shared library (loaded by rclcpp_components)
add_library(single_camera_node_component SHARED
  src/single_camera_node.cpp
)
target_compile_features(single_camera_node_component PUBLIC cxx_std_17)
target_include_directories(single_camera_node_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(single_camera_node_component
  v4l2_publisher_backend
  rectification
  ${OpenCV_LIBS}
)
ament_target_dependencies(single_camera_node_component
  rclcpp
  rclcpp_components
  sensor_msgs
)
rclcpp_components_register_nodes(single_camera_node_component
  "v4l2_publisher::SingleCameraNode"
)

# Composable IMU node shared library
add_library(imu_node_component SHARED
  src/imu_node.cpp
)
target_compile_features(imu_node_component PUBLIC cxx_std_17)
target_include_directories(imu_node_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(imu_node_component imu_device)
ament_target_dependencies(imu_node_component
  rclcpp
  rclcpp_components
  sensor_msgs
)
rclcpp_components_register_nodes(imu_node_component
  "v4l2_publisher::ImuNode"
)

# Standalone IMU node executable
add_executable(imu_node_exe src/imu_node_main.cpp)
target_compile_features(imu_node_exe PUBLIC cxx_std_17)
target_include_directories(imu_node_exe PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_link_libraries(imu_node_exe imu_node_component)
ament_target_dependencies(imu_node_exe rclcpp rclcpp_components)

# Composable stereo camera node shared library
add_library(stereo_camera_node_component SHARED
  src/stereo_camera_node.cpp
)
target_compile_features(stereo_camera_node_component PUBLIC cxx_std_17)
target_include_directories(stereo_camera_node_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(stereo_camera_node_component
  v4l2_publisher_backend
  rectification
  ${OpenCV_LIBS}
)
ament_target_dependencies(stereo_camera_node_component
  rclcpp
  rclcpp_components
  sensor_msgs
)
rclcpp_components_register_nodes(stereo_camera_node_component
  "v4l2_publisher::StereoCameraNode"
)

# Standalone executable entry point
add_executable(single_camera_node_exe src/single_camera_node_main.cpp)
target_compile_features(single_camera_node_exe PUBLIC cxx_std_17)
target_include_directories(single_camera_node_exe PRIVATE
  ${OpenCV_INCLUDE_DIRS}
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_link_libraries(single_camera_node_exe
  single_camera_node_component
  ${OpenCV_LIBS}
)
ament_target_dependencies(single_camera_node_exe rclcpp rclcpp_components)

install(TARGETS
  stereo_image_publisher
  camera_backend_probe
  rectification
  imu_device
  v4l2_publisher_backend
  single_camera_node_component
  stereo_camera_node_component
  single_camera_node_exe
  imu_node_component
  imu_node_exe
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/start_imu_after_camera_ready.py
  imu_ros2_pub_bin/imu_ros2_pub_test
  DESTINATION lib/${PROJECT_NAME}
)

install(FILES
  imu_ros2_pub_bin/libSDK.so
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
