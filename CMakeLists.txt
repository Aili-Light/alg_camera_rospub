cmake_minimum_required(VERSION 3.8)
project(alg_camera_rospub)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)

# --- Include directories ---
include_directories(include)
include_directories(src)   # for C HAL headers

# --- Static library: V4L2 capture (ROS-agnostic) ---
add_library(v4l2_capture STATIC
  src/v4l2_capture.cpp
  src/clock_offset.cpp
)
target_compile_features(v4l2_capture PUBLIC cxx_std_17)
target_compile_options(v4l2_capture PRIVATE -fPIC)
target_include_directories(v4l2_capture PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_link_libraries(v4l2_capture PUBLIC ${OpenCV_LIBS})

# --- Static library: I2C IMU driver ---
add_library(i2c_imu STATIC
  src/i2c_drive_linux.c
  src/imu_device.cpp
  src/imu_icm_45686.cpp
)
target_compile_features(i2c_imu PUBLIC cxx_std_17)
target_compile_options(i2c_imu PRIVATE -fPIC)
target_include_directories(i2c_imu PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
)

# --- Component: Single Camera ---
add_library(single_camera_component SHARED src/single_camera_node.cpp)
target_compile_features(single_camera_component PUBLIC cxx_std_17)
target_link_libraries(single_camera_component v4l2_capture)
target_include_directories(single_camera_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
ament_target_dependencies(single_camera_component
  rclcpp rclcpp_components sensor_msgs OpenCV
)

# --- Component: Stereo Camera ---
add_library(stereo_camera_component SHARED src/stereo_camera_node.cpp)
target_compile_features(stereo_camera_component PUBLIC cxx_std_17)
target_link_libraries(stereo_camera_component v4l2_capture)
target_include_directories(stereo_camera_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
ament_target_dependencies(stereo_camera_component
  rclcpp rclcpp_components sensor_msgs OpenCV
)

# --- Component: IMU (uses ClockOffset from v4l2_capture) ---
add_library(imu_component SHARED src/imu_node.cpp)
target_compile_features(imu_component PUBLIC cxx_std_17)
target_link_libraries(imu_component i2c_imu v4l2_capture)
target_include_directories(imu_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
ament_target_dependencies(imu_component
  rclcpp rclcpp_components sensor_msgs
)

# --- Standalone executables ---
add_executable(single_camera_publisher src/single_camera_main.cpp)
target_link_libraries(single_camera_publisher single_camera_component)
ament_target_dependencies(single_camera_publisher rclcpp sensor_msgs)
target_include_directories(single_camera_publisher PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

add_executable(stereo_camera_publisher src/stereo_camera_main.cpp)
target_link_libraries(stereo_camera_publisher stereo_camera_component)
ament_target_dependencies(stereo_camera_publisher rclcpp sensor_msgs)
target_include_directories(stereo_camera_publisher PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

add_executable(imu_publisher src/imu_main.cpp)
target_link_libraries(imu_publisher imu_component)
ament_target_dependencies(imu_publisher rclcpp sensor_msgs)
target_include_directories(imu_publisher PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

# Ensure RPATH is set so executables can find their companion .so in the install tree
set_target_properties(single_camera_publisher PROPERTIES
  INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}"
  BUILD_WITH_INSTALL_RPATH FALSE
  INSTALL_RPATH_USE_LINK_PATH TRUE
)
set_target_properties(stereo_camera_publisher PROPERTIES
  INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}"
  BUILD_WITH_INSTALL_RPATH FALSE
  INSTALL_RPATH_USE_LINK_PATH TRUE
)
set_target_properties(imu_publisher PROPERTIES
  INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}"
  BUILD_WITH_INSTALL_RPATH FALSE
  INSTALL_RPATH_USE_LINK_PATH TRUE
)

# --- Install targets ---
install(TARGETS
  v4l2_capture
  i2c_imu
  single_camera_component
  stereo_camera_component
  imu_component
  single_camera_publisher
  stereo_camera_publisher
  imu_publisher
  ARCHIVE DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib/${PROJECT_NAME}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# --- Component registration ---
rclcpp_components_register_nodes(single_camera_component
  "alg_camera_rospub::SingleCameraNode"
)

rclcpp_components_register_nodes(stereo_camera_component
  "alg_camera_rospub::StereoCameraNode"
)

rclcpp_components_register_nodes(imu_component
  "alg_camera_rospub::ImuNode"
)

install(PROGRAMS
  scripts/validate_topics.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
