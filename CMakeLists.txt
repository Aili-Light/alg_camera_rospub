cmake_minimum_required(VERSION 3.8)
project(v4l2_publisher)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
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

install(TARGETS
  stereo_image_publisher
  camera_backend_probe
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
