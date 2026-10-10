cmake_minimum_required(VERSION 3.8)
project(go2_cloud_map_projector)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(go2_cloud_map_projector src/go2_cloud_map_projector.cpp)
target_include_directories(go2_cloud_map_projector PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(go2_cloud_map_projector PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(
  go2_cloud_map_projector
  "nav_msgs"
  "rclcpp"
  "sensor_msgs"
)

install(TARGETS go2_cloud_map_projector
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
