cmake_minimum_required(VERSION 3.8)
project(go2_tf_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(go2_tf_bridge src/go2_tf_bridge.cpp)
target_include_directories(go2_tf_bridge PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(go2_tf_bridge PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(
  go2_tf_bridge
  "geometry_msgs"
  "nav_msgs"
  "rclcpp"
  "tf2"
  "tf2_ros"
)

install(TARGETS go2_tf_bridge
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
