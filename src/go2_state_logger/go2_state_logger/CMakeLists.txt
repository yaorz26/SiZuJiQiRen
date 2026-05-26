cmake_minimum_required(VERSION 3.8)
project(go2_state_logger)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(unitree_go REQUIRED)
find_package(nav_msgs REQUIRED)

add_executable(go2_state_logger src/go2_state_logger.cpp)
target_include_directories(go2_state_logger PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(go2_state_logger PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(
  go2_state_logger
  "rclcpp"
  "unitree_go"
  "nav_msgs"
)

install(TARGETS go2_state_logger
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
