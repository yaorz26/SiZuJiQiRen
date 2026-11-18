cmake_minimum_required(VERSION 3.8)
project(go2_exploration)
find_package(ament_cmake REQUIRED)
install(PROGRAMS scripts/frontier_explorer.py scripts/indoor_sim.py scripts/run_sim.py
  scripts/guarded_velocity_bridge.py
  DESTINATION lib/${PROJECT_NAME})
install(FILES scripts/frontiers.py DESTINATION lib/${PROJECT_NAME})
if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(frontiers test/test_frontiers.py)
  ament_add_pytest_test(velocity_bridge test/test_velocity_bridge.py)
endif()
ament_package()
