cmake_minimum_required(VERSION 3.8)
project(swarm_planner)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/SwarmPlannerDebug.msg"
  DEPENDENCIES std_msgs geometry_msgs
)

add_library(swarm_planner_core_lib STATIC
  src/geo_utils.cpp
  src/swarm_planner_core.cpp
)
set_target_properties(swarm_planner_core_lib PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(swarm_planner_core_lib PUBLIC include ${EIGEN3_INCLUDE_DIRS})
ament_target_dependencies(swarm_planner_core_lib rclcpp Eigen3)

install(TARGETS
  swarm_planner_core_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include)
install(PROGRAMS
  script/analyze_swarm_debug
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_swarm_planner_core test/test_swarm_planner_core.cpp)
  if(TARGET test_swarm_planner_core)
    target_include_directories(test_swarm_planner_core PRIVATE include ${EIGEN3_INCLUDE_DIRS})
    target_link_libraries(test_swarm_planner_core swarm_planner_core_lib)
    ament_target_dependencies(test_swarm_planner_core rclcpp Eigen3)
  endif()

  ament_add_gtest(test_swarm_geo_utils test/test_geo_utils.cpp)
  if(TARGET test_swarm_geo_utils)
    target_include_directories(test_swarm_geo_utils PRIVATE include ${EIGEN3_INCLUDE_DIRS})
    target_link_libraries(test_swarm_geo_utils swarm_planner_core_lib)
    ament_target_dependencies(test_swarm_geo_utils rclcpp Eigen3)
  endif()
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(swarm_planner_core_lib)
ament_export_dependencies(rclcpp geometry_msgs std_msgs Eigen3 rosidl_default_runtime)

ament_package()
