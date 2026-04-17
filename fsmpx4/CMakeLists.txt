cmake_minimum_required(VERSION 3.8)
project(fsmpx4)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(swarm_planner REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/FSMDebug.msg"
  DEPENDENCIES std_msgs geometry_msgs
)

# ── Static libraries ──

add_library(fsmpx4_input STATIC src/input.cpp)
set_target_properties(fsmpx4_input PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(fsmpx4_input PUBLIC include ${EIGEN3_INCLUDE_DIRS} ${swarm_planner_INCLUDE_DIRS})
ament_target_dependencies(fsmpx4_input rclcpp geometry_msgs sensor_msgs nav_msgs std_msgs px4_msgs Eigen3 swarm_planner)

add_library(fsmpx4_control STATIC src/control.cpp)
set_target_properties(fsmpx4_control PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(fsmpx4_control PUBLIC include ${EIGEN3_INCLUDE_DIRS} ${swarm_planner_INCLUDE_DIRS})
ament_target_dependencies(fsmpx4_control rclcpp geometry_msgs sensor_msgs nav_msgs std_msgs px4_msgs Eigen3 swarm_planner)

add_library(fsmpx4_param_loader STATIC src/param_loader.cpp)
set_target_properties(fsmpx4_param_loader PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(fsmpx4_param_loader PUBLIC include ${EIGEN3_INCLUDE_DIRS} ${swarm_planner_INCLUDE_DIRS})
ament_target_dependencies(fsmpx4_param_loader rclcpp Eigen3 swarm_planner)

# ── FSM Component (shared library) ──

add_library(fsmpx4_fsm SHARED src/fsmpx4.cpp src/fsmpx4_debug.cpp)
target_include_directories(fsmpx4_fsm PUBLIC include ${EIGEN3_INCLUDE_DIRS} ${swarm_planner_INCLUDE_DIRS})
target_link_libraries(fsmpx4_fsm
  fsmpx4_input
  fsmpx4_control
  fsmpx4_param_loader
  ${swarm_planner_LIBRARIES})
ament_target_dependencies(fsmpx4_fsm rclcpp rclcpp_components geometry_msgs sensor_msgs px4_msgs Eigen3 swarm_planner)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(fsmpx4_fsm "${cpp_typesupport_target}")
rclcpp_components_register_nodes(fsmpx4_fsm "fsmpx4::FSMPX4")

# ── Standalone executables ──

add_executable(fsm_node src/fsm_node.cpp)
target_link_libraries(fsm_node fsmpx4_fsm)
ament_target_dependencies(fsm_node rclcpp)

# ── Install ──

install(TARGETS
  fsmpx4_input fsmpx4_control fsmpx4_param_loader
  fsmpx4_fsm
  fsm_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)
install(DIRECTORY script/ DESTINATION share/${PROJECT_NAME}/script
  PATTERN "__pycache__" EXCLUDE PATTERN "*.pyc" EXCLUDE)
install(PROGRAMS
  script/auto_arm.py
  script/rc.py
  script/analyze_swarm_bag
  script/swarm_enu_rviz.py
  DESTINATION lib/${PROJECT_NAME}
)

# ── Testing ──

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(fsmpx4_fsm)
ament_export_dependencies(
  rclcpp rclcpp_components geometry_msgs sensor_msgs nav_msgs std_msgs px4_msgs Eigen3 rosidl_default_runtime swarm_planner)

ament_package()
