cmake_minimum_required(VERSION 3.8)
project(joystick_rsu)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(rsu_link_plotter_node src/rsu_link_plotter_node.cpp)
ament_target_dependencies(rsu_link_plotter_node
  rclcpp
  visualization_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  geometry_msgs
)

install(TARGETS rsu_link_plotter_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config robot_model
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/joy_joint_state_node.py
  DESTINATION lib/${PROJECT_NAME}
)

install(FILES
  scripts/util/gamepad_reader.py
  DESTINATION lib/${PROJECT_NAME}
)

install(FILES
  scripts/util/rsu_solver.py
  DESTINATION lib/${PROJECT_NAME}
)

install(FILES
  scripts/test_rsu_solver.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
