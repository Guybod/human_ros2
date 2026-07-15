cmake_minimum_required(VERSION 3.8)
project(codroid_description)

set(CMAKE_AUTOMOC ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)
find_package(rclcpp REQUIRED)
find_package(rviz_common REQUIRED)
find_package(std_srvs REQUIRED)

add_library(codroid_arm_control_panel SHARED
  include/codroid_description/arm_control_panel.hpp
  src/arm_control_panel.cpp
)
target_include_directories(codroid_arm_control_panel PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(codroid_arm_control_panel
  geometry_msgs
  pluginlib
  rclcpp
  rviz_common
  std_srvs
)
target_link_libraries(codroid_arm_control_panel Qt5::Widgets)
pluginlib_export_plugin_description_file(rviz_common rviz_plugin_description.xml)

install(DIRECTORY
  launch
  rviz
  urdf
  meshes
  docs
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(TARGETS codroid_arm_control_panel
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(FILES rviz_plugin_description.xml
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/codroid_arm_bridge.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME codroid_arm_bridge
)

install(FILES
  scripts/codroid_kinematics.py
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/codroid_arm_trajectory.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME codroid_arm_trajectory
)

install(PROGRAMS
  scripts/codroid_wave_demo.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME codroid_wave_demo
)

ament_export_include_directories(include)
ament_export_libraries(codroid_arm_control_panel)
ament_export_dependencies(geometry_msgs pluginlib rclcpp rviz_common std_srvs)

ament_package()
