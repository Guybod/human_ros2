cmake_minimum_required(VERSION 3.8)
project(codroid_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  launch
  rviz
  urdf
  meshes
  docs
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

ament_package()
