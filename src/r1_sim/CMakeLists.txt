cmake_minimum_required(VERSION 3.8)
project(r1_sim)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  models
  worlds
  launch
  config
  maps
  rviz
  scripts
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/sdf_to_urdf.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
