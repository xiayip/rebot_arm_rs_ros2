cmake_minimum_required(VERSION 3.22)
project(rebot_arm_rs_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch meshes rviz urdf usd
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
