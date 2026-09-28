cmake_minimum_required(VERSION 3.22)
project(rebot_arm_rs_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY LICENSES launch meshes rviz urdf usd
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_description test/test_description.py)
endif()

ament_package()
