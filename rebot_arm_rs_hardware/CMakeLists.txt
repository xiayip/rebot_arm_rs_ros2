cmake_minimum_required(VERSION 3.22)
project(rebot_arm_rs_hardware)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

add_library(rebot_arm_rs_protocol STATIC
  src/robstride_protocol.cpp
  src/socketcan_transport.cpp
)
set_target_properties(rebot_arm_rs_protocol PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(rebot_arm_rs_protocol PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_library(${PROJECT_NAME} SHARED src/rebot_arm_rs_system.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME} rebot_arm_rs_protocol)
ament_target_dependencies(${PROJECT_NAME}
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

pluginlib_export_plugin_description_file(hardware_interface rebot_arm_rs_hardware.xml)

install(TARGETS rebot_arm_rs_protocol ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_robstride_protocol test/test_robstride_protocol.cpp)
  target_link_libraries(test_robstride_protocol rebot_arm_rs_protocol)
endif()

ament_export_include_directories(include)
ament_export_libraries(rebot_arm_rs_protocol ${PROJECT_NAME})
ament_export_dependencies(hardware_interface pluginlib rclcpp rclcpp_lifecycle)
ament_package()

