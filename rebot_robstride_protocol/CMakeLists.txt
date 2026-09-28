cmake_minimum_required(VERSION 3.22)
project(rebot_robstride_protocol)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(pluginlib REQUIRED)
find_package(shared_can_interface REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/robstride_codec.cpp
  src/robstride_protocol.cpp
  src/rebot_gripper_protocol.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME} pluginlib shared_can_interface)

pluginlib_export_plugin_description_file(shared_can_interface rebot_robstride_protocol.xml)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_robstride_protocol test/test_robstride_protocol.cpp)
  target_link_libraries(test_robstride_protocol ${PROJECT_NAME})
  ament_target_dependencies(test_robstride_protocol shared_can_interface)
  ament_add_gtest(test_startup_readiness test/test_startup_readiness.cpp)
  target_link_libraries(test_startup_readiness ${PROJECT_NAME})
  ament_target_dependencies(test_startup_readiness shared_can_interface)
  ament_add_gtest(test_rebot_gripper_protocol test/test_rebot_gripper_protocol.cpp)
  target_link_libraries(test_rebot_gripper_protocol ${PROJECT_NAME})
  ament_target_dependencies(test_rebot_gripper_protocol shared_can_interface)
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(shared_can_interface)
ament_package()
