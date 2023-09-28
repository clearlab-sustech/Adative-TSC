cmake_minimum_required(VERSION 3.8)
project(visualization)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(core REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(pinocchio_interface REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(pinocchio REQUIRED pinocchio)
include(cmake/pinocchio_config.cmake)

find_package(trans REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(asserts REQUIRED)

set(TARGET_DEPENDENCE
  ament_cmake
  rclcpp
  core
  pinocchio_interface
  geometry_msgs
  sensor_msgs
  tf2_ros
  visualization_msgs
  trans
  yaml-cpp
  asserts
)

file(GLOB_RECURSE source ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cc)
add_executable(${PROJECT_NAME} ${source})
target_link_libraries(${PROJECT_NAME}
  ${pinocchio_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
)
target_compile_options(${PROJECT_NAME} PUBLIC ${DEEPBREAK_CXX_FLAGS})
target_include_directories(${PROJECT_NAME}
  PUBLIC
  ${EIGEN3_INCLUDE_DIRS}
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(${PROJECT_NAME} ${TARGET_DEPENDENCE})
ament_export_dependencies(${TARGET_DEPENDENCE})

install(TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME})

ament_package()
