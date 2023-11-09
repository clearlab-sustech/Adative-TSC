cmake_minimum_required(VERSION 3.8)
project(ocs2_ros_interfaces)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ocs2_msgs REQUIRED)
find_package(ocs2_core REQUIRED)
find_package(ocs2_mpc REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(ocs2_centroidal_model REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(interactive_markers REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(PkgConfig REQUIRED)
pkg_check_modules(pinocchio REQUIRED pinocchio)
include(cmake/pinocchio_config.cmake)

set(AMENT_PACKAGE_DEPENDENCIES
    rclcpp
    ocs2_msgs
    ocs2_core
    ocs2_mpc
    std_msgs
    visualization_msgs
    geometry_msgs
    nav_msgs
    ocs2_centroidal_model
    pinocchio
    interactive_markers
)

file(GLOB_RECURSE source ${CMAKE_CURRENT_SOURCE_DIR}/src/ocs2_ros_interfaces/*.cpp)
add_library(${PROJECT_NAME} SHARED ${source})
target_link_libraries(${PROJECT_NAME}
    ${LIBS}
    ${pinocchio_LIBRARIES}
)
target_include_directories(${PROJECT_NAME}
    PUBLIC
    ${EIGEN3_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
target_compile_options(${PROJECT_NAME} PUBLIC ${ocs2_CXX_FLAGS})   

ament_target_dependencies(${PROJECT_NAME} ${AMENT_PACKAGE_DEPENDENCIES})
ament_export_targets(${PROJECT_NAME}_Targets HAS_LIBRARY_TARGET)
ament_export_dependencies(${AMENT_PACKAGE_DEPENDENCIES})

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}_Targets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

ament_package()
