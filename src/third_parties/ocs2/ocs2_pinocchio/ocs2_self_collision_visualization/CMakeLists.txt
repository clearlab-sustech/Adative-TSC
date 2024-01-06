cmake_minimum_required(VERSION 3.8)
project(ocs2_self_collision_visualization)

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
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

find_package(ocs2_core REQUIRED)
find_package(ocs2_robotic_tools REQUIRED)
find_package(ocs2_pinocchio_interface REQUIRED)
find_package(ocs2_ros_interfaces REQUIRED)
find_package(ocs2_self_collision REQUIRED)

file(GLOB_RECURSE source ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)
add_library(${PROJECT_NAME} SHARED ${source})
target_link_libraries(${PROJECT_NAME}
    ${LIBS}
    ${pinocchio_LIBRARIES}  
)
target_include_directories(${PROJECT_NAME}
    PUBLIC
    ${pinocchio_INCLUDE_DIRS}
    ${hpp-fcl_INCLUDE_DIRS}
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
target_compile_options(${PROJECT_NAME} PUBLIC ${ocs2_PINOCCHIO_FLAGS})   

ament_target_dependencies(${PROJECT_NAME} 
  rclcpp  
  std_msgs
  geometry_msgs
  visualization_msgs
  ocs2_core 
  ocs2_robotic_tools 
  ocs2_pinocchio_interface  
  ocs2_self_collision
  pinocchio)
ament_export_targets(${PROJECT_NAME}_Targets HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp std_msgs 
geometry_msgs visualization_msgs
ocs2_core ocs2_robotic_tools ocs2_pinocchio_interface ocs2_self_collision pinocchio)

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
