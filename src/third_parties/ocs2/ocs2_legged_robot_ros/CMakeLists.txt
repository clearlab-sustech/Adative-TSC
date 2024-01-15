cmake_minimum_required(VERSION 3.8)
project(ocs2_legged_robot_ros)

# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#   add_compile_options(-Wall -Wextra -Wpedantic)
# endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(urdf REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(robot_state_publisher REQUIRED)
find_package(rclcpp REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(ocs2_core REQUIRED)
find_package(ocs2_oc REQUIRED)
find_package(ocs2_msgs REQUIRED)
find_package(ocs2_mpc REQUIRED)
find_package(ocs2_sqp REQUIRED)
find_package(ocs2_robotic_tools REQUIRED)
find_package(ocs2_pinocchio_interface REQUIRED)
find_package(ocs2_centroidal_model REQUIRED)
find_package(ocs2_ros_interfaces REQUIRED)
find_package(ocs2_legged_robot REQUIRED)

set(AMENT_PACKAGE_DEPENDENCIES
  rclcpp
  urdf
  kdl_parser
  robot_state_publisher
  visualization_msgs
  geometry_msgs
  ocs2_core
  ocs2_oc
  ocs2_msgs
  ocs2_mpc
  ocs2_sqp
  ocs2_robotic_tools
  ocs2_pinocchio_interface
  ocs2_centroidal_model
  ocs2_ros_interfaces
  ocs2_legged_robot
  pinocchio
)

# BLASFEO Option
if(NOT TARGET blasfeo)
  add_library(blasfeo SHARED IMPORTED)
  set_target_properties(blasfeo PROPERTIES IMPORTED_LOCATION ${BLASFEO_PATH}/lib/libblasfeo.so)
endif()

file(GLOB_RECURSE source ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)
add_library(${PROJECT_NAME} SHARED 
  src/gait/GaitKeyboardPublisher.cpp
  src/gait/GaitReceiver.cpp
  src/visualization/LeggedRobotVisualizer.cpp)
target_link_libraries(${PROJECT_NAME}
    ${LIBS}
    ${Boost_LIBRARIES}
    ${pinocchio_LIBRARIES}
)
target_include_directories(${PROJECT_NAME}
    PUBLIC
    ${EIGEN3_INCLUDE_DIRS}
    ${pinocchio_INCLUDE_DIRS}
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
target_compile_options(${PROJECT_NAME} PUBLIC ${FLAGS})   
ament_target_dependencies(${PROJECT_NAME} ${AMENT_PACKAGE_DEPENDENCIES})
ament_export_targets(${PROJECT_NAME}_Targets HAS_LIBRARY_TARGET)
ament_export_dependencies(${AMENT_PACKAGE_DEPENDENCIES})

install(DIRECTORY include/
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

## SQP-MPC node for legged robot
add_executable(legged_robot_sqp_mpc
  src/LeggedRobotSqpMpcNode.cpp
)
ament_target_dependencies(legged_robot_sqp_mpc
  ${AMENT_PACKAGE_DEPENDENCIES}
)
target_link_libraries(legged_robot_sqp_mpc
  ${PROJECT_NAME}
  ${LIBS}
  ${pinocchio_LIBRARIES}
)
target_compile_options(legged_robot_sqp_mpc  PRIVATE ${OCS2_CXX_FLAGS})

# Dummy node
add_executable(legged_robot_dummy
  src/LeggedRobotDummyNode.cpp
)
ament_target_dependencies(legged_robot_dummy
  ${AMENT_PACKAGE_DEPENDENCIES}
)
target_link_libraries(legged_robot_dummy
  ${PROJECT_NAME}
  ${LIBS}
  ${pinocchio_LIBRARIES}
)
target_compile_options(legged_robot_dummy PRIVATE ${OCS2_CXX_FLAGS})

# Keyboard command node for target trajectories
add_executable(legged_robot_target
  src/LeggedRobotPoseCommandNode.cpp
)
ament_target_dependencies(legged_robot_target
  ${AMENT_PACKAGE_DEPENDENCIES}
)
target_link_libraries(legged_robot_target
  ${PROJECT_NAME}
  ${LIBS}
  ${pinocchio_LIBRARIES}
)
target_compile_options(legged_robot_target PRIVATE ${OCS2_CXX_FLAGS})

# Keyboard command node for gait
add_executable(legged_robot_gait_command
  src/LeggedRobotGaitCommandNode.cpp
)
ament_target_dependencies(legged_robot_gait_command
  ${AMENT_PACKAGE_DEPENDENCIES}
)
target_link_libraries(legged_robot_gait_command
  ${PROJECT_NAME}
  ${LIBS}
  ${pinocchio_LIBRARIES}
)
target_compile_options(legged_robot_gait_command PRIVATE ${OCS2_CXX_FLAGS})


install(
  TARGETS
    legged_robot_sqp_mpc
    legged_robot_dummy
    legged_robot_target
    legged_robot_gait_command
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
