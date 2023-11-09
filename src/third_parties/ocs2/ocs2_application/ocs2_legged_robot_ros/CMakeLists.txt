# cmake_minimum_required(VERSION 3.8)
# project(ocs2_legged_robot_ros)

# # if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
# #   add_compile_options(-Wall -Wextra -Wpedantic)
# # endif()

# # find dependencies
# find_package(ament_cmake REQUIRED)
# find_package(urdf REQUIRED)
# find_package(kdl_parser REQUIRED)
# find_package(robot_state_publisher REQUIRED)
# find_package(rclcpp REQUIRED)
# find_package(visualization_msgs REQUIRED)
# find_package(geometry_msgs REQUIRED)

# # uncomment the following section in order to fill in
# # further dependencies manually.
# # find_package(<dependency> REQUIRED)
# find_package(Boost REQUIRED ALL)
# find_package(ocs2_core REQUIRED)
# find_package(ocs2_oc REQUIRED)
# find_package(ocs2_msgs REQUIRED)
# find_package(ocs2_ddp REQUIRED)
# find_package(ocs2_mpc REQUIRED)
# find_package(ocs2_sqp REQUIRED)
# find_package(ocs2_ipm REQUIRED)
# find_package(ocs2_robotic_tools REQUIRED)
# find_package(ocs2_pinocchio_interface REQUIRED)
# find_package(ocs2_centroidal_model REQUIRED)
# find_package(ocs2_robotic_assets REQUIRED)
# find_package(ocs2_ros_interfaces REQUIRED)
# find_package(ocs2_legged_robot REQUIRED)
# find_package(Eigen3 3.3 REQUIRED NO_MODULE)
# find_package(OpenMP REQUIRED)
# find_package(PkgConfig REQUIRED)
# pkg_check_modules(pinocchio REQUIRED pinocchio)

# set(AMENT_PACKAGE_DEPENDENCIES
#   rclcpp
#   urdf
#   kdl_parser
#   robot_state_publisher
#   visualization_msgs
#   geometry_msgs
#   ocs2_core
#   ocs2_oc
#   ocs2_msgs
#   ocs2_ddp
#   ocs2_mpc
#   ocs2_sqp
#   ocs2_ipm
#   ocs2_robotic_tools
#   ocs2_pinocchio_interface
#   ocs2_centroidal_model
#   ocs2_robotic_assets
#   ocs2_ros_interfaces
#   ocs2_legged_robot
#   pinocchio
# )
# include(cmake/pinocchio_config.cmake)

# # BLASFEO Option
# if(NOT TARGET blasfeo)
#   add_library(blasfeo SHARED IMPORTED)
#   set_target_properties(blasfeo PROPERTIES IMPORTED_LOCATION ${BLASFEO_PATH}/lib/libblasfeo.so)
# endif()

# file(GLOB_RECURSE source ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)
# add_library(${PROJECT_NAME} SHARED 
#   src/gait/GaitKeyboardPublisher.cpp
#   src/gait/GaitReceiver.cpp
#   src/visualization/LeggedRobotVisualizer.cpp)
# target_link_libraries(${PROJECT_NAME}
#     ${LIBS}
#     ${Boost_LIBRARIES}
#     ${pinocchio_LIBRARIES}
# )
# target_include_directories(${PROJECT_NAME}
#     PUBLIC
#     ${EIGEN3_INCLUDE_DIRS}
#     ${pinocchio_INCLUDE_DIRS}
#     $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#     $<INSTALL_INTERFACE:include>)
# target_compile_options(${PROJECT_NAME} PUBLIC ${FLAGS})   
# ament_target_dependencies(${PROJECT_NAME} ${AMENT_PACKAGE_DEPENDENCIES})
# ament_export_targets(${PROJECT_NAME}_Targets HAS_LIBRARY_TARGET)
# ament_export_dependencies(${AMENT_PACKAGE_DEPENDENCIES})

# install(DIRECTORY include/
#   DESTINATION include
# )
# install(
#   TARGETS ${PROJECT_NAME}
#   EXPORT ${PROJECT_NAME}_Targets
#   LIBRARY DESTINATION lib
#   ARCHIVE DESTINATION lib
#   RUNTIME DESTINATION bin
#   INCLUDES DESTINATION include
# )

# # DDP-MPC node for legged robot
# add_executable(legged_robot_ddp_mpc
#   src/LeggedRobotDdpMpcNode.cpp
# )
# ament_target_dependencies(legged_robot_ddp_mpc
#   ${AMENT_PACKAGE_DEPENDENCIES}
# )
# target_link_libraries(legged_robot_ddp_mpc
#   ${PROJECT_NAME}
#   ${LIBS}
#   ${pinocchio_LIBRARIES}
# )
# target_compile_options(legged_robot_ddp_mpc PRIVATE ${ocs2_CXX_FLAGS})

# ## SQP-MPC node for legged robot
# add_executable(legged_robot_sqp_mpc
#   src/LeggedRobotSqpMpcNode.cpp
# )
# ament_target_dependencies(legged_robot_sqp_mpc
#   ${AMENT_PACKAGE_DEPENDENCIES}
# )
# target_link_libraries(legged_robot_sqp_mpc
#   ${PROJECT_NAME}
#   ${LIBS}
#   ${pinocchio_LIBRARIES}
# )
# target_compile_options(legged_robot_sqp_mpc  PRIVATE ${ocs2_CXX_FLAGS})

# ## IPM-MPC node for legged robot
# add_executable(legged_robot_ipm_mpc
#   src/LeggedRobotIpmMpcNode.cpp
# )
# ament_target_dependencies(legged_robot_ipm_mpc
#   ${AMENT_PACKAGE_DEPENDENCIES}
# )
# target_link_libraries(legged_robot_ipm_mpc
#   ${PROJECT_NAME}
#   ${LIBS}
#   ${pinocchio_LIBRARIES}
# )
# target_compile_options(legged_robot_ipm_mpc  PRIVATE ${ocs2_CXX_FLAGS})

# # Dummy node
# add_executable(legged_robot_dummy
#   src/LeggedRobotDummyNode.cpp
# )
# ament_target_dependencies(legged_robot_dummy
#   ${AMENT_PACKAGE_DEPENDENCIES}
# )
# target_link_libraries(legged_robot_dummy
#   ${PROJECT_NAME}
#   ${LIBS}
#   ${pinocchio_LIBRARIES}
# )
# target_compile_options(legged_robot_dummy PRIVATE ${ocs2_CXX_FLAGS})

# # Keyboard command node for target trajectories
# add_executable(legged_robot_target
#   src/LeggedRobotPoseCommandNode.cpp
# )
# ament_target_dependencies(legged_robot_target
#   ${AMENT_PACKAGE_DEPENDENCIES}
# )
# target_link_libraries(legged_robot_target
#   ${PROJECT_NAME}
#   ${LIBS}
#   ${pinocchio_LIBRARIES}
# )
# target_compile_options(legged_robot_target PRIVATE ${ocs2_CXX_FLAGS})

# # Keyboard command node for gait
# add_executable(legged_robot_gait_command
#   src/LeggedRobotGaitCommandNode.cpp
# )
# ament_target_dependencies(legged_robot_gait_command
#   ${AMENT_PACKAGE_DEPENDENCIES}
# )
# target_link_libraries(legged_robot_gait_command
#   ${PROJECT_NAME}
#   ${LIBS}
#   ${pinocchio_LIBRARIES}
# )
# target_compile_options(legged_robot_gait_command PRIVATE ${ocs2_CXX_FLAGS})


# install(
#   TARGETS
#     legged_robot_ddp_mpc
#     legged_robot_sqp_mpc
#     legged_robot_ipm_mpc
#     legged_robot_dummy
#     legged_robot_target
#     legged_robot_gait_command
#   RUNTIME DESTINATION lib/${PROJECT_NAME}
# )
# install(DIRECTORY launch rviz
#   DESTINATION share/${PROJECT_NAME}
# )

# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   # the following line skips the linter which checks for copyrights
#   # comment the line when a copyright and license is added to all source files
#   set(ament_cmake_copyright_FOUND TRUE)
#   # the following line skips cpplint (only works in a git repo)
#   # comment the line when this package is in a git repo and when
#   # a copyright and license is added to all source files
#   set(ament_cmake_cpplint_FOUND TRUE)
#   ament_lint_auto_find_test_dependencies()
# endif()

# ament_package()
