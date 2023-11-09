cmake_minimum_required(VERSION 3.5)
project(ocs2_msgs)

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
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

file(GLOB_RECURSE source ${CMAKE_CURRENT_SOURCE_DIR}/msg/* ${CMAKE_CURRENT_SOURCE_DIR}/srv/* )
rosidl_generate_interfaces(${PROJECT_NAME}
        msg/MpcState.msg
        msg/MpcInput.msg
        msg/ModeSchedule.msg
        msg/MpcObservation.msg
        msg/MpcPerformanceIndices.msg
        msg/MpcTargetTrajectories.msg
        msg/ControllerData.msg
        msg/MpcFlattenedController.msg
        msg/LagrangianMetrics.msg
        msg/Multiplier.msg
        msg/Constraint.msg
        msg/ActuatorCmds.msg
        srv/Reset.srv
        srv/SimReset.srv
        DEPENDENCIES std_msgs builtin_interfaces
        )

ament_package()