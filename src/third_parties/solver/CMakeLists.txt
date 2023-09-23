cmake_minimum_required(VERSION 3.5)
project(solver)

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
find_package(core REQUIRED)
find_package(blasfeo_ament REQUIRED)
find_package(hpipm_ament REQUIRED)
find_package(yaml-cpp REQUIRED)

set(TARGET_DEPENDENCE
  ament_cmake
  rclcpp
  core
  blasfeo_ament
  hpipm_ament
  yaml-cpp
)

file(GLOB_RECURSE source ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cc)
add_library(${PROJECT_NAME} SHARED ${source})
target_link_libraries(${PROJECT_NAME}
  ${YAML_CPP_LIBRARIES}
)
target_compile_options(${PROJECT_NAME} PUBLIC ${PINOCCHIO_FLAGS})

target_include_directories(${PROJECT_NAME}
  PUBLIC
  ${pinocchio_INCLUDE_DIRS}
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(${PROJECT_NAME} ${TARGET_DEPENDENCE})
ament_export_targets(${PROJECT_NAME}_Targets HAS_LIBRARY_TARGET)
ament_export_dependencies(${TARGET_DEPENDENCE})

add_executable(test_qp test/test.cc)
target_include_directories(test_qp PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_link_libraries(test_qp
  ${PROJECT_NAME}
  ${pinocchio_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
)

ament_target_dependencies(test_qp ${TARGET_DEPENDENCE})
ament_export_dependencies(${TARGET_DEPENDENCE})

install(TARGETS test_qp
  DESTINATION lib/${PROJECT_NAME})

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
