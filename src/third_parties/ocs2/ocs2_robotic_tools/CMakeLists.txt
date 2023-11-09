cmake_minimum_required(VERSION 3.8)
project(ocs2_robotic_tools)

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
find_package(ocs2_core REQUIRED)
find_package(ocs2_oc REQUIRED)

file(GLOB_RECURSE source ${CMAKE_CURRENT_SOURCE_DIR}/src/common/*.cpp)
add_library(${PROJECT_NAME} SHARED ${source})
target_link_libraries(${PROJECT_NAME}
    ${LIBS}
)
target_include_directories(${PROJECT_NAME}
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
target_compile_options(${PROJECT_NAME} PUBLIC ${ocs2_CXX_FLAGS})   

ament_target_dependencies(${PROJECT_NAME} ocs2_core ocs2_oc)
ament_export_targets(${PROJECT_NAME}_Targets HAS_LIBRARY_TARGET)
ament_export_dependencies(ocs2_core ocs2_oc)

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

add_executable(${PROJECT_NAME}_lintTarget
  src/lintTarget.cpp
)
target_include_directories(${PROJECT_NAME}_lintTarget
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
ament_target_dependencies(${PROJECT_NAME}_lintTarget ocs2_core ocs2_oc)

ament_package()
