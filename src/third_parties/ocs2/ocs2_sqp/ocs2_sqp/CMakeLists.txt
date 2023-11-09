cmake_minimum_required(VERSION 3.8)
project(ocs2_sqp)

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
find_package(ocs2_mpc REQUIRED)
find_package(ocs2_oc REQUIRED)
find_package(hpipm_ament REQUIRED)

set(TARGET_DEPENDENCE ocs2_core ocs2_mpc ocs2_oc hpipm_ament)

file(GLOB_RECURSE source ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)
add_library(${PROJECT_NAME} SHARED ${source})
target_link_libraries(${PROJECT_NAME}
    ${LIBS}
)
target_include_directories(${PROJECT_NAME}
    PUBLIC
    ${EIGEN3_INCLUDE_DIRS}
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
target_compile_options(${PROJECT_NAME} PUBLIC ${ocs2_CXX_FLAGS})   

ament_target_dependencies(${PROJECT_NAME} ${TARGET_DEPENDENCE})
ament_export_targets(${PROJECT_NAME}_Targets HAS_LIBRARY_TARGET)
ament_export_dependencies(${TARGET_DEPENDENCE})

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
