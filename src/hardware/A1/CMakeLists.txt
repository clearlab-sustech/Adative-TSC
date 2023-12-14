cmake_minimum_required(VERSION 3.5)
project(hardware)

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
find_package(ament_index_cpp REQUIRED)
find_package(trans REQUIRED)
find_package(asserts REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(core REQUIRED)
find_package(yaml-cpp REQUIRED)

set(TARGET_DEPENDENCE
        ament_cmake
        ament_index_cpp
        trans
        sensor_msgs
        rclcpp
        geometry_msgs
        core
        yaml-cpp
)

link_directories(/usr/local/lib)

add_library(libunitree_legged_sdk SHARED IMPORTED)
set_target_properties(libunitree_legged_sdk PROPERTIES IMPORTED_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/lib/libunitree_legged_sdk_amd64.so)

file(GLOB_RECURSE source ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cc)

add_executable(${PROJECT_NAME} ${source})
target_include_directories(${PROJECT_NAME} PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)

target_link_libraries(${PROJECT_NAME}
        libunitree_legged_sdk lcm
        ${YAML_CPP_LIBRARIES}
)

ament_target_dependencies(${PROJECT_NAME} ${TARGET_DEPENDENCE})
ament_export_dependencies(${TARGET_DEPENDENCE})

file(GLOB_RECURSE libs ${CMAKE_CURRENT_SOURCE_DIR}/lib/*)

install(FILES ${libs}
        DESTINATION lib)

install(TARGETS ${PROJECT_NAME}
        DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
        DESTINATION share/${PROJECT_NAME})

ament_package()
