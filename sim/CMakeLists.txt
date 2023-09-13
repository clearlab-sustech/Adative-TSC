cmake_minimum_required(VERSION 3.10)
#project(xiaotian_sim)

set(BUILD_TYPE_RELEASE TRUE)

# set(CMAKE_C_FLAGS "-O3  -ggdb  -march=native -std=gnu99 -I.")

set(CMAKE_CXX_STANDARD 11)

set(MUJOCO_LIB_DIR ${CMAKE_SOURCE_DIR}/sim/lib/)
link_directories(${MUJOCO_LIB_DIR})

find_library(GLFW libglfw.so.3 HINTS ${MUJOCO_LIB_DIR})
# find_library(MUJOCO libmujoco200.so ${MUJOCO_LIB_DIR} REQUIRE

include_directories(${THIS_COM}/sim/include/)
include_directories("./data")

add_executable(sim main.cpp include/uitools.c include/PeriodicTask.cpp)
add_executable(state_vis state_vis.cpp include/uitools.c include/PeriodicTask.cpp)
# target_include_directories(${PROJECT_NAME} PRIVATE ./include/)
target_link_libraries(sim mujoco210 ${GLFW} GL glew pthread rt lcm)
target_link_libraries(state_vis mujoco210 ${GLFW} GL glew pthread rt lcm)
