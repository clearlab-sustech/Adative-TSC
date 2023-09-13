cmake_minimum_required(VERSION 3.10)
project(ClimbStairs)

set(CMAKE_CXX_STANDARD 11)

#set(CMAKE_CXX_FLAGS "-O3 -no-pie -ggdb -march=native -Wall \
#  -Wextra -Wcast-align -Wdisabled-optimization -Wformat=2 \
#  -Winit-self -Wmissing-include-dirs -Woverloaded-virtual \
#  -Wshadow -Wsign-promo -mavx -mfma -ffast-math -fopenmp")
set(CMAKE_CXX_FLAGS "-O3 -no-pie -ggdb -march=native -mavx -mfma -ffast-math -fopenmp -pthread -lpthread")
set(CMAKE_C_FLAGS "-O3  -ggdb  -march=native -std=gnu99 -I.")

list(APPEND CMAKE_PREFIX_PATH "/opt/openrobots/")
find_package(pinocchio REQUIRED)
if (pinocchio_FOUND)
    # message("pinocchio found")
    include_directories(SYSTEM ${pinocchio_INCLUDE_DIRS})
    # message("pinocchio include dir at " ${pinocchio_INCLUDE_DIRS})
else ()
    message(FATAL_ERROR "Cannot find pinocchio")
endif ()

SET(THIS_COM ${CMAKE_CURRENT_SOURCE_DIR}/)
CONFIGURE_FILE(${CMAKE_CURRENT_SOURCE_DIR}/config.h.cmake
        ${CMAKE_BINARY_DIR}/Configuration.h)

include_directories(./data)
include_directories(./controller)
include_directories(./estimator)
include_directories(./gait)
include_directories(./planner)
include_directories(./Perception)
include_directories(./user)
include_directories(./utility)
include_directories(./lcm)
include_directories(./Hardware)
include_directories(SYSTEM /usr/include/eigen3)

include_directories(./third_party/ParamHandler)
include_directories(./third_party/aliengo_sdk)
include_directories(${CMAKE_BINARY_DIR})
include_directories(SYSTEM ${THIS_COM}/PoplarLib/include)
include_directories(SYSTEM ${THIS_COM}/PoplarLib/third_party/qpOASES/include)
include_directories(SYSTEM ${THIS_COM}/PoplarLib/third_party/eiquadprog/include)
include_directories(SYSTEM ${THIS_COM}/PoplarLib/third_party/qpSWIFT/include)
include_directories(SYSTEM ${THIS_COM}/third_party/aliengo_sdk/include)

# matlab relevant
file(GLOB_RECURSE lib_source "./controller/*.cpp" "./estimator/*.cpp" "./gait/*.cpp" "./planner/*.cpp" "./Perception/*.cpp" "./sensors/*.cpp" "./user/*.cpp" "./utility/*.cpp" "./Hardware/*.cpp")

add_subdirectory(./third_party)
add_subdirectory(./sim)
#add_subdirectory(./test)
add_subdirectory(./PoplarLib)

link_directories(./third_party/aliengo_sdk/lib)

add_library(ClimbStairsLib SHARED ${lib_source})
target_link_libraries(ClimbStairsLib qpOASES dynacore_param_handler pinocchio::pinocchio  rt pthread matio PoplarLib lcm libunitree_legged_sdk_amd64.so)
add_executable(ctrl main.cpp)
target_link_libraries(ctrl ClimbStairsLib)