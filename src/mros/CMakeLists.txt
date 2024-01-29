cmake_minimum_required(VERSION 3.5)
project(mros)

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

set(TARGET_DEPENDENCE
  ament_cmake
)

install(
  DIRECTORY include/
  DESTINATION include
)

file(GLOB_RECURSE libs ${CMAKE_CURRENT_SOURCE_DIR}/lib/*)

install(FILES ${libs}
  DESTINATION lib)

ament_package(CONFIG_EXTRAS cmake/mros.cmake)
