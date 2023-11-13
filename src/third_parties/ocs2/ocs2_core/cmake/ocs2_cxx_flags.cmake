
list(APPEND OCS2_CXX_FLAGS
  "-march=native"
  "-mtune=native"
  "-fPIC"
  "-pthread"
  "-Wfatal-errors"
  "-Wl,--no-as-needed"
  )

# Force Boost dynamic linking
find_package(Boost COMPONENTS log REQUIRED)
list(APPEND ocs2_CXX_FLAGS
  "-DBOOST_ALL_DYN_LINK"
  )

# Add OpenMP flags
if (NOT DEFINED OpenMP_CXX_FOUND)
  find_package(OpenMP REQUIRED)
endif (NOT DEFINED OpenMP_CXX_FOUND)
list(APPEND OCS2_CXX_FLAGS
  ${OpenMP_CXX_FLAGS}
  )

# Cpp standard version
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
