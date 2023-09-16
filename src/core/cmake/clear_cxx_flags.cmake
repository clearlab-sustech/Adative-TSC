list(APPEND CLEAR_CXX_FLAGS
  "-march=native"
  "-mtune=native"
  "-fPIC"
  "-pthread"
  "-Wfatal-errors"
  "-Wl,--no-as-needed"
  )

list(APPEND CLEAR_CXX_FLAGS
  "-DBOOST_ALL_DYN_LINK"
  )

# Cpp standard version
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
