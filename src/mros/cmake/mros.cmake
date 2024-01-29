add_library(mroslib SHARED IMPORTED)
set_target_properties(mroslib PROPERTIES IMPORTED_LOCATION ${CMAKE_INSTALL_PREFIX}/lib/libmroslib.so)