#
# target_link_libraries(
#   main
#   PUBLIC
#   SuiteSparse::cholmod
# )
#
find_package(suitesparse QUIET)

if(NOT TARGET suitesparse::cholmod)
  find_path(
    CHOLMOD_INCLUDE_DIR NAMES cholmod.h amd.h camd.h
    PATHS
    ${SUITE_SPARSE_ROOT}/include
    /usr/include/suitesparse
    /usr/include/ufsparse
    /opt/local/include/ufsparse
    /usr/local/include/ufsparse
    /sw/include/ufsparse
  )

  find_library(
    CHOLMOD_LIBRARY
    NAMES cholmod
    PATHS
    ${SUITE_SPARSE_ROOT}/lib
    /usr/lib
    /usr/local/lib
    /opt/local/lib
    /sw/lib
  )

  include(FindPackageHandleStandardArgs)
  find_package_handle_standard_args(
    CHOLMOD DEFAULT_MSG
    CHOLMOD_INCLUDE_DIR
    CHOLMOD_LIBRARY
  )

  add_library(
    cholmod
    INTERFACE
  )

  add_library(
    SuiteSparse::cholmod
    ALIAS
    cholmod
  )

  target_include_directories(
    cholmod
    SYSTEM INTERFACE
    ${CHOLMOD_INCLUDE_DIR}
  )

  target_link_libraries(
    cholmod
    INTERFACE
    ${CHOLMOD_LIBRARY}
  )
endif()

