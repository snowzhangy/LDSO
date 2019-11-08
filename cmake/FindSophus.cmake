set(SOPHUS_DIR ${PROJECT_SOURCE_DIR}/thirdparty/Sophus)

find_path(
  SOPHUS_INCLUDE_DIR
  sophus/sophus.hpp
  HINTS ${SOPHUS_DIR}
)

add_library(
  Sophus
  INTERFACE
)
add_library(Sophus::Sophus ALIAS Sophus)

target_include_directories(
  Sophus
  SYSTEM INTERFACE
  ${SOPHUS_INCLUDE_DIR}
)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(
  Sophus
  SOPHUS_INCLUDE_DIR
)

