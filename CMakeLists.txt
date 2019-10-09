cmake_minimum_required(VERSION 3.14)
project(ldso)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake)

include(options)
include(suitesparse)

find_package(Eigen3 REQUIRED)
find_package(glog REQUIRED)
find_package(Pangolin REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Sophus REQUIRED)
find_package(fmt REQUIRED)
find_package(DBow3 REQUIRED)
find_package(g2o REQUIRED)
# flags
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-march=native" COMPILER_SUPPORTS_MARCH_NATIVE)
if (COMPILER_SUPPORTS_MARCH_NATIVE)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=native")
endif ()
list(APPEND CMAKE_CXX_FLAGS ${SSE_FLAGS})

if (MSVC)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /EHsc")
endif (MSVC)


add_subdirectory(src)
add_subdirectory(examples)
