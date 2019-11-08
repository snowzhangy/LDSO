# /cmake/options.cmake
add_library(
  options
  INTERFACE
)

add_library(
  dso::options
  ALIAS
  options
)

target_compile_features(
  options
  INTERFACE
  cxx_std_17
)

target_compile_options(
  options
  INTERFACE
  #$<$<CXX_COMPILER_ID:Clang>:-Weverything;-Wno-c++98-compat-pedantic;-fcolor-diagnostics;-Wno-unused-macros>
  #$<$<CXX_COMPILER_ID:GNU>:-Wall;-W;-Wpedantic;-Wshadow;-Wnon-virtual-dtor;-Wold-style-cast;-Wunused;-Wformat=2>
  $<$<CXX_COMPILER_ID:MSVC>:/bigobj;/W3>
)

target_compile_definitions(
  options 
  INTERFACE 
  $<$<CXX_COMPILER_ID:MSVC>:_DISABLE_EXTENDED_ALIGNED_STORAGE;__SSE3__;__SSE2__;__SSE1__>)

target_link_libraries(
  options
  INTERFACE
  $<$<CXX_COMPILER_ID:Linux>:stdc++fs>
)


add_library(
  SIMD
  INTERFACE
)

add_library(
  dso::SIMD
  ALIAS
  SIMD
)

target_compile_options(
  SIMD
  INTERFACE
  $<$<PLATFORM_ID:Linux>:-msse4.1;-march=native>
  $<$<PLATFORM_ID:Windows>:/arch:AVX2>
)

target_compile_definitions(
  SIMD
  INTERFACE
  ENABLE_SSE
)