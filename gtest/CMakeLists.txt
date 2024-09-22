# Build gtest
include_directories(${CMAKE_CURRENT_SOURCE_DIR})
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
add_library(gtest STATIC src/gtest-all.cc)
add_library(gtest_main STATIC src/gtest_main.cc)
target_link_libraries(gtest_main gtest)
target_compile_features(gtest PUBLIC cxx_std_11)