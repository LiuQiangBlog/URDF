cmake_minimum_required(VERSION 3.5)
project(rdl_benchmark)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
add_compile_options(-Wall -Wextra -Wpedantic -Werror)

find_package(ament_cmake REQUIRED)
find_package(rdl_dynamics)
find_package(rdl_urdfreader)
find_package(Boost REQUIRED COMPONENTS program_options)

include_directories(SYSTEM ${Boost_INCLUDE_DIRS})
include_directories(include)

add_executable(${PROJECT_NAME} src/model_generator.cpp src/Human36Model.cpp src/benchmark.cpp)
target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME} Boost rdl_urdfreader rdl_dynamics)

add_executable(${PROJECT_NAME}_kinematics_benchmark src/kinematics_benchmark.cpp)
target_include_directories(${PROJECT_NAME}_kinematics_benchmark
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME}_kinematics_benchmark Boost rdl_urdfreader rdl_dynamics)

install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_kinematics_benchmark
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_cmake_xmllint REQUIRED)
  ament_xmllint()

  find_package(ament_cmake_lint_cmake REQUIRED)
  ament_lint_cmake()

  find_package(ament_cmake_cppcheck REQUIRED)
  ament_cppcheck()

  find_package(ament_cmake_clang_format REQUIRED)
  ament_clang_format(CONFIG_FILE $ENV{HOME}/.clang-format)
endif()

ament_package()
