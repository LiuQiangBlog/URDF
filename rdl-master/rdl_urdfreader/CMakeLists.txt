cmake_minimum_required(VERSION 3.5)
project(rdl_urdfreader)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_compile_options(-Wall -Wextra -Wpedantic -Werror)

find_package(ament_cmake REQUIRED)
find_package(urdf REQUIRED)
find_package(rdl_dynamics)
find_package(tinyxml_vendor REQUIRED)
find_package(TinyXML REQUIRED)
find_package(rclcpp REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/urdfreader.cpp)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME} TinyXML rdl_dynamics urdf rclcpp)

add_executable(${PROJECT_NAME}_util src/rdl_urdfreader_util.cpp)
target_link_libraries(${PROJECT_NAME}_util ${PROJECT_NAME})
ament_target_dependencies(${PROJECT_NAME}_util TinyXML rdl_dynamics urdf)

install(DIRECTORY include/
  DESTINATION include
)

install(TARGETS ${PROJECT_NAME} EXPORT ${PROJECT_NAME}_TARGETS
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(TARGETS ${PROJECT_NAME}_util
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_index_cpp REQUIRED)

  ament_add_gtest(UrdfReaderTests tests/UrdfReaderTests.cpp)
  target_link_libraries(UrdfReaderTests ${PROJECT_NAME})
  ament_target_dependencies(UrdfReaderTests TinyXML rdl_dynamics urdf ament_index_cpp)
  target_include_directories(UrdfReaderTests
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )

  install(DIRECTORY tests DESTINATION share/${PROJECT_NAME})

  find_package(ament_cmake_xmllint REQUIRED)
  ament_xmllint()

  find_package(ament_cmake_lint_cmake REQUIRED)
  ament_lint_cmake()

  find_package(ament_cmake_cppcheck REQUIRED)
  ament_cppcheck()

  find_package(ament_cmake_clang_format REQUIRED)
  ament_clang_format(CONFIG_FILE $ENV{HOME}/.clang-format)
endif()

ament_export_targets(${PROJECT_NAME}_TARGETS HAS_LIBRARY_TARGET)

ament_export_include_directories(
  include
)
ament_export_libraries(
  ${PROJECT_NAME}
)
ament_export_dependencies(
  rdl_dynamics
  urdf
  tinyxml_vendor
  TinyXML
  rclcpp
)

ament_package()
