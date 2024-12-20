# Include the exported CMake file
get_filename_component(orwell_CMAKE_DIR "${CMAKE_CURRENT_LIST_FILE}" PATH)

# This macro enables usage of find_dependency().
# https://cmake.org/cmake/help/v3.11/module/CMakeFindDependencyMacro.html
include(CMakeFindDependencyMacro)
find_package(Eigen3 REQUIRED)

# Declare the used packages in order to communicate the requirements upstream.
if(NOT TARGET orwell::orwell)
    include("${orwell_CMAKE_DIR}/orwell-config-targets.cmake")
    include("${orwell_CMAKE_DIR}/orwell-packages.cmake")
else()
    set(BUILD_TARGET orwell::orwell)

    get_target_property(TARGET_INCLUDE_DIRS ${BUILD_TARGET} INTERFACE_INCLUDE_DIRECTORIES)
    set(TARGET_INCLUDE_DIRS "${TARGET_INCLUDE_DIRS}" CACHE PATH "${BUILD_TARGET} include directories")
    list(APPEND orwell_INCLUDE_DIRS ${TARGET_INCLUDE_DIRS})
endif()

check_required_components(orwell)