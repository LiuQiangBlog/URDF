configure_file(${CMAKE_CURRENT_LIST_DIR}/${PROJECT_NAME}_package_config.h.in "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}/${PROJECT_NAME}_package_config.hpp")

add_library(${PROJECT_NAME} INTERFACE)

set_target_properties(${PROJECT_NAME} PROPERTIES
  VERSION ${${PROJECT_NAME}_VERSION}
  SOVERSION 1
  INTERFACE_${PROJECT_NAME}_MAJOR_VERSION 1
)

target_include_directories(${PROJECT_NAME}
    INTERFACE
        $<INSTALL_INTERFACE:${${PROJECT_NAME}_PREFIX_INCLUDE}>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}>
        $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/..>
)

#find_package(sackmesser REQUIRED)
#find_package(Eigen3 REQUIRED)

target_link_libraries(${PROJECT_NAME}
    INTERFACE
        sackmesser::sackmesser
        Eigen3::Eigen
)

install(
    FILES 
        "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}/${PROJECT_NAME}_package_config.hpp" 
        orwell.hpp
        Controller.hpp
        Controller.hxx
        ControllerManager.hpp
        ControllerManager.hxx
        RobotState.hpp
        RobotState.hxx
        RobotModel.hpp
        RobotModel.hxx
        ReferenceController.hpp
        ReferenceController.hxx
        ParallelController.hpp
        ParallelController.hxx
        StoppingCriterion.hpp
        MultiRobotController.hpp
        MultiRobotController.hxx
    DESTINATION 
        ${${PROJECT_NAME}_PREFIX_INCLUDE}/${PROJECT_NAME} 
    COMPONENT 
        Devel
)

add_subdirectory(torque)
add_subdirectory(velocity)
add_subdirectory(position)