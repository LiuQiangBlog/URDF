add_library(tinyurdf
        KDLRobot.cpp
        TinyURDFParser.cpp)
target_link_libraries(tinyurdf tinyxml2)

add_executable(tutorial_tinyurdf_loader tutorial/tutorial_tinyurdf_loader.cpp)
target_link_libraries(tutorial_tinyurdf_loader tinyurdf orocos-kdl)

add_executable(kdl_robot_example tutorial/kdl_robot_example.cpp)
target_link_libraries(kdl_robot_example tinyurdf orocos-kdl)

add_library(urdf_parser
        src/joint.cpp
        src/link.cpp
        src/model.cpp
        src/pose.cpp
        src/pose.hpp
        src/twist.cpp
        src/urdf_model_state.cpp
        src/urdf_sensor.cpp
        src/world.cpp
)
target_link_libraries(urdf_parser tinyxml2)

add_library(kdl_parser src/kdl_parser.cpp)
target_link_libraries(kdl_parser orocos-kdl urdf_parser)

add_executable(check_urdf src/check_urdf.cpp)
target_link_libraries(check_urdf urdf_parser)

add_executable(urdf_to_graphviz src/urdf_to_graphviz.cpp)
target_link_libraries(urdf_to_graphviz urdf_parser)

