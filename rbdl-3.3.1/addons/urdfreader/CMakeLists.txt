add_library(urdfreader urdfreader.cc)
target_link_libraries(urdfreader urdf_parser rbdl-static tinyxml2)

add_executable(testURDF testURDF.cc)
target_link_libraries(testURDF urdfreader urdf_parser rbdl-static tinyxml2)

add_executable(rbdl_urdfreader_util rbdl_urdfreader_util.cc)
target_link_libraries(rbdl_urdfreader_util urdfreader urdf_parser rbdl-static tinyxml2)

add_executable(example example.cc)
target_link_libraries(example urdfreader urdf_parser rbdl-static tinyxml2)