# 
# Copyright (c) 2022, CNRS-UM LIRMM
# All rights reserved.
# 
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# 
# 1. Redistributions of source code must retain the above copyright notice,
# this list of conditions and the following disclaimer.
# 
# 2. Redistributions in binary form must reproduce the above copyright notice,
# this list of conditions and the following disclaimer in the documentation
# and/or other materials provided with the distribution.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
# 

# Set the name of the urdf file: 
set(ManipulatorName "panda_foot")


get_filename_component(PARENT_DIR ${CMAKE_CURRENT_SOURCE_DIR} DIRECTORY)



set(TestFiles "PandaExample;FbRobotExample;Transform;Dynamics;Xman")


foreach(file ${TestFiles})

  configure_file(${CMAKE_CURRENT_SOURCE_DIR}/${file}.conf.cpp ${CMAKE_CURRENT_BINARY_DIR}/${file}.cpp @ONLY)
  file(COPY ${CMAKE_CURRENT_BINARY_DIR}/${file}.cpp   DESTINATION ${CMAKE_CURRENT_SOURCE_DIR}  FILE_PERMISSIONS OWNER_READ  GROUP_READ WORLD_READ )
  
  set(${file}_src ${file}.cpp )
  
  add_executable(${file} ${${file}_src})
  # add_dependencies(${file} GeometricRobotics)
  target_link_libraries(${file} PRIVATE gtest_main)
  #target_link_libraries(${file} PRIVATE yaml-cpp)
  #target_link_libraries(${file} PUBLIC RobotInterface::RobotInterface)
  # target_link_libraries(${file} PUBLIC ${CMAKE_BINARY_DIR}/src/libGeometricRobotics.so)  # Link to the newly built library
  target_link_libraries(${file} PUBLIC GeometricRobotics)  # Link to the newly built library
  target_link_libraries(${file} PUBLIC RoboticsUtils::RoboticsUtils)
  add_test(NAME ${file}Test COMMAND ${file})

endforeach(file)

