#
#Copyright (c) 2022, CNRS-UM LIRMM
#All rights reserved.
#
#Redistribution and use in source and binary forms, with or without
#modification, are permitted provided that the following conditions are met:
#
#1. Redistributions of source code must retain the above copyright notice,
#this list of conditions and the following disclaimer.
#
#2. Redistributions in binary form must reproduce the above copyright notice,
#this list of conditions and the following disclaimer in the documentation
#and/or other materials provided with the distribution.
#
#THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
#AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
#IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
#ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
#LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
#CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
#SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
#INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
#CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
#ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#POSSIBILITY OF SUCH DAMAGE.
#

# GoogleTest requires at least C++14
set(CMAKE_CXX_STANDARD 14)

# include(FetchContent)
# FetchContent_Declare(
#   googletest
#   URL https://github.com/google/googletest/archive/609281088cfefc76f9d0ce82e1ff6c30cc3591e5.zip
# )
# 
# # For Windows: Prevent overriding the parent project's compiler/linker settings
# set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
# FetchContent_MakeAvailable(googletest)

# set(rosRobotPath $ENV{HOME}/local/mc_rtc/catkin_data_ws/src)


set(RobotExamplesPath "${CMAKE_CURRENT_SOURCE_DIR}/RobotExamples/urdf/")

set(anymalPackageName anymal_b_simple_description)
set(anymalName anymal)
set(anymalBase base)

set(jvrcPackageName jvrc_description)
set(jvrcName jvrc1)
set(jvrcBase base_link)

set(hrpPackageName hrp4_description)
set(hrpName hrp4)
set(hrpBase base_link)

set(krRobotPath $ENV{HOME}/catkin_ws/src/RobotExamples/urdf/)
set(krPackageName KR5)
set(krName KR5)
set(krBase base_link)



# Add the example
add_subdirectory(Twists)
add_subdirectory(KinematicTree)
add_subdirectory(Jacobian)
add_subdirectory(Dynamics)
add_subdirectory(ImpactDynamics)
add_subdirectory(CoriolisAndCentrifugal)
