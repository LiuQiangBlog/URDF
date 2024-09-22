cmake_minimum_required(VERSION 3.5)
project(rdl_dynamics)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
add_compile_options(-Wall -Wextra -Wpedantic -Werror -Wno-maybe-uninitialized)

#find_package(ament_cmake REQUIRED)
#find_package(eigen3_cmake_module REQUIRED)
#find_package(Eigen3 REQUIRED)
#find_package(Boost REQUIRED COMPONENTS system thread)
#find_package(rclcpp REQUIRED)

include_directories(include)
add_library(${PROJECT_NAME} SHARED
  src/rdl_eigenmath.cpp
  src/rdl_mathutils.cpp
  src/rdl_utils.cpp
  src/rdl_hydro_utils.cpp
  src/Contacts.cpp
  src/Dynamics.cpp
  src/Joint.cpp
  src/Model.cpp
  src/Kinematics.cpp
  src/ReferenceFrame.cpp
  src/SpatialMotion.cpp
  src/SpatialAcceleration.cpp
  src/MotionVector.cpp
  src/FrameObject.cpp
  src/RigidBodyInertia.cpp
  src/HydroDynamics.cpp)

#ament_target_dependencies(${PROJECT_NAME} Boost Eigen3 rclcpp)

install(DIRECTORY include/
  DESTINATION include
)

install(TARGETS ${PROJECT_NAME} EXPORT ${PROJECT_NAME}_TARGETS
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

## Add gtest based cpp test target and link libraries
#if(BUILD_TESTING)
#  find_package(ament_cmake_gtest REQUIRED)
#
#  ament_add_gtest(ReferenceFrameTests tests/ReferenceFrameTest.cpp)
#  target_link_libraries(ReferenceFrameTests ${PROJECT_NAME})
#  ament_add_gtest(Point3Tests tests/Point3Test.cpp)
#  target_link_libraries(Point3Tests ${PROJECT_NAME})
#  ament_add_gtest(FramePointTests tests/FramePointTest.cpp)
#  target_link_libraries(FramePointTests ${PROJECT_NAME})
#  ament_add_gtest(FrameVectorTests tests/FrameVectorTest.cpp)
#  target_link_libraries(FrameVectorTests ${PROJECT_NAME})
#  ament_add_gtest(ModelFrameTests tests/ModelFrameTests.cpp)
#  target_link_libraries(ModelFrameTests ${PROJECT_NAME})
#  ament_add_gtest(RigidBodyInertiaTests tests/RigidBodyInertiaTests.cpp)
#  target_link_libraries(RigidBodyInertiaTests ${PROJECT_NAME})
#  ament_add_gtest(MotionVectorTests tests/MotionVectorTests.cpp)
#  target_link_libraries(MotionVectorTests ${PROJECT_NAME})
#  ament_add_gtest(ForceVectorTests tests/ForceVectorTests.cpp)
#  target_link_libraries(ForceVectorTests ${PROJECT_NAME})
#  ament_add_gtest(SpatialMotionTests tests/SpatialMotionTests.cpp)
#  target_link_libraries(SpatialMotionTests ${PROJECT_NAME})
#  ament_add_gtest(SpatialForceTests tests/SpatialForceTests.cpp)
#  target_link_libraries(SpatialForceTests ${PROJECT_NAME})
#  ament_add_gtest(SpatialMomentumTests tests/SpatialMomentumTests.cpp)
#  target_link_libraries(SpatialMomentumTests ${PROJECT_NAME})
#  ament_add_gtest(SpatialRigidBodyInertiaTests tests/SpatialRigidBodyInertiaTests.cpp)
#  target_link_libraries(SpatialRigidBodyInertiaTests ${PROJECT_NAME})
#  ament_add_gtest(MomentumTests tests/MomentumTests.cpp)
#  target_link_libraries(MomentumTests ${PROJECT_NAME})
#  ament_add_gtest(SpatialAccelerationTests tests/SpatialAccelerationTests.cpp)
#  target_link_libraries(SpatialAccelerationTests ${PROJECT_NAME})
#  ament_add_gtest(RdlCalcAccelerationsTests tests/RdlCalcAccelerationTests.cpp)
#  target_link_libraries(RdlCalcAccelerationsTests ${PROJECT_NAME})
#  ament_add_gtest(RdlBodyTests tests/RdlBodyTests.cpp)
#  target_link_libraries(RdlBodyTests ${PROJECT_NAME})
#  ament_add_gtest(RdlKinematicsTests tests/RdlKinematicsTests.cpp)
#  target_link_libraries(RdlKinematicsTests ${PROJECT_NAME})
#  ament_add_gtest(RdlCalcVelocitiesTests tests/RdlCalcVelocitiesTests.cpp)
#  target_link_libraries(RdlCalcVelocitiesTests ${PROJECT_NAME})
#  ament_add_gtest(RdlDynamicsTests tests/RdlDynamicsTests.cpp)
#  target_link_libraries(RdlDynamicsTests ${PROJECT_NAME})
#  ament_add_gtest(RdlContactsTests tests/RdlContactsTests.cpp)
#  target_link_libraries(RdlContactsTests ${PROJECT_NAME})
#  ament_add_gtest(RdlFloatingBaseTests tests/RdlFloatingBaseTests.cpp)
#  target_link_libraries(RdlFloatingBaseTests ${PROJECT_NAME})
#  ament_add_gtest(RdlMultiDofTests tests/RdlMultiDofTests.cpp)
#  target_link_libraries(RdlMultiDofTests ${PROJECT_NAME})
#  ament_add_gtest(RdlCompositeRigidBodyTests tests/RdlCompositeRigidBodyTests.cpp)
#  target_link_libraries(RdlCompositeRigidBodyTests ${PROJECT_NAME})
#  ament_add_gtest(RdlSparseFactorizationTests tests/RdlSparseFactorizationTests.cpp)
#  target_link_libraries(RdlSparseFactorizationTests ${PROJECT_NAME})
#  ament_add_gtest(RdlInverseDynamicsTests tests/RdlInverseDynamicsTests.cpp)
#  target_link_libraries(RdlInverseDynamicsTests ${PROJECT_NAME})
#  ament_add_gtest(RdlTwoLegModelTests tests/RdlTwoLegModelTests.cpp)
#  target_link_libraries(RdlTwoLegModelTests ${PROJECT_NAME})
#  ament_add_gtest(RdlImpulsesTests tests/RdlImpulsesTests.cpp)
#  target_link_libraries(RdlImpulsesTests ${PROJECT_NAME})
#  ament_add_gtest(MathTests tests/MathTests.cpp)
#  target_link_libraries(MathTests ${PROJECT_NAME})
#  ament_add_gtest(RdlModelTests tests/RdlModelTests.cpp)
#  target_link_libraries(RdlModelTests ${PROJECT_NAME})
#  ament_add_gtest(RdlUtilsTests tests/RdlUtilsTests.cpp)
#  target_link_libraries(RdlUtilsTests ${PROJECT_NAME})
#  ament_add_gtest(UnitTestUtilsTests tests/UnitTestUtilsTests.cpp)
#  target_link_libraries(UnitTestUtilsTests ${PROJECT_NAME})
#  ament_add_gtest(JointTests tests/JointTests.cpp)
#  target_link_libraries(JointTests ${PROJECT_NAME})
#  ament_add_gtest(RdlCustomJointTests tests/RdlCustomJointTests.cpp)
#  target_link_libraries(RdlCustomJointTests ${PROJECT_NAME})
#  ament_add_gtest(RdlCustomJointSingleBodyTests tests/RdlCustomJointSingleBodyTests.cpp)
#  target_link_libraries(RdlCustomJointSingleBodyTests ${PROJECT_NAME})
#  ament_add_gtest(RdlCustomJointMultiBodyTests tests/RdlCustomJointMultiBodyTests.cpp)
#  target_link_libraries(RdlCustomJointMultiBodyTests ${PROJECT_NAME})
#  ament_add_gtest(rdl_eigenmath_tests tests/rdl_eigenmath.cpp)
#  target_link_libraries(rdl_eigenmath_tests ${PROJECT_NAME})
#  ament_add_gtest(FrameOrientationTests tests/FrameOrientationTests.cpp)
#  target_link_libraries(FrameOrientationTests ${PROJECT_NAME})
#  ament_add_gtest(FrameVectorPairTests tests/FrameVectorPairTests.cpp)
#  target_link_libraries(FrameVectorPairTests ${PROJECT_NAME})
#  ament_add_gtest(model_parameter_updates tests/model_parameter_updates.cpp)
#  target_link_libraries(model_parameter_updates ${PROJECT_NAME})
#  ament_add_gtest(HydroDynamicsTests tests/HydroDynamicsTests.cpp)
#  target_link_libraries(HydroDynamicsTests ${PROJECT_NAME})
#
#  find_package(ament_cmake_xmllint REQUIRED)
#  ament_xmllint()
#
#  find_package(ament_cmake_lint_cmake REQUIRED)
#  ament_lint_cmake()
#
#  find_package(ament_cmake_cppcheck REQUIRED)
#  ament_cppcheck()
#
#  find_package(ament_cmake_clang_format REQUIRED)
#  ament_clang_format(CONFIG_FILE $ENV{HOME}/.clang-format)
#
#endif()
#
#ament_export_targets(${PROJECT_NAME}_TARGETS HAS_LIBRARY_TARGET)
#ament_export_include_directories(include)
#ament_export_libraries(${PROJECT_NAME})
#ament_export_dependencies(Boost eigen3_cmake_module Eigen3 rclcpp)
#ament_package(
#  CONFIG_EXTRAS "cfg-extras.cmake"
#)
