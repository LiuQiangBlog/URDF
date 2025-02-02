# File:   UseADIFOR2.cmake
# Author: Christian Hoffmann
# Date:   2007--2009
#
# This file is part of the MUSCOD/VPLAN suite, which is proprietary software of
#   Simulation and Optimization Workgroup
#   Interdisciplinary Center for Scientific Computing (IWR)
#   University of Heidelberg, Germany.
#
# Copyright (C) 2007--2009. All rights reserved.
#
####################################################################################################
#
# Including this file makes prepares your CMake script for package XXX. Usage example:
#
# FIND_PACKAGE( XXX ) # invokes FindADIFOR2.cmake
# IF( XXX_FOUND )
#   INCLUDE( UseXXX.cmake )
# ENDIF()
#
####################################################################################################

IF (NOT _AMD_)
    SET(_AMD_ TRUE)


    INCLUDE(UseUFCONFIG)

    ADD_DEFINITIONS(${AMD_COMPILER_FLAGS})
    INCLUDE_DIRECTORIES(${AMD_INCLUDE_DIRS})
    LINK_DIRECTORIES(${AMD_LIBRARY_DIRS})

    MESSAGE(STATUS "Using AMD")

ENDIF ()
