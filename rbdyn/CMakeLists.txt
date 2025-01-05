#
# Copyright 2012-2019 CNRS-UM LIRMM, CNRS-AIST JRL
#

set(SOURCES
        MultiBodyGraph.cpp
        MultiBody.cpp
        MultiBodyConfig.cpp
        FK.cpp
        FV.cpp
        FA.cpp
        Jacobian.cpp
        ID.cpp
        IK.cpp
        IS.cpp
        FD.cpp
        NumericalIntegration.cpp
        CoM.cpp
        Momentum.cpp
        ZMP.cpp
        IDIM.cpp
        VisServo.cpp
        Coriolis.cpp)
set(HEADERS
        Body.h
        Joint.h
        MultiBodyGraph.h
        MultiBody.h
        MultiBodyConfig.h
        FK.h
        FV.h
        FA.h
        Jacobian.h
        ID.h
        IK.h
        IS.h
        FD.h
        NumericalIntegration.h
        EulerIntegration.h
        CoM.h
        Momentum.h
        ZMP.h
        IDIM.h
        VisServo.h
        util.hh
        util.hxx
        Coriolis.h)

add_library(RBDyn ${SOURCES} ${HEADERS})
set_target_properties(RBDyn PROPERTIES CXX_STANDARD 11)
set_target_properties(RBDyn PROPERTIES SOVERSION ${PROJECT_VERSION_MAJOR}
        VERSION ${PROJECT_VERSION})



