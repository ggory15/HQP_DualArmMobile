#
# Copyright (c) 2017 CNRS
#
# This file is part of tsid
# pinocchio is free software: you can redistribute it
# and/or modify it under the terms of the GNU Lesser General Public
# License as published by the Free Software Foundation, either version
# 3 of the License, or (at your option) any later version.
# pinocchio is distributed in the hope that it will be
# useful, but WITHOUT ANY WARRANTY; without even the implied warranty
# of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
# General Lesser Public License for more details. You should have
# received a copy of the GNU Lesser General Public License along with
# pinocchio If not, see
# <http://www.gnu.org/licenses/>.

# --- MAIN LIBRARY --- #
SET(LIBRARY_NAME ${PROJECT_NAME})

SET(${LIBRARY_NAME}_CONSTRAINT_SOURCES
   constraint/constraint-base.cpp
   constraint/constraint-equality.cpp
   constraint/constraint-inequality.cpp
   constraint/constraint-bound.cpp
  )

SET(${LIBRARY_NAME}_TASK_SOURCES
    tasks/task-base.cpp
    tasks/task-com.cpp
    #tasks/task-contact-force.cpp
    tasks/task-joint-bounds.cpp
    tasks/task-joint-posture.cpp
    tasks/task-mobile.cpp
    tasks/task-motion.cpp
    tasks/task-operational.cpp
    tasks/task-singualrity.cpp
  )

SET(${LIBRARY_NAME}_CONTACT_SOURCES
    contacts/contact-base.cpp
    contacts/contact-3d.cpp
  )

SET(${LIBRARY_NAME}_TRAJECTORIE_SOURCES
    trajectories/trajectory-jointspace.cpp
    trajectories/trajectory-operationalspace.cpp
  )

SET(${LIBRARY_NAME}_SOLVERS_SOURCES
  #  solvers/eiguadprog-fast.cpp
    solvers/solver-HQP-base.cpp
    solvers/solver-HQP-factory.cpp
    solvers/solver-HQP-eiquadprog.cpp
    solvers/solver-HQP-qpoases.cpp
    solvers/solver-utils.cpp
  )

SET(${LIBRARY_NAME}_ROBOTS_SOURCES
    robot/robot_model.cpp
  )

SET(${LIBRARY_NAME}_FORMULATIONS_SOURCES
    controller/inverse-dynamics.cpp
  )

SET(${LIBRARY_NAME}_SOURCES
    ${${LIBRARY_NAME}_CONSTRAINT_SOURCES}
    ${${LIBRARY_NAME}_TASK_SOURCES}
    ${${LIBRARY_NAME}_CONTACTS_SOURCES}
    ${${LIBRARY_NAME}_TRAJECTORIE_SOURCES}
    ${${LIBRARY_NAME}_SOLVERS_SOURCES}
    ${${LIBRARY_NAME}_ROBOTS_SOURCES}
    ${${LIBRARY_NAME}_FORMULATIONS_SOURCES}
    ${VREP_ROOT}/programming/remoteApi/extApi.c
    ${VREP_ROOT}/programming/remoteApi/extApiPlatform.c
    utils/utils.cpp
    vrep_bridge/vrep_bridge.cpp
  )

  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
  #add_definitions(-D_GLIBCXX_USE_CXX11_ABI=0)
  set(CMAKE_BUILD_TYPE RELEASE)
  


ADD_LIBRARY(${LIBRARY_NAME} SHARED ${HEADERS_FULL_PATH} ${${LIBRARY_NAME}_SOURCES})
SET_TARGET_PROPERTIES(${LIBRARY_NAME} PROPERTIES LINKER_LANGUAGE CXX)
ADD_HEADER_GROUP(HEADERS_FULL_PATH)
ADD_SOURCE_GROUP(${LIBRARY_NAME}_SOURCES)

PKG_CONFIG_USE_DEPENDENCY(${LIBRARY_NAME} eigen3)
#PKG_CONFIG_USE_DEPENDENCY(${LIBRARY_NAME} rbdl)
#PKG_CONFIG_USE_DEPENDENCY(${LIBRARY_NAME} qpoases)

INSTALL(TARGETS ${PROJECT_NAME} DESTINATION lib)

