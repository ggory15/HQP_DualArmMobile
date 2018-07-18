
SET(PROJECT_NAME HQP)
SET(PROJECT_DESCRIPTION "HQP algorithm for various robot")
SET(PROJECT_URL "http://github.com/ggory15")
OPTION(INSTALL_DOCUMENTATION "Generate and install the documentation" OFF)

cmake_minimum_required(VERSION 2.8)
aux_source_directory(. SRC_LIST)

LIST( APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/../rbdl" )
LIST( APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}" )
MESSAGE( STATUS "CMAKE_MODULE_PATH: " ${CMAKE_MODULE_PATH} )

INCLUDE(cmake/base.cmake)
INCLUDE(cmake/boost.cmake)
#INCLUDE(cmake/eigen.cmake)
INCLUDE(cmake/FindRBDL.cmake)
INCLUDE(cmake/ide.cmake)

IF(APPLE)
  APPLY_DEFAULT_APPLE_CONFIGURATION()
ENDIF(APPLE)

# Disable -Werror on Unix for now.
SET(CXX_DISABLE_WERROR True)
SET(CMAKE_VERBOSE_MAKEFILE True)

SETUP_PROJECT()

IF(WIN32)
  SET(LINK copy_if_different)
ELSE(WIN32)
  SET(LINK create_symlink)
ENDIF(WIN32)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
#add_definitions(-D_GLIBCXX_USE_CXX11_ABI=0)
set(CMAKE_BUILD_TYPE RELEASE)


find_package(PkgConfig QUIET)

OPTION (INITIALIZE_WITH_NAN "Initialize Eigen entries with NaN" OFF)
OPTION (EIGEN_RUNTIME_NO_MALLOC "If ON, it can assert in case of runtime allocation" ON)
OPTION (EIGEN_NO_AUTOMATIC_RESIZING "If ON, it forbids automatic resizing of dynamics arrays and matrices" OFF)

ADD_REQUIRED_DEPENDENCY("eigen3 >= 3.2.0")

SEARCH_FOR_BOOST()
set (VREP_ROOT
    $ENV{HOME}/vrep/
    CACHE "VRep root dir" STRING)

include_directories(
  ${EIGEN3_INCLUDE_DIR} 
  ${RBDL_INCLUDE_DIR}
  ${VREP_ROOT}/programming/include 
  ${VREP_ROOT}/programming/remoteApi
  ${Boost_INCLUDE_DIR}
  ${VREP_PATH}/remoteApi 
  ${VREP_PATH}/include
)

if(MSVC)
    add_definitions(-DBOOST_ALL_NO_LIB)
endif()
add_definitions(-DBOOST_TEST_DYN_LINK)


SET(${PROJECT_NAME}_TASKS_HEADERS
    include/tasks/task-base.h
    include/tasks/task-com.h
    include/tasks/task-joint-bounds.h
    include/tasks/task-joint-posture.h
    include/tasks/task-mobile.h
    include/tasks/task-motion.h
    include/tasks/task-operational.h
    include/tasks/task-singularity.h
    include/tasks/trajectory-base.h
)
SET(${PROJECT_NAME}_CONSTRAINT_HEADERS
    include/constraint/constraint-base.h
    include/constraint/constraint-bound.h
    include/constraint/constraint-equality.h
    include/constraint/constraint-inequality.h
    include/constraint/fwd_constraints.h
)
SET(${PROJECT_NAME}_CONTACT_HEADERS
    include/contacts/contact-base.h
    include/contacts/contact-3d.h
)
SET(${PROJECT_NAME}_TRAJECTORY_HEADERS
    include/trajectories/trajectory-base.h
    include/trajectories/trajectory-jointspace.h
    include/trajectories/trajectory-operationalspace.h
)
SET(${PROJECT_NAME}_SOLVERS_HEADERS
    include/solvers/eiquadprog_2011.h
    include/solvers/fwd_solver.h
    include/solvers/solver-HQP-base.h
    include/solvers/solver-HQP-eiquadprog.h
    include/solvers/solver-HQP-factory.h
    include/solvers/solver-HQP-factory.hxx
    include/solvers/solver-HQP-output.h
    include/solvers/solver-HQP-qpoases.h
    include/solvers/solver-utils.h
)
SET(${PROJECT_NAME}_CONTROLLER_HEADERS
    include/controller/Inverse-dynamics.h
)
SET(${PROJECT_NAME}_ROBOT_HEADERS
    include/robot/robot_model.h
)
SET(${PROJECT_NAME}_UTILS_HEADERS
    include/utils/container.h
    include/utils/motion.h
    include/utils/utils.h
)
SET(${PROJECT_NAME}_VREP_BRIDGE_HEADERS
    include/vrep_bridge/vrep_bridge.h
)


SET(HEADERS
    include/fwd.h
    ${${PROJECT_NAME}_TASKS_HEADERS}
    ${${PROJECT_NAME}_CONSTRAINT_HEADERS}
    ${${PROJECT_NAME}_CONTACT_HEADERS}
    ${${PROJECT_NAME}_TRAJECTORY_HEADERS}
    ${${PROJECT_NAME}_SOLVERS_HEADERS}
    ${${PROJECT_NAME}_CONTROLLER_HEADERS}
    ${${PROJECT_NAME}_UTILS_HEADERS}
    ${${PROJECT_NAME}_VREP_BRIDGE_HEADERS}
    ${${PROJECT_NAME}_ROBOT_HEADERS}       
)
LIST(REMOVE_DUPLICATES HEADERS)

SET(HEADERS_FULL_PATH "")
FOREACH(header ${HEADERS})
  LIST(APPEND HEADERS_FULL_PATH "${CMAKE_SOURCE_DIR}/${header}") 
  GET_FILENAME_COMPONENT(headerName ${header} NAME)
  GET_FILENAME_COMPONENT(headerPath ${header} PATH)
  INSTALL(FILES ${${PROJECT_NAME}_SOURCE_DIR}/${header}
    DESTINATION ${CMAKE_INSTALL_PREFIX}/${headerPath}
          PERMISSIONS OWNER_READ GROUP_READ WORLD_READ OWNER_WRITE)
ENDFOREACH(header ${HEADERS})

add_definitions (-DNON_MATLAB_PARSING)
add_definitions (-DMAX_EXT_API_CONNECTIONS=255)
add_definitions (-DDO_NOT_USE_SHARED_MEMORY)
add_definitions (-Ddof=14)
ADD_SUBDIRECTORY(src)

add_executable(test_mobile demo-humanoid/demo_mobilemanipulator.cpp)
PKG_CONFIG_USE_DEPENDENCY(test_mobile eigen3)
target_link_libraries(test_mobile HQP rbdl qpOASES pthread)
