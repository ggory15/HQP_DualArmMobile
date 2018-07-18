project(robostar_arm_controller_7dof)
cmake_minimum_required(VERSION 2.8)
aux_source_directory(. SRC_LIST)

LIST( APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/../rbdl" )
LIST( APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}" )
MESSAGE( STATUS "CMAKE_MODULE_PATH: " ${CMAKE_MODULE_PATH} )


find_package(PkgConfig QUIET)
find_package(Eigen3 REQUIRED)
find_package(RBDL REQUIRED)
find_package(fcl QUIET)
find_package(Boost COMPONENTS filesystem)


set (VREP_ROOT
    $ENV{HOME}/vrep/
    CACHE "VRep root dir" STRING)

if (fcl_FOUND)
message("fcl found!")
endif (fcl_FOUND)

if(PKG_CONFIG_FOUND)
  pkg_check_modules(CCD ccd)
  # check to see if the pkg is installed under the libccd name
  if(NOT CCD_FOUND)
    pkg_check_modules(CCD libccd)
  endif()
endif()

if(NOT CCD_FOUND)
    # if pkgconfig is not installed, then fall back on more fragile detection
    # of ccd
    find_path(CCD_INCLUDE_DIRS ccd/ccd.h)
    find_library(CCD_LIBRARY
        ${CMAKE_SHARED_LIBRARY_PREFIX}ccd${CMAKE_SHARED_LIBRARY_SUFFIX})
    if(CCD_INCLUDE_DIRS AND CCD_LIBRARY)
        set(CCD_LIBRARIES "${CCD_LIBRARY}")
    else()
        message(FATAL_ERROR "Libccd is required by FCL")
    endif()
endif()
include_directories(${CCD_INCLUDE_DIRS})
link_directories(${CCD_LIBRARY_DIRS})

if(NOT MSVC)
    set(PKG_CFLAGS "-std=c++11")
endif()


include_directories(
  ${EIGEN3_INCLUDE_DIR} 
  ${RBDL_INCLUDE_DIR}
  ${VREP_ROOT}/programming/include 
  ${VREP_ROOT}/programming/remoteApi
  ${fcl_INCLUDE_DIRS}
  ${FCL_INCLUDE_DIRS}
  ${CCD_LIBRARY_DIRS}
  ${Boost_INCLUDE_DIR}
)
if(MSVC)
    add_definitions(-DBOOST_ALL_NO_LIB)
endif()
add_definitions(-DBOOST_TEST_DYN_LINK)



include_directories (${VREP_PATH}/remoteApi ${VREP_PATH}/include)

SET(${PROJECT_NAME}_HEADERS
include/fcl/fcl_utility.h
include/fcl/fcl_model.h


include/robot/vrep_bridge.h
include/robot/haptic.h
include/robot/math_type_define.h
include/robot/robostar_7_dof_robot_hw_config.h
include/robot/controller.h
)



add_definitions (-DNON_MATLAB_PARSING)
add_definitions (-DMAX_EXT_API_CONNECTIONS=255)
add_definitions (-DDO_NOT_USE_SHARED_MEMORY)

# set (SOURCES
#     fcl_utility.cpp
#     main.cpp 
#     controller.cpp
#     vrep_bridge.cpp
    
#     ${VREP_ROOT}/programming/remoteApi/extApi.c
#     ${VREP_ROOT}/programming/remoteApi/extApiPlatform.c
#     )

add_library(global_lib
controller.cpp
vrep_bridge.cpp
fcl_utility.cpp
fcl_model.cpp
${VREP_ROOT}/programming/remoteApi/extApi.c
${VREP_ROOT}/programming/remoteApi/extApiPlatform.c
)


add_executable(main main.cpp)


target_link_libraries(global_lib
${RBDL_LIBRARY}
pthread
${CCD_LIBRARIES}
${fcl_LIBRARIES}
fcl
${Boost_FILESYSTEM_LIBRARY}
${Boost_SYSTEM_LIBRARY}
)

target_link_libraries(main
 global_lib
)

# target_link_libraries(${PROJECT_NAME}
#  ${RBDL_LIBRARY}
#  pthread
#  ${CCD_LIBRARIES}
#  ${fcl_LIBRARIES}
#  fcl
#  ${Boost_FILESYSTEM_LIBRARY}
#  ${Boost_SYSTEM_LIBRARY}

# )


set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

set(CMAKE_BUILD_TYPE RELEASE)

