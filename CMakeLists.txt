cmake_minimum_required(VERSION 3.15)
project(03_MyCurveFitting_LM)

set( CMAKE_CXX_STANDARD 11 )
set( CMAKE_CXX_FLAGS "-std=c++11 -g -Wall" )
set( DEFAULT_BUILD_TYPE "Release" )
if ( NOT CMAKE_BUILD_TYPE )
    message( STATUS "Setting build type to '${DEFAULT_BUILD_TYPE}' as none was specified. " )
    set( CMAKE_BUILD_TYPE "${DEFAULT_BUILD_TYPE}" CACHE
            STRING "Choose the type of build." FORCE )
    # Set the possible value of build type for cmake-gui
    set_property( CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS
            "Debug" "Release" "MinSizeRel" "RelWithDebInfo" )
endif ()

FIND_PACKAGE( OpenMP REQUIRED )
if ( OPENMP_FOUND )
    message( "OPENMP FOUND" )
    ADD_DEFINITIONS( -DUSE_OPENMP )
    set( CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}" )
    set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}" )
    set( CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}" )
endif ()

set( CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS} -Wno-reorder" CACHE STRING "" FORCE )
set( CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS} -DNDEBUG -Who-reorder -O2" CACHE STRING "" FORCE )

list( APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake" )


option( BUILD_APPS "Build APPs for vio cource" YES )
option( BUILD_APPS "Build test for vio cource" No )

# third party libs
# eigen
include_directories( "usr/include/eigen3" )

# opencv
FIND_PACKAGE( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

include_directories( ${PROJECT_SOURCE_DIR} )

add_subdirectory( backend )
add_subdirectory( utils )


if ( BUILD_APPS )
    add_subdirectory(src)
endif()
