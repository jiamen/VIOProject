cmake_minimum_required(VERSION 3.15)
project(03_MyCurveFitting_LM)

set( CMAKE_CXX_STANDARD 11 )
set( CMAKE_CXX_FLAGS "-std=c++11 -g -Wall" )
set( DEFAULT_BUILD_TYPE "Release" )
if ( NOT CMAKE_BUILD_TYPE )
    message( STATUS "Setting build type to '${DEFAULT_BUILD_TYPE}' as none was specified. " )

    
endif ()


FIND_PACKAGE( OpenMP REQUIRED )


option( BUILD_APPS "Build APPs for vio cource" YES )
option( BUILD_APPS "Build test for vio cource" No )



if ( BUILD_APPS )
    add_subdirectory(src)
endif()