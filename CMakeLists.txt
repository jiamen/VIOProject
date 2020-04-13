cmake_minimum_required(VERSION 2.8.3)
project(JMVIO)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

set(CMAKE_MODULE_PATH ${PROECT_SOURCE_DIR}/cmake)

find_package(Eigen3 REQUIRED)
find_package(Ceres REQUIRED)
find_package(Pangolin REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem program_options system)

include_directories(
        ${PROJECT_SOURCE_DIR}/include
        ${EIGEN3_INCLUDE_DIR}
        ${CERES_INCLUDE_DIRS}
        ${Pangolin_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIRS}
)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)


add_library(camera_model SHARED
        src/camera_models/chessboard/Chessboard.cpp
        )

add_library(JmVio SHARED
        src/System.cpp
        src/parameters.cpp
        src/estimator.cpp
        src/feature_manager.cpp
        src/feature_tracker.cpp


        src/utility/utility.cpp

        src/initial/initial_ex_rotation.cpp
        src/initial/initial_sfm.cpp
        src/initial/solve_5pts.cpp
        src/initial/initial_aligment.cpp


        src/backend/loss_function.cpp
        src/backend/imu_integration.cpp
        src/backend/vertex.cpp
        src/backend/edge.cpp
        src/backend/vertex_pose.cpp
        src/backend/edge_reprojection.cpp
        src/backend/edge_imu.cpp
        src/backend/edge_prior.cpp
)

target_link_libraries(JmVio
        camera_model
        ${OpenCV_LIBS}
        ${CERES_LIBRARIES}
        ${Pangolin_LIBRARIES}
)

add_executable(run_euroc test/run_euroc.cpp)
target_link_libraries(run_euroc
        JmVio
        -lpthread)

add_executable(testCurveFitting test/CurveFitting.cpp)
target_link_libraries(testCurveFitting JmVio)

