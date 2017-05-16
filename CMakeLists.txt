project(ExtendedKF)

cmake_minimum_required (VERSION 3.5)

add_definitions(-std=c++0x)

set(sources
    src/FusionEKF.cpp
    src/FusionEKF.h
    src/kalman_filter.cpp
    src/kalman_filter.h
    src/main.cpp
    src/ground_truth_package.h
    src/tools.cpp
    src/tools.h
    src/laser_updater.cpp
    src/laser_updater.h
    src/radar_updater.cpp
    src/radar_updater.h
    )

add_executable(ExtendedKF ${sources})
