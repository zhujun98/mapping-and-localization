cmake_minimum_required(VERSION 3.6)
project(BayesianFilter)

set(CMAKE_CXX_STANDARD 11)

set(SOURCE_FILES
        main.cpp
        bayesian_filter.cpp
        bayesian_filter.h
        help_functions.h
        map.h
        measurement_package.h)

add_executable(BayesianFilter ${SOURCE_FILES})