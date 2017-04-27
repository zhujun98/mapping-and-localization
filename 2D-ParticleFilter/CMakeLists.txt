cmake_minimum_required(VERSION 3.6)
project(2D_ParticleFilter)

set(CMAKE_CXX_STANDARD 11)

set(SOURCE_FILES
        src/main.cpp
        src/map.h
        src/particle_filter.cpp
        src/particle_filter.h
        src/utilities.h)

add_executable(2D_ParticleFilter ${SOURCE_FILES})