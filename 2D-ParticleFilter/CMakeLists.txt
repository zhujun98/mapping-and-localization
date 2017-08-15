cmake_minimum_required(VERSION 3.6)
project(2D_ParticleFilter)

set(CMAKE_CXX_STANDARD 11)

set(SOURCE_FILES
        src/main.cpp
        src/particle_filter.cpp
        src/map.cpp)

add_executable(2D_ParticleFilter ${SOURCE_FILES})