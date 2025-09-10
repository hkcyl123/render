cmake_minimum_required(VERSION 4.0)
project(render)

set(CMAKE_CXX_STANDARD 20)

add_executable(render main.cpp
        tgaimage.cpp
        tgaimage.h)
