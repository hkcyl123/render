cmake_minimum_required(VERSION 4.0)
project(render)

set(CMAKE_CXX_STANDARD 20)

include_directories(/usr/local/include)

include_directories("/opt/homebrew/Cellar/opencv/4.12.0_4/include")
include_directories("/opt/homebrew/Cellar/eigen/3.4.0_1/include/eigen3")
link_directories("/opt/homebrew/Cellar/opencv/4.12.0_4/lib")

find_package(OpenCV REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})



add_executable(render main.cpp
        tgaimage.cpp
        tgaimage.h
        model.cpp
        model.h
)

target_link_libraries(render ${OpenCV_LIBS})
