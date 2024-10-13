cmake_minimum_required(VERSION 3.5)

set(CMAKE_CXX_COMPILER "C:/MinGW/bin/g++.exe")
set(CMAKE_C_COMPILER "C:/MinGW/bin/gcc.exe")

project(ImageDetection)

find_package(OpenCV REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(ImageDetection main.cpp)

target_link_libraries(ImageDetection ${OpenCV_LIBS})