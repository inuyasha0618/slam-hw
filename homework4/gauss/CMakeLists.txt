cmake_minimum_required(VERSION 2.8)
project(gauss)

#set(CMAKE_CXX_FLAGS "-std=c++11")
find_package(OpenCV 3 REQUIRED)

include_directories("/usr/include/eigen3")
include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(gaussnewton gaussnewton.cpp)

target_link_libraries(gaussnewton ${OpenCV_LIBS})
