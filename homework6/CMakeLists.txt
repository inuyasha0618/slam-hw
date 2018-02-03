cmake_minimum_required(VERSION 2.8)

set(CMAKE_CXX_FLAGS "-std=c++11")

find_package(OpenCV 3 REQUIRED)
INCLUDE_DIRECTORIES(${OpenCV_INCLUDE_DRIS})
include_directories(/usr/local/include/eigen3)
set(Sophus_LIBRARIES /home/slam/Sophus/libSophus.so)
include_directories(/home/slam/Sophus)

find_package(Pangolin 0.4 REQUIRED)
include_directories(${Pangolin_INCLUDE_DIRS})

add_executable(optical_flow optical_flow.cpp)
add_executable(direct_method direct_method3.cpp)
target_link_libraries(optical_flow ${OpenCV_LIBS})
target_link_libraries(direct_method ${OpenCV_LIBS})
target_link_libraries(direct_method ${Sophus_LIBRARIES})
target_link_libraries(direct_method ${Pangolin_LIBRARIES})
