cmake_minimum_required( VERSION 2.8 )
project( geometry )

# 添加Eigen头文件
include_directories( "/usr/include/eigen3" )

add_executable(exercise3 exercise3.cpp)

add_executable(exercise2_5 exercise2_5.cpp)
