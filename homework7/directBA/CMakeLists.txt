cmake_minimum_required( VERSION 2.8 )
project( directBA )

#set( CMAKE_BUILD_TYPE "Release" )
set( CMAKE_CXX_FLAGS "-std=c++11 -O3" )

list( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules )

# 寻找G2O
find_package( G2O REQUIRED )
include_directories(
        ${G2O_INCLUDE_DIR}
        "/usr/include/eigen3"
)

find_package(Pangolin 0.4 REQUIRED)
include_directories(${Pangolin_INCLUDE_DIRS})

#Sophus
set(Sophus_LIBRARIES /home/slam/Sophus/libSophus.so)
include_directories(/home/slam/Sophus)

SET(G2O_LIBS g2o_cli g2o_types_sba g2o_core g2o_interface
        g2o_solver_csparse g2o_solver_dense
        g2o_stuff  g2o_parser g2o_types_data )

# OpenCV
find_package( OpenCV 3 REQUIRED )
include_directories( ${OpenCV_DIRS} )

add_executable( directBA directBA.cpp )
# 与G2O和OpenCV链接
target_link_libraries( directBA
        ${OpenCV_LIBS}
        ${G2O_LIBS}
        ${Sophus_LIBRARIES}
        ${Pangolin_LIBRARIES}
        )