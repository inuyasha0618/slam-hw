add_compile_options(-std=c++11)
#find_package(Sophus REQUIRED)
INCLUDE_DIRECTORIES("/usr/include/eigen3")
#INCLUDE_DIRECTORIES(${Sophus_INCLUDE_DRIS})
set(Sophus_LIBRARIES /home/slam/Sophus/libSophus.so)
include_directories(/home/slam/Sophus)
add_executable(GN-BA GN-BA.cpp)
target_link_libraries(GN-BA ${Sophus_LIBRARIES})
