cmake_minimum_required(VERSION 2.8.3)
project(object_finder)

find_package(catkin_simple REQUIRED)

find_package(PCL 1.7 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package(Eigen REQUIRED)
include_directories(${Eigen_INCLUDE_DIRS})

catkin_simple()

# example boost usage
# find_package(Boost REQUIRED COMPONENTS system thread)

# C++0x support - not quite the same as final C++11!
# need to specify no SSE because Eigen/GCC will generate invalid assembly instruction. :(
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -mno-sse")

# Libraries
# cs_add_libraries(my_lib src/my_lib.cpp)   

# Executables
cs_add_executable(object_finder src/object_finder.cpp)
# target_link_library(example my_lib)

target_link_libraries(object_finder ${PCL_LIBRARIES})

cs_install()
cs_export()
    
