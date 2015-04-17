cmake_minimum_required(VERSION 2.8.3)
project(ik_marker)


find_package(catkin_simple REQUIRED)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

catkin_simple()
find_package(Eigen REQUIRED)
include_directories(${Eigen_INCLUDE_DIRS})
add_definitions(${EIGEN_DEFINITIONS})


# example boost usage
# find_package(Boost REQUIRED COMPONENTS system thread)

# C++0x support - not quite the same as final C++11!
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")

# Libraries
# cs_add_libraries(my_lib src/my_lib.cpp)   

## Declare a cpp executable
cs_add_executable(ik_marker_node_1 src/marker_example.cpp)
cs_add_executable(reachability_from_above_node src/reachability_from_above_node.cpp)
cs_add_executable(ik_marker_node src/ik_marker_node.cpp)

# target_link_library(example my_lib)
 
cs_install()
cs_export()
