cmake_minimum_required(VERSION 3.1)

project(cloud_viewer)


include(CMakeToolsHelpers OPTIONAL)
find_package(PCL 1.2 REQUIRED COMPONENTS common)
find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

include_directories(include)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_library(clustering src/cluster.cpp src/hc.cpp src/mst.cpp src/smoothenCloud.cpp)
target_link_libraries(clustering ${PCL_LIBRARIES})
set_property(TARGET clustering PROPERTY CXX_STANDARD 14)
