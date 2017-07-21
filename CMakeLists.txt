cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(cloud_viewer)

include(CMakeToolsHelpers OPTIONAL)
find_package(PCL 1.2 REQUIRED)
find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (cloud_viewer src/cluster.cpp src/hc.cpp src/main.cpp src/mst.cpp src/smoothenCloud.cpp)
target_link_libraries (cloud_viewer ${PCL_LIBRARIES})
