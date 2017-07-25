cmake_minimum_required(VERSION 3.1)

project(attpc_clustering)

include(CMakeToolsHelpers OPTIONAL)
find_package(PCL 1.2 REQUIRED COMPONENTS common)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

include_directories(include)

set(SRC_FILES
    src/Cluster.cpp
    src/utilities.cpp
    src/HTripletClustering.cpp
    src/metrics.cpp
)

add_library(attpc_clustering ${SRC_FILES})
target_link_libraries(attpc_clustering ${PCL_LIBRARIES})
set_property(TARGET attpc_clustering PROPERTY CXX_STANDARD 14)

# Set up unit tests
set(TEST_SRC_FILES
    test/catch_main.cpp
)

add_executable(test_attpc_clustering ${TEST_SRC_FILES})
target_link_libraries(test_attpc_clustering attpc_clustering)
set_property(TARGET test_attpc_clustering PROPERTY CXX_STANDARD 14)