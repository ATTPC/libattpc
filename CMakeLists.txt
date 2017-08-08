cmake_minimum_required(VERSION 3.1)

project(attpc_cleaning)

include(CMakeToolsHelpers OPTIONAL)
find_package(PCL 1.2 REQUIRED COMPONENTS common kdtree io)
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
    src/HoughTransform.cpp
    src/LinearHoughTransform.cpp
    src/CircularHoughTransform.cpp
    src/metrics.cpp
    src/HoughSpiralCleaner.cpp
)

add_library(attpc_cleaning SHARED ${SRC_FILES})
target_link_libraries(attpc_cleaning ${PCL_LIBRARIES})
set_property(TARGET attpc_cleaning PROPERTY CXX_STANDARD 14)

# Set up unit tests

add_library(Catch INTERFACE)
target_include_directories(Catch INTERFACE libs/catch)

set(TEST_SRC_FILES
    test/catch_main.cpp
    test/test_utilities.cpp
    test/test_metrics.cpp
    test/test_LinearHoughTransform.cpp
    test/test_CircularHoughTransform.cpp
    test/test_HoughSpiralCleaner.cpp
)

add_executable(test_attpc_cleaning EXCLUDE_FROM_ALL ${TEST_SRC_FILES})
target_link_libraries(test_attpc_cleaning Catch attpc_cleaning)
set_property(TARGET test_attpc_cleaning PROPERTY CXX_STANDARD 14)

# Set up target for documentation

add_custom_target(doc
    COMMAND doxygen
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/doc
)
