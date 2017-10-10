# CMake Configuration for the cleaning components of libattpc
# ===========================================================
#
# This is a sub-component of libattpc, and it is NOT set up to be built or used
# independently of the complete library! Please see the master libattpc repository
# for build instructions for the complete library.

#################################
# Directory-level configuration #
#################################

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")

#################
# Library setup #
#################

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
target_include_directories(attpc_cleaning PUBLIC include)
target_include_directories(attpc_cleaning SYSTEM PUBLIC ${PCL_INCLUDE_DIRS})
target_link_libraries(attpc_cleaning ${PCL_LIBRARIES})
target_compile_options(attpc_cleaning PRIVATE ${WARNING_FLAGS})
set_property(TARGET attpc_cleaning PROPERTY CXX_STANDARD 14)

# install(
#     TARGETS attpc_cleaning
#     EXPORT attpc_cleaning_targets
#     LIBRARY DESTINATION lib
# )
# install(
#     DIRECTORY include/
#     DESTINATION include/attpc/cleaning
#     FILES_MATCHING PATTERN "*.h"
# )
# install(
#     EXPORT attpc_cleaning_targets
#     NAMESPACE attpc::
#     DESTINATION lib/attpc
# )

###################
# Unit test setup #
###################

set(TEST_SRC_FILES
    test/catch_main.cpp
    test/test_utilities.cpp
    test/test_metrics.cpp
    test/test_HoughSpace.cpp
    test/test_LinearHoughTransform.cpp
    test/test_CircularHoughTransform.cpp
    test/test_HoughSpiralCleaner.cpp
    test/test_neighbors.cpp
)

add_executable(test_attpc_cleaning ${TEST_SRC_FILES})
target_link_libraries(test_attpc_cleaning Catch attpc_cleaning)
target_compile_options(test_attpc_cleaning PRIVATE ${WARNING_FLAGS})
set_property(TARGET test_attpc_cleaning PROPERTY CXX_STANDARD 14)

###################
# Profiling setup #
###################

add_executable(prof_LinearHoughTransform EXCLUDE_FROM_ALL prof/prof_LinearHoughTransform.cpp)
target_link_libraries(prof_LinearHoughTransform attpc_cleaning)
target_compile_options(prof_LinearHoughTransform PRIVATE ${WARNING_FLAGS})
set_property(TARGET prof_LinearHoughTransform PROPERTY CXX_STANDARD 14)
