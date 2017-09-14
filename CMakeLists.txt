cmake_minimum_required(VERSION 3.1)

project(attpc_mergers)

# Set up warnings
if ("${CMAKE_CXX_COMPILER_ID}" MATCHES "Clang")
    set(WARNING_FLAGS -Weverything -Wno-c++98-compat -Wno-padded -Wno-weak-vtables -Wno-exit-time-destructors)
elseif("${CMAKE_CXX_COMPILER_ID}" MATCHES "GNU")
    set(WARNING_FLAGS -Wall -pedantic -Wextra -Wfloat-equal -Wpointer-arith -Wcast-qual)
endif()

include_directories(include)

# Set up library
set(SRC_FILES
    src/GRAWFile.cpp
    src/GRAWFrame.cpp
    src/GRAWHeader.cpp
)

add_library(attpc_mergers SHARED ${SRC_FILES})
target_compile_options(attpc_mergers PRIVATE ${WARNING_FLAGS})
set_property(TARGET attpc_mergers PROPERTY CXX_STANDARD 14)

# Set up unit tests

add_library(Catch INTERFACE)
target_include_directories(Catch INTERFACE libs/catch)

set(TEST_SRC_FILES
    test/catch_main.cpp
    test/test_RawFrame.cpp
    test/test_utilities.cpp
)

add_executable(test_attpc_mergers ${TEST_SRC_FILES})
target_link_libraries(test_attpc_mergers Catch attpc_mergers)
target_compile_options(test_attpc_mergers PRIVATE ${WARNING_FLAGS})
set_property(TARGET test_attpc_mergers PROPERTY CXX_STANDARD 14)
