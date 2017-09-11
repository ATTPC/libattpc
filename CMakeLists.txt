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
)

add_library(attpc_mergers SHARED ${SRC_FILES})
target_compile_options(attpc_mergers PRIVATE ${WARNING_FLAGS})
set_property(TARGET attpc_mergers PROPERTY CXX_STANDARD 14)
