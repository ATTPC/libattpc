cmake_minimum_required(VERSION 3.1)

project(attpc_common)

# Set up warnings
if ("${CMAKE_CXX_COMPILER_ID}" MATCHES "Clang")
    set(WARNING_FLAGS -Weverything -Wno-c++98-compat -Wno-padded -Wno-weak-vtables -Wno-exit-time-destructors)
elseif("${CMAKE_CXX_COMPILER_ID}" MATCHES "GNU")
    set(WARNING_FLAGS -Wall -pedantic -Wextra -Wfloat-equal -Wpointer-arith -Wcast-qual)
endif()

include_directories(include)

# Find Boost
find_package(Boost 1.63 REQUIRED)
include_directories(${Boost_INCLUDE_DIRS})

# Set up YAML-CPP dependency
add_subdirectory(libs/yaml-cpp)
include_directories(SYSTEM libs/yaml-cpp/include)

# Build library
set(SRC_FILES
    src/ConfigStore.cpp
)

add_library(attpc_common SHARED ${SRC_FILES})
target_link_libraries(attpc_common yaml-cpp)
target_compile_options(attpc_common PRIVATE ${WARNING_FLAGS})
set_property(TARGET attpc_common PROPERTY CXX_STANDARD 14)

# Set up unit tests

add_library(Catch INTERFACE)
target_include_directories(Catch INTERFACE libs/catch)

set(TEST_SRC_FILES
    test/catch_main.cpp
    test/test_ConfigStore.cpp
)

add_executable(test_attpc_common ${TEST_SRC_FILES})
target_link_libraries(test_attpc_common Catch attpc_common)
target_compile_options(test_attpc_common PRIVATE ${WARNING_FLAGS})
set_property(TARGET test_attpc_common PROPERTY CXX_STANDARD 14)

configure_file(${CMAKE_CURRENT_SOURCE_DIR}/test/data/test_config.yml ${CMAKE_CURRENT_BINARY_DIR}/data/test_config.yml COPYONLY)

# Set up target for documentation

# find_package(Doxygen 1.8)
# if(${DOXYGEN_FOUND})
#     set(DOXYFILE_IN ${CMAKE_CURRENT_SOURCE_DIR}/doc/Doxyfile.in)
#     set(DOXYFILE_OUT ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile)
#
#     configure_file(${DOXYFILE_IN} ${DOXYFILE_OUT} @ONLY)
#
#     add_custom_target(doc
#         COMMAND doxygen
#         WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
#         COMMENT "Building documentation"
#     )
# else()
#     message(WARNING "Doxygen is required to build the documentation.")
# endif()
