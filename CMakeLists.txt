# CMake Configuration for libattpc
# ================================
#
# This defines the build steps for the complete libattpc library,
# including all components.

cmake_minimum_required(VERSION 3.1)

project(libattpc)

#####################
# Find dependencies #
#####################

# Find all packages here, but set include directories and link to libraries at
# the level of individual targets.

find_package(OpenMP REQUIRED)
find_package(Boost 1.63 REQUIRED COMPONENTS program_options)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(HDF5 1.10 REQUIRED COMPONENTS CXX)
find_package(PCL 1.2 REQUIRED COMPONENTS common kdtree io)

########################
# Set up warning flags #
########################

# Flags are set in a variable called WARNING_FLAGS. This can be applied to individual
# targets using target_compile_options. This way, flags can be overridden for individual
# targets if necessary.

if ("${CMAKE_CXX_COMPILER_ID}" MATCHES "Clang")
    set(WARNING_FLAGS -Weverything -Wno-c++98-compat -Wno-padded -Wno-weak-vtables -Wno-exit-time-destructors)
elseif("${CMAKE_CXX_COMPILER_ID}" MATCHES "GNU")
    set(WARNING_FLAGS -Wall -pedantic -Wextra -Wfloat-equal -Wpointer-arith -Wcast-qual)
endif()

##################
# Add components #
##################

add_subdirectory(external)  # Third-party dependencies

add_subdirectory(common)
add_subdirectory(mergers)
add_subdirectory(cleaning)
