# CMake Configuration for the common components of libattpc
# =========================================================
#
# This is a sub-component of libattpc, and it is NOT set up to be built or used
# independently of the complete library! Please see the master libattpc repository
# for build instructions for the complete library.

#################
# Library setup #
#################

set(SRC_FILES
    src/ConfigStore.cpp
    src/HDF5DataFile.cpp
    src/Trace.cpp
    src/FullTraceEvent.cpp
)

add_library(attpc_common SHARED ${SRC_FILES})
target_include_directories(attpc_common PUBLIC include)
target_include_directories(attpc_common SYSTEM PUBLIC ${Boost_INCLUDE_DIRS} ${HDF5_INCLUDE_DIRS})
target_link_libraries(attpc_common yaml-cpp Eigen3::Eigen ${HDF5_CXX_LIBRARIES})
target_compile_options(attpc_common PRIVATE ${WARNING_FLAGS})
set_property(TARGET attpc_common PROPERTY CXX_STANDARD 14)

# install(
#     TARGETS attpc_common
#     EXPORT attpc_common_targets
#     LIBRARY DESTINATION lib
# )
# install(
#     DIRECTORY include/
#     DESTINATION include/attpc/common
#     FILES_MATCHING PATTERN "*.h"
# )
# install(
#     EXPORT attpc_common_targets
#     NAMESPACE attpc::
#     DESTINATION lib/attpc
# )

###################
# Unit test setup #
###################

set(TEST_SRC_FILES
    test/catch_main.cpp
    test/test_ConfigStore.cpp
    test/test_yaml_conversions.cpp
    test/test_HardwareAddress.cpp
    test/test_Trace.cpp
    test/test_FullTraceEvent.cpp
    test/test_HDF5DataFile.cpp
)

add_executable(test_attpc_common ${TEST_SRC_FILES})
target_link_libraries(test_attpc_common Catch attpc_common)
target_compile_options(test_attpc_common PRIVATE ${WARNING_FLAGS})
set_property(TARGET test_attpc_common PROPERTY CXX_STANDARD 14)

configure_file(${CMAKE_CURRENT_SOURCE_DIR}/test/data/test_config.yml ${CMAKE_CURRENT_BINARY_DIR}/data/test_config.yml COPYONLY)

#######################
# Documentation setup #
#######################

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
