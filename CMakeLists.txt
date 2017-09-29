# CMake Configuration for the merging components of libattpc
# ==========================================================
#
# This is a sub-component of libattpc, and it is NOT set up to be built or used
# independently of the complete library! Please see the master libattpc repository
# for build instructions for the complete library.

#################
# Library setup #
#################

set(SRC_FILES
    src/GRAWFile.cpp
    src/GRAWFrame.cpp
    src/GRAWHeader.cpp
    src/Merger.cpp
    src/FrameAccumulator.cpp
)

add_library(attpc_mergers SHARED ${SRC_FILES})
target_include_directories(attpc_mergers PUBLIC include)
target_link_libraries(attpc_mergers attpc_common)
target_compile_options(attpc_mergers PRIVATE ${WARNING_FLAGS})
set_property(TARGET attpc_mergers PROPERTY CXX_STANDARD 14)

###################
# Unit test setup #
###################

set(TEST_SRC_FILES
    test/catch_main.cpp
    test/test_RawFrame.cpp
    test/test_utilities.cpp
    test/test_GRAWHeaderField.cpp
    test/test_GRAWFile.cpp
)

add_executable(test_attpc_mergers ${TEST_SRC_FILES})
target_link_libraries(test_attpc_mergers Catch attpc_mergers)
target_compile_options(test_attpc_mergers PRIVATE ${WARNING_FLAGS})
set_property(TARGET test_attpc_mergers PROPERTY CXX_STANDARD 14)

configure_file(test/data/partial_readout_frame.graw test_data/partial_readout_frame.graw COPYONLY)
