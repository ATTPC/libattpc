cmake_minimum_required(VERSION 3.1)

project(libattpc)

# This macro gets a list of all submodules in the libs directory and stores it to its first argument
macro(find_submodules result)
    # Based on https://stackoverflow.com/a/7788165/3820658
    file(GLOB children RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/libs/*)
    set(dirlist "")
    foreach(child ${children})
        if(IS_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/${child})
            list(APPEND dirlist ${child})
        endif()
    endforeach()
    set(${result} ${dirlist})
endmacro()

# Now find all submodules and add them

find_submodules(submodule_list)

foreach(submod ${submodule_list})
    message(STATUS "Configuring module ${submod}")
    add_subdirectory(${submod})
endforeach()

