add_subdirectory(catch)

set(YAML_CPP_BUILD_TOOLS OFF CACHE BOOL "Enable testing and parse tools")
set(YAML_CPP_BUILD_CONTRIB OFF CACHE BOOL "Enable contrib stuff in library")
add_subdirectory(yaml-cpp)
include_directories(SYSTEM yaml-cpp/include)
