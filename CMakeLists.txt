cmake_minimum_required(VERSION 3.23)
project(cad)

find_program(CCACHE_FOUND ccache)
if(CCACHE_FOUND)
    message("CCACHE FOUND, SETTING COMPILER CACHING")
    set_property(GLOBAL PROPERTY RULE_LAUNCH_COMPILE ccache)
endif(CCACHE_FOUND)

message("ADDING THIRDPARTY LIBRARIES")
add_subdirectory(thirdparty)
message("ADDING TINYCAD EXECUTABLE")
add_subdirectory(src)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
