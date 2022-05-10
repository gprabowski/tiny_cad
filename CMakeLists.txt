cmake_minimum_required(VERSION 3.23)
project(cad)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_program(CCACHE_FOUND ccache)
if(CCACHE_FOUND)
    message("CCACHE FOUND, SETTING COMPILER CACHING")
    set_property(GLOBAL PROPERTY RULE_LAUNCH_COMPILE ccache)
endif(CCACHE_FOUND)

set(TC_DEBUG TRUE)

if((MSVC) AND (TC_DEBUG))
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /Zi")
elseif(${TC_DEBUG})
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g")
endif()

message("ADDING THIRDPARTY LIBRARIES")
add_subdirectory(thirdparty)
message("ADDING TINYCAD EXECUTABLE")
add_subdirectory(src)


