cmake_minimum_required(VERSION 3.23)
project(cpp_gl_second)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

#GLFW
set(GLFW_BUILD_DOCS OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)

set(GLFW_USE_WAYLAND ON CACHE BOOL "Use Wayland for window creation" FORCE)
set(GLFW_BUILD_X11 OFF CACHE BOOL "Turn off X11 support" FORCE)

add_subdirectory(thirdparty/glfw)

add_library(imgui
    #Main imgui files
    thirdparty/imgui/imgui.cpp thirdparty/imgui/imgui_draw.cpp
    thirdparty/imgui/imgui_tables.cpp thirdparty/imgui/imgui_widgets.cpp
    # GLFW+OpenGL - specific files
    thirdparty/imgui/backends/imgui_impl_glfw.cpp
    thirdparty/imgui/backends/imgui_impl_opengl3.cpp

    #cpp
    thirdparty/imgui/misc/cpp/imgui_stdlib.cpp

    # demo window
    thirdparty/imgui/imgui_demo.cpp)

add_library(glad thirdparty/glad/glad.c)
add_library(imguizmo 
    thirdparty/ImGuizmo/GraphEditor.cpp
    thirdparty/ImGuizmo/GraphEditor.h
    thirdparty/ImGuizmo/ImCurveEdit.cpp
    thirdparty/ImGuizmo/ImCurveEdit.h
    thirdparty/ImGuizmo/ImGradient.cpp
    thirdparty/ImGuizmo/ImGradient.h
    thirdparty/ImGuizmo/ImGuizmo.cpp
    thirdparty/ImGuizmo/ImGuizmo.h
    thirdparty/ImGuizmo/ImSequencer.cpp
    thirdparty/ImGuizmo/ImSequencer.h
    thirdparty/ImGuizmo/ImZoomSlider.h 
)
target_include_directories(imguizmo PUBLIC thirdparty/imgui thirdparty/imgui/backends thirdparty/glfw/include)

target_include_directories(glad PUBLIC thirdparty/glfw/include/)

set(SOURCES
    src/main.cpp
    src/gui.cpp
    src/callbacks.cpp
    src/init_gl.cpp
    src/shader.cpp
    src/systems.cpp
    src/component_manager.cpp
    src/constructors.cpp
    src/input_handlers.cpp
    src/rendering_systems.cpp
    include/app_state.h
    include/gl_object.h
    include/init_gl.h
    include/torus.h
    include/callbacks.h
    include/gui.h
    include/parametric.h
    include/transformation.h
    include/ecs.h
    include/component_manager.h
    )



add_executable(prog ${SOURCES})

add_custom_command(TARGET prog PRE_BUILD
                   COMMAND ${CMAKE_COMMAND} -E create_symlink
                       ${CMAKE_SOURCE_DIR}/resources/ $<TARGET_FILE_DIR:prog>/resources)

target_include_directories(prog PUBLIC thirdparty)
target_include_directories(prog PUBLIC include)
target_include_directories(prog PUBLIC thirdparty/glfw/include/)
target_include_directories(prog PUBLIC thirdparty/ImGuizmo/)

target_include_directories(imgui PUBLIC thirdparty/imgui thirdparty/imgui/backends thirdparty/glfw/include)

message(STATUS "<<${OPENGL_INCLUDE_DIRS}>>")

target_link_libraries(prog glad)
target_link_libraries(prog glfw)
target_link_libraries(prog imgui)
target_link_libraries(prog imguizmo)
