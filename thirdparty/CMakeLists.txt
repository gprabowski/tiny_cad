add_library(imgui
    #Main imgui files
    imgui/imgui.cpp imgui/imgui_draw.cpp
    imgui/imgui_tables.cpp imgui/imgui_widgets.cpp
    # GLFW+OpenGL - specific files
    imgui/backends/imgui_impl_glfw.cpp
    imgui/backends/imgui_impl_opengl3.cpp
    #cpp
    imgui/misc/cpp/imgui_stdlib.cpp
    # demo window
    imgui/imgui_demo.cpp)

add_library(glad glad/glad.c)
add_library(imguizmo 
    ImGuizmo/GraphEditor.cpp
    ImGuizmo/ImCurveEdit.cpp
    ImGuizmo/ImGradient.cpp
    ImGuizmo/ImSequencer.cpp
    ImGuizmo/ImGuizmo.cpp
    ImGuizmo/GraphEditor.h
    ImGuizmo/ImCurveEdit.h
    ImGuizmo/ImGradient.h
    ImGuizmo/ImGuizmo.h
    ImGuizmo/ImSequencer.h
    ImGuizmo/ImZoomSlider.h 
)

target_include_directories(imgui PUBLIC imgui imgui/backends glfw/include)

target_include_directories(imguizmo PUBLIC imgui imgui/backends glfw/include)

target_include_directories(glad PUBLIC glfw/include/)

#GLFW
set(GLFW_BUILD_DOCS OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)

set(GLFW_USE_WAYLAND ON CACHE BOOL "Use Wayland for window creation" FORCE)
set(GLFW_BUILD_X11 OFF CACHE BOOL "Turn off X11 support" FORCE)

add_subdirectory(glfw)
add_subdirectory(spdlog)
add_subdirectory(glm)
