message("ADDING DEAR IMGUI")
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

message("ADDING GLAD")
add_library(glad glad/glad.c)

message("ADDING IMGUIZMO")
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

message("ADDING IMPLOT")
add_library(implot
    implot/implot.h
    implot/implot_internal.h
    implot/implot.cpp
    implot/implot_demo.cpp
    implot/implot_items.cpp
)


target_include_directories(imgui PUBLIC imgui imgui/backends glfw/include)

target_include_directories(imguizmo PUBLIC imgui imgui/backends glfw/include)

target_include_directories(glad PUBLIC glfw/include/)

target_include_directories(implot PUBLIC imgui imgui/backends implot)

#GLFW
message("GLFW DOCS BUILD TURNED OFF")
set(GLFW_BUILD_DOCS OFF CACHE BOOL "" FORCE)
message("GLFW TESTS BUILD TURNED OFF")
set(GLFW_BUILD_TESTS OFF CACHE BOOL "" FORCE)
message("GLFW EXAMPLES BUILD TURNED OFF")
set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)

message("GLFW WAYLAND BUILD TURNED ON")
set(GLFW_USE_WAYLAND ON CACHE BOOL "Use Wayland for window creation" FORCE)
message("GLFW X11 BUILD TURNED ON")
set(GLFW_BUILD_X11 OFF CACHE BOOL "Turn off X11 support" FORCE)

message(" ADDING GLFW")
add_subdirectory(glfw)
message(" ADDING SPDLOG")
add_subdirectory(spdlog)
message(" ADDING GLM")
add_subdirectory(glm)
