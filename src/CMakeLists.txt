set(SOURCES
    main.cpp
    gui.cpp
    callbacks.cpp
    init.cpp
    shader.cpp
    systems.cpp
    component_manager.cpp
    constructors.cpp
    input_handlers.cpp
    log.cpp
    )

add_executable(tiny_cad)

target_compile_options(tiny_cad PRIVATE -Werror -Wall)
target_compile_features(tiny_cad PRIVATE cxx_std_17)

target_sources(tiny_cad PUBLIC ${SOURCES})

message("CMAKE ROOT IS ${CMAKE_SOURCE_DIR}")
target_include_directories(tiny_cad 
    PUBLIC 
    ${CMAKE_SOURCE_DIR}/thirdparty/glm/glm
    ${CMAKE_SOURCE_DIR}/thirdparty
    ${CMAKE_SOURCE_DIR}/thirdparty/glfw/include/
    ${CMAKE_SOURCE_DIR}/thirdparty/ImGuizmo/
    ${CMAKE_SOURCE_DIR}/thirdparty/spdlog/include/
    ${CMAKE_SOURCE_DIR}/include
 )

message(STATUS "<<${OPENGL_INCLUDE_DIRS}>>")

target_link_libraries(tiny_cad 
        glad
        spdlog
        glfw
        glm
        imgui
        imguizmo
)

add_custom_command(TARGET tiny_cad PRE_BUILD
                   COMMAND ${CMAKE_COMMAND} -E create_symlink
                       ${CMAKE_SOURCE_DIR}/resources/ $<TARGET_FILE_DIR:tiny_cad>/resources)
