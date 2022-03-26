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
    rendering_systems.cpp
    log.cpp
    )

add_executable(tiny_cad)
target_sources(tiny_cad PUBLIC ${SOURCES})

target_include_directories(tiny_cad PUBLIC ../thirdparty/glm/glm)
target_include_directories(tiny_cad PUBLIC ../thirdparty)
target_include_directories(tiny_cad PUBLIC ../thirdparty/glfw/include/)
target_include_directories(tiny_cad PUBLIC ../thirdparty/ImGuizmo/)
target_include_directories(tiny_cad PUBLIC ../thirdparty/spdlog/include/)
target_include_directories(tiny_cad PUBLIC ../include)

message(STATUS "<<${OPENGL_INCLUDE_DIRS}>>")

target_link_libraries(tiny_cad glad)
target_link_libraries(tiny_cad spdlog)
target_link_libraries(tiny_cad glfw)
target_link_libraries(tiny_cad glm)
target_link_libraries(tiny_cad imgui)
target_link_libraries(tiny_cad imguizmo)

add_custom_command(TARGET tiny_cad PRE_BUILD
                   COMMAND ${CMAKE_COMMAND} -E create_symlink
                       ${CMAKE_SOURCE_DIR}/resources/ $<TARGET_FILE_DIR:tiny_cad>/resources)
