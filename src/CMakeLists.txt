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
find_program( CLANG_TIDY_EXE NAMES "clang-tidy" DOC "Path to clang-tidy executable" ) 
if(NOT CLANG_TIDY_EXE) 
    message(STATUS "clang-tidy not found.") 
else() 
    message(STATUS "clang-tidy found: ${CLANG_TIDY_EXE}") 
    message(STATUS "clang-tidy turned off")
    #set(DO_CLANG_TIDY "${CLANG_TIDY_EXE}" "-checks=*,-clang-analyzer-alpha.*") 
endif()
set_target_properties( tiny_cad PROPERTIES CXX_STANDARD 17 CXX_STANDARD_REQUIRED ON )
if(CLANG_TIDY_EXE) 
    set_target_properties( tiny_cad PROPERTIES CXX_CLANG_TIDY "${DO_CLANG_TIDY}" ) 
endif()

target_compile_options(tiny_cad PUBLIC -Werror -Wall)
target_compile_features(tiny_cad PUBLIC cxx_std_17)
set_target_properties(tiny_cad PROPERTIES
    CXX_STANDARD 17 )

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
