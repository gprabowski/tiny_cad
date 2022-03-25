#pragma once

#include <glad/glad.h>

#include <component_manager.h>

namespace constructors {
ecs::EntityType add_cursor(ecs::component_manager &cm, transformation &&_t,
                           gl_object &&_g);

gl_object get_cursor_geometry(const GLint program);

void setup_initial_geometry(ecs::component_manager &cm, GLuint program);

ecs::EntityType add_point(ecs::component_manager &cm, transformation &&_t,
                          const GLuint program);

ecs::EntityType add_torus(ecs::component_manager &cm, parametric &&_p,
                          transformation &&_t, torus_params &&_tp,
                          const GLuint program);
} // namespace constructors
