#pragma once

#include <memory>

#include <glad/glad.h>

#include <app_state.h>
#include <registry.h>

namespace constructors {
ecs::EntityType add_cursor(ecs::registry &reg, transformation &&_t,
                           gl_object &&_g);

gl_object get_cursor_geometry(const GLint program);

void setup_initial_geometry(ecs::registry &reg, GLuint program);

ecs::EntityType add_point(ecs::registry &reg, transformation &&_t,
                          const GLuint program);

ecs::EntityType add_torus(ecs::registry &reg, parametric &&_p,
                          transformation &&_t, torus_params &&_tp,
                          const GLuint program);

ecs::EntityType add_bezier(ecs::registry &reg, std::shared_ptr<app_state> &s,
                           const GLuint program);

ecs::EntityType add_bspline(ecs::registry &reg, std::shared_ptr<app_state> &s,
                            const GLuint program);
} // namespace constructors
